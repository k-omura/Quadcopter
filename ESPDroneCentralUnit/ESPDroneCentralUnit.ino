/*
   Drone central controler

   Created on December 4, 2017

   December 4, 2017: MPU6050 DMP
   December 16, 2017: PID test
   January 21, 2018: Organize programme

   ESP8266
   MPU6050
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <FS.h>

//class default I2C address is 0x68
//specific I2C addresses may be passed as a parameter here
//AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
//AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); //<-- use for AD0 high

#define MPU6050_INTERRUPT_PIN 15

//union transmit USART data
union eusartTransmit {
  unsigned char raw;

  struct split {
    unsigned int address : 2;
    unsigned int data : 6;
  } split;
} eusartTransmit;

typedef struct {
  double x;
  double y;
} xydetaset;

//vars MPU control/status
bool dmpReady = false; //set true if DMP init was successful
uint8_t mpuIntStatus; //holds actual interrupt status byte from MPU
uint8_t devStatus; //return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; //expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; //count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; //FIFO storage buffer

//vars orientation/motion
Quaternion q; //[w, x, y, z] quaternion container
VectorFloat gravity; //[x, y, z] gravity vector
float euler[3]; //[psi, theta, phi] Euler angle container
float ypr[3]; //[yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//vars PID control balance
unsigned long timeNow, timePrev;
double elapsedTime;

xydetaset PID, error, previous_error;
xydetaset pid_p, pid_i, pid_d;
xydetaset desired_angle; //the angle to stay steady
double desired_yaw, zControl;

//PID constants
double kp = 0;
double ki = 0;
double kd = 0;
unsigned int throttle = 0b000001; //initial value of throttle to the motors
int motorOutput[4] = {0};
int motorSpeedData = 0;
#define throttle_max 0b111111
#define throttle_min 0b000001

//WLAN controller
#define BUFFER_SIZE 16384
const char *ssid = "ESPAP";
const char *password = "12345678";
char startMotor, prevStartMotor;
uint8_t buf[BUFFER_SIZE];
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

//interrupt detection routine
volatile bool mpuInterrupt = false; //indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//WLAN communication
//Just a little test message.  Go to http://192.168.4.1 in a web browser connected to this access point to see it.

void handleRoot() {
  //Serial.println("Access");
  server.send(200, "text/html", (char *)buf);
}

boolean readHTML() {
  File html = SPIFFS.open("/index.html", "r");
  if (!html) {
    //Serial.println("Failed to open html");
    return false;
  }
  size_t size = html.size();
  if (size >= BUFFER_SIZE) {
    //Serial.print("File Size Error:");
    //Serial.println((int)size);
  } else {
    //Serial.print("File Size OK:");
    //Serial.println((int)size);
  }
  html.read(buf, size);
  html.close();
  return true;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
  switch (type) {
    case WStype_DISCONNECTED:
      //Serial.printf("[%u] Disconnected!\n", num);
      startMotor = 0;
      break;
    case WStype_CONNECTED: {
        IPAddress ip = webSocket.remoteIP(num);
        //Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);

        // send message to client
        char message[100];
        sprintf(message, "{\"num\":\"%u\"}", num);
        webSocket.sendTXT(num, message);
        // send data to all connected clients
        //webSocket.broadcastTXT("message here");
      }
      break;
    case WStype_TEXT:
      if (num == 0) {
        //Serial.printf("[%u] %s\n", num, payload);
        //receiave json data
        char json[70];
        sprintf(json, "%s", payload);
        //receiave json data end

        //json parse
        StaticJsonBuffer<200> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(json);
        if (!root.success()) {
          //Serial.println("parse failed");
          return;
        }
        //json parse end
        if (root.containsKey("data")) {
          desired_angle.x = root["data"]["roll"];
          desired_angle.y = root["data"]["pitch"];
          desired_yaw = root["data"]["yaw"];
          zControl = root["data"]["z"];

          zControl /= 5;
          if (zControl < 1) {
            zControl = 1;
          } else if (zControl > throttle_max) {
            zControl = throttle_max;
          } else if (zControl < throttle_min) {
            zControl = throttle_min;
          }
          throttle = (int)zControl;

          //desired value input debug
          /*
            Serial.print("Control:\t");
            Serial.print(desired_angle.x);
            Serial.print("\t");
            Serial.print(desired_angle.y);
            Serial.print("\t");
            Serial.print(desired_yaw);
            Serial.print("\t");
            Serial.println(throttle);
          */
        } else if (root.containsKey("const")) {
          kp = root["const"]["kp"];
          ki = root["const"]["ki"];
          kd = root["const"]["kd"];

          //PID input debug
          /*
            Serial.print("PID:\t");
            Serial.print(kp);
            Serial.print("\t");
            Serial.print(ki);
            Serial.print("\t");
            Serial.println(kd);
          */
        } else if (root.containsKey("start")) {
          startMotor = (int)root["start"];

          //Motor start/stop input debug
          /*
            Serial.print("Motor start/stop:\t");
            Serial.println(startMotor);
          */
        }
      }
      break;
    default:
      break;
  }
}

//setup
void setup() {
  Wire.begin(2, 14);
  Serial.begin(115200); //initialize serial communication
  pinMode(MPU6050_INTERRUPT_PIN, INPUT);
  while (!Serial);

  //server setting
  //read html
  SPIFFS.begin();
  delay(10);
  if (!readHTML()) {
    //Serial.println("Read HTML error!!");
  }
  delay(300);

  //You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  //Serial.println("AP initialize");
  IPAddress myIP = WiFi.softAPIP();
  //Serial.println(myIP);

  //start webSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  server.on("/", handleRoot);
  server.begin();
  //server setting end

  //sensor settings
  mpu.initialize(); //initialize device
  mpu.testConnection(); //verify connection
  devStatus = mpu.dmpInitialize(); //load and configure the DMP

  //supply your own gyro offsets here, scaled for min sensitivity
  //1
  /*
    mpu.setXAccelOffset(-4721);
    mpu.setYAccelOffset(-4781);
    mpu.setZAccelOffset(1853);
    mpu.setXGyroOffset(-32);
    mpu.setYGyroOffset(36);
    mpu.setZGyroOffset(100);
  */

  //2
  mpu.setXAccelOffset(-1834);
  mpu.setYAccelOffset(-907);
  mpu.setZAccelOffset(677);
  mpu.setXGyroOffset(184);
  mpu.setYGyroOffset(-1);
  mpu.setZGyroOffset(-14);

  //make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true); //turn on the DMP, now that it's ready
    attachInterrupt(MPU6050_INTERRUPT_PIN, dmpDataReady, RISING); //enable Arduino interrupt detection
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true; //set our DMP Ready flag so the main loop() function knows it's okay to use it
    packetSize = mpu.dmpGetFIFOPacketSize(); //get expected DMP packet size for later comparison
  } else {
    //ERROR!
    //1 = initial memory load failed
    //2 = DMP configuration updates failed
    //(if it's going to break, usually the code will be 1)
  }
  //sensor settings end

  timeNow = micros(); //Start counting time
  delay(500);
}

void loop() {
  int motorNum;

  //server loop
  webSocket.loop();
  server.handleClient();
  //server loop end

  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize); //wait for MPU interrupt or extra packet(s) available

  //reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount(); //get current FIFO count

  //check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO(); //reset so we can continue cleanly
  } else if (mpuIntStatus & 0x02) {
    timePrev = timeNow; //the previous time is stored before the actual time read
    timeNow = micros(); //actual time read
    elapsedTime = (timeNow - timePrev) / 1000000;

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); //wait for correct available data length, should be a VERY short wait
    mpu.getFIFOBytes(fifoBuffer, packetSize); //read a packet from FIFO
    fifoCount -= packetSize; //track FIFO count here in case there is > 1 packet available

    //display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    /*
      //debug sensor
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
    */

    //PID control x
    error.x = (ypr[1] * 180 / M_PI) - desired_angle.x;
    pid_p.x = kp * error.x;
    if (-3 < error.x < 3) {
      pid_i.x += (ki * error.x);
    } else {
      pid_i.x = 0;
    }
    pid_d.x = kd * ((error.x - previous_error.x) / elapsedTime);
    PID.x = pid_p.x + pid_i.x + pid_d.x; //The final PID.x values is the sum of each of this 3 parts

    //PID control y
    error.y = (ypr[2] * 180 / M_PI) - desired_angle.y;
    pid_p.y = kp * error.y;
    if (-3 < error.y < 3) {
      pid_i.y += (ki * error.y);
    } else {
      pid_i.y = 0;
    }
    pid_d.y = kd * ((error.y - previous_error.y) / elapsedTime);
    PID.y = pid_p.y + pid_i.y + pid_d.y; //The final PID.y values is the sum of each of this 3 parts

    motorOutput[0] = PID.x - PID.y;
    motorOutput[1] = -PID.x - PID.y;
    motorOutput[2] = -PID.x + PID.y;
    motorOutput[3] = PID.x + PID.y;

    //PID output debug
    /*
        Serial.print("PID\t");
        Serial.print(motorOutput[0]);
        Serial.print("\t");
        Serial.print(motorOutput[1]);
        Serial.print("\t");
        Serial.print(motorOutput[2]);
        Serial.print("\t");
        Serial.println(motorOutput[3]);
    */

    if (startMotor && prevStartMotor) {
      for (motorNum = 0; motorNum <= 0b11; motorNum++) {
        eusartTransmit.split.address = motorNum;
        motorSpeedData = throttle + motorOutput[motorNum];
        if (motorSpeedData < 0b000001) {
          motorSpeedData = 0b000001;
        } else if (motorSpeedData > 0b111111) {
          motorSpeedData = 0b111111;
        }
        eusartTransmit.split.data = motorSpeedData;
        Serial.write(eusartTransmit.raw);
      }
    } else if (startMotor && !prevStartMotor) {
      for (motorNum = 0; motorNum <= 0b11; motorNum++) {
        eusartTransmit.split.address = motorNum;
        eusartTransmit.split.data = 0b000001;
        delay(1500);
        Serial.write(eusartTransmit.raw);
      }
    } else if (!startMotor) {
      for (motorNum = 0; motorNum <= 0b11; motorNum++) {
        eusartTransmit.split.address = motorNum;
        eusartTransmit.split.data = 0b000000;
        Serial.write(eusartTransmit.raw);
      }
    }

    prevStartMotor = startMotor;
    previous_error = error; //Remember to store the previous error
  }
}
