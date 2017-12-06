/*
   Drone central controler

   Created on August 28, 2017

   December 4: MPU6050 DMP

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

#define INTERRUPT_PIN 15

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
bool dmpReady = false;  //set true if DMP init was successful
uint8_t mpuIntStatus;   //holds actual interrupt status byte from MPU
uint8_t devStatus;      //return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    //expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     //count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; //FIFO storage buffer

//vars orientation/motion
Quaternion q;           //[w, x, y, z]         quaternion container
VectorInt16 aa;         //[x, y, z]            accel sensor measurements
VectorInt16 aaReal;     //[x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    //[x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    //[x, y, z]            gravity vector
float euler[3];         //[psi, theta, phi]    Euler angle container
float ypr[3];           //[yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//vars PID control balance
double elapsedTime, timeNow, timePrev;

xydetaset PID, error, previous_error;
xydetaset pid_p, pid_i, pid_d;
xydetaset desired_angle; //the angle to stay steady

//PID constants
double kp = 3;
double ki = 0;
double kd = 0;
char throttle = 0b100000; //initial value of throttle to the motors

//WLAN controller
#define BUFFER_SIZE 16384
//#define BUFFER_SIZE 30000
const char *ssid = "ESPAP";
const char *password = "12345678";
bool startMotor;
uint8_t buf[BUFFER_SIZE];
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

//interrupt detection routine
volatile bool mpuInterrupt = false;     //indicates whether MPU interrupt pin has gone high
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
      Serial.printf("[%u] Disconnected!\n", num);
      startMotor = 0;
      break;
    case WStype_CONNECTED: {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);

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
          Serial.println("parse failed");
          return;
        }
        //json parse end
        if (root.containsKey("data")) {
          desired_angle.x = root["data"]["roll"];
          desired_angle.y = root["data"]["pitch"];
          /*
            yawTarget = root["data"]["yaw"];
            zControl = root["data"]["z"];
            zControl = 1 + zControl / 1000;
          */

          // Print values.
          //Serial.printf("%d %d %d %d\n", rollTarget, pitchTarget, yawTarget, zControl);
        } else if (root.containsKey("const")) {
          kp = root["const"]["kp"];
          ki = root["const"]["ki"];
          kd = root["const"]["kd"];

          Serial.printf("kp:%f ki:%f kd:%f\n", kp, ki, kd);
        } else if (root.containsKey("start")) {
          startMotor = root["start"];
          //Serial.println(startMotor);
        }
      }
      break;
  }
}

//setup
void setup() {
  Wire.begin(2, 14);
  Serial.begin(115200); //initialize serial communication
  pinMode(INTERRUPT_PIN, INPUT);
  while (!Serial);

  //server setting
  //read html
  SPIFFS.begin();
  delay(10);
  if (!readHTML()) {
    Serial.println("Read HTML error!!");
  }
  delay(300);

  //You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  Serial.println("AP initialize");
  IPAddress myIP = WiFi.softAPIP();
  Serial.println(myIP);

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
  mpu.setXAccelOffset(-4577);
  mpu.setYAccelOffset(-165);
  mpu.setZAccelOffset(1772); //1688 factory default for my test chip
  mpu.setXGyroOffset(120);
  mpu.setYGyroOffset(-46);
  mpu.setZGyroOffset(22);

  //make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true); //turn on the DMP, now that it's ready
    attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING); //enable Arduino interrupt detection
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

  timeNow = millis(); //Start counting time in milliseconds]
  delay(500);
}

void loop() {
  //server loop
  webSocket.loop();
  server.handleClient();
  //server loop end

  timePrev = timeNow;  //the previous time is stored before the actual time read
  timeNow = millis();  //actual time read
  elapsedTime = (timeNow - timePrev) / 1000;

  //if programming failed, don't try to do anything
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
      pid_i.x = pid_i.x + (ki * error.x);
    }
    pid_d.x = kd * ((error.x - previous_error.x) / elapsedTime);
    PID.x = pid_p.x + pid_i.x + pid_d.x; //The final PID.x values is the sum of each of this 3 parts

    //PID control y
    error.y = (ypr[2] * 180 / M_PI) - desired_angle.y;
    pid_p.y = kp * error.y;
    if (-3 < error.y < 3) {
      pid_i.y = pid_i.y + (ki * error.y);
    }
    pid_d.y = kd * ((error.y - previous_error.y) / elapsedTime);
    PID.y = pid_p.y + pid_i.y + pid_d.y; //The final PID.y values is the sum of each of this 3 parts

    /*
        //PID output debug
        Serial.print("PID\t");
        Serial.print(-PID.x + PID.y); //0b00
        Serial.print("\t");
        Serial.println(-PID.x - PID.y); //0b01
        Serial.print("\t");
        Serial.print(PID.x - PID.y); //0b10
        Serial.print("\t");
        Serial.print(PID.x + PID.y); //0b11
    */
    /*
      eusartTransmit.split.address = 0b01;
      eusartTransmit.split.data = throttle + PID.x + PID.y;
      if (eusartTransmit.split.data <= 0) {
      eusartTransmit.split.data = 1;
      }
      if (eusartTransmit.split.data > 0b111111) {
      eusartTransmit.split.data = 0b111111;
      }
      Serial.write(eusartTransmit.raw);

      eusartTransmit.split.address = 0b11;
      eusartTransmit.split.data = throttle - PID.x - PID.y;
      if (eusartTransmit.split.data <= 0) {
      eusartTransmit.split.data = 1;
      }
      if (eusartTransmit.split.data > 0b111111) {
      eusartTransmit.split.data = 0b111111;
      }
      Serial.write(eusartTransmit.raw);
    */

    previous_error.x = error.x; //Remember to store the previous error.x
    previous_error.y = error.y; //Remember to store the previous error.y
  }
}
