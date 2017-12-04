/*
 * Drone central controler
 * 
 * Created on August 28, 2017
 * 
 * December 4: MPU6050 DMP
 * 
 * ESP8266
 * MPU6050
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 15

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//interrupt detection routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//union transmit USART data
union eusartTransmit {
  unsigned char raw;

  struct split {
    unsigned int address : 2;
    unsigned int data : 6;
  } split;
} eusartTransmit;

void setup() {
  Wire.begin(2, 14);
  Serial.begin(115200);// initialize serial communication
  pinMode(INTERRUPT_PIN, INPUT);
  while (!Serial);

  mpu.initialize();// initialize device
  mpu.testConnection();// verify connection
  devStatus = mpu.dmpInitialize();// load and configure the DMP

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-4577);
  mpu.setYAccelOffset(-165);
  mpu.setZAccelOffset(1772); // 1688 factory default for my test chip
  mpu.setXGyroOffset(120);
  mpu.setYGyroOffset(-46);
  mpu.setZGyroOffset(22);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);// turn on the DMP, now that it's ready
    attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);// enable Arduino interrupt detection
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;// set our DMP Ready flag so the main loop() function knows it's okay to use it
    packetSize = mpu.dmpGetFIFOPacketSize();// get expected DMP packet size for later comparison
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
  }
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {// wait for MPU interrupt or extra packet(s) available
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();// reset so we can continue cleanly
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();// wait for correct available data length, should be a VERY short wait
    mpu.getFIFOBytes(fifoBuffer, packetSize);// read a packet from FIFO
    fifoCount -= packetSize;// track FIFO count here in case there is > 1 packet available

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
  }
  /*
    eusartTransmit.split.address = 0b01;
    eusartTransmit.split.data = 0b100000;
    Serial.write(eusartTransmit.raw);
    delay(3000);
    eusartTransmit.split.data = 0b000000;
    Serial.write(eusartTransmit.raw);
    delay(500);

    eusartTransmit.split.address = 0b11;
    eusartTransmit.split.data = 0b100000;
    Serial.write(eusartTransmit.raw);
    delay(3000);
    eusartTransmit.split.data = 0b000000;
    Serial.write(eusartTransmit.raw);
    delay(500);
  */
}
