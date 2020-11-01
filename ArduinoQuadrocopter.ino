/*
 * Created:  17.08.2019 12:42:39
 * Authors: Lasse Kelling, Till Koch, Linus Lingstaedt
 */
#include "SimpleTimer.h"
#include <Wire.h>
#include <Arduino.h>
#include <SoftwareSerialParity.h>

#define RAWTORAD DEG_TO_RAD / 32.8

#define READINTERVAL 1 //ms
#define STARTBYTE 0x0f
#define ENDBYTE 0x00

#define MOTORFRONTLEFTPIN 3   //490Hz
#define MOTORFRONTRIGHTPIN 9  //490Hz
#define MOTORBACKLEFTPIN 10   //490Hz
#define MOTORBACKRIGHTPIN 11 //490Hz

#define MOTORPOWERCONSTANT 1 / 1.6

#define CHANNELENABLEMOTOR 5
#define CHANNELZEROPOSITION 8
#define CHANNELPOWER 3
#define CHANNELYAW 4
#define CHANNELPITCH 2
#define CHANNELROLL 1

SimpleTimer timer;
SoftwareSerialParity sbusSerial(2,4); //RX, TX

int channels[18];
double channel[19];
long time;
boolean failsafe = true;
boolean motorArmed = false;
int motorIdlePower = 135;

double motorPowerFrontLeft = 0;  //basepower + yaw - pitch + roll
double motorPowerFrontRight = 0; //basepower - yaw - pitch - roll
double motorPowerBackLeft = 0;   //basepower - yaw + pitch + roll
double motorPowerBackRight = 0;  //basepower + yaw + pitch - roll

int16_t gyroRawX, gyroRawY, gyroRawZ;
int gyroErrorX, gyroErrorY, gyroErrorZ;
double gyroX, gyroY, gyroZ;
double velocityYaw, velocityPitch, velocityRoll;

double yawVelocity = 0, idealYawVelocity = 0, controllerYawError = 0, controllerYawIValue = 0, controllerYawSignal = 0;
double pitchVelocity = 0, idealPitchVelocity = 0, controllerPitchError = 0, controllerPitchIValue = 0, controllerPitchSignal = 0;
double rollVelocity = 0, idealRollVelocity = 0, controllerRollError = 0, controllerRollIValue = 0, controllerRollSignal = 0;

double controllerYawKP = 300, controllerPitchKP = 150, controllerRollKP = 150;
double controllerYawKI = 0, controllerPitchKI = 0, controllerRollKI = 0;



void setup() {
  analogWrite(MOTORFRONTLEFTPIN, 128);
  analogWrite(MOTORFRONTRIGHTPIN, 128);
  analogWrite(MOTORBACKLEFTPIN, 128);
  analogWrite(MOTORBACKRIGHTPIN, 128);

  delay(2000);

  Serial.begin(115200);
  Serial.println("Akku anschliessen");

  sbusSerial.begin(100000, EVEN);

  Serial.println("SBUS Protocol getartet");
  Serial.println(failsafe);

  Wire.begin();
  // Deaktivieren des Stand-by-Modus
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // Konfiguration des Gyroskop Messbereichs
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission();
  calibrateGyro(200);
  Serial.println("MPU Offset gemessen.");

  timer.setTimer(READINTERVAL, processData, 1);
}
void loop() {
  motorArmed = channel[CHANNELENABLEMOTOR] == 1;
  time = millis();
  readRawGyroData();
  computeCorrectVelocity();

  float remotePower = channel[CHANNELPOWER];
  float remoteYaw = channel[CHANNELYAW];
  float remotePitch = channel[CHANNELPITCH];
  float remoteRoll = channel[CHANNELROLL];

  idealYawVelocity =  -(remoteYaw - 0.5)  * 20;
  idealPitchVelocity =  (remotePitch -0.5)  * 20;
  idealRollVelocity = -(remoteRoll - 0.5)* 20;

  controllerYawError  = idealYawVelocity  - velocityYaw;
  controllerPitchError  = idealPitchVelocity  + velocityPitch;
  controllerRollError = idealRollVelocity + velocityRoll;

  controllerYawIValue += controllerYawError * controllerYawKI;
  controllerPitchIValue += controllerPitchError * controllerPitchKI;
  controllerRollIValue += controllerRollError * controllerRollKI;

  controllerYawSignal = controllerYawError * controllerYawKP + controllerYawIValue;
  controllerPitchSignal = controllerPitchError * controllerPitchKP + controllerPitchIValue;
  controllerRollSignal = controllerRollError * controllerRollKP + controllerRollIValue;

  int basepower = remotePower * 1000;

  computeMotorPower(basepower, controllerYawSignal, controllerPitchSignal, controllerRollSignal);
  setMotorPower();
}
void disableMotors() {
  analogWrite(MOTORFRONTLEFTPIN, 128);
  analogWrite(MOTORFRONTRIGHTPIN, 128);
  analogWrite(MOTORBACKLEFTPIN, 128);
  analogWrite(MOTORBACKRIGHTPIN, 128);
}
void computeMotorPower(double base, double yaw, double pitch, double roll) {
  if (base < 0) {
    disableMotors();
    return;
  }
  motorPowerFrontLeft = base - yaw - pitch - roll;
  motorPowerFrontRight =  base + yaw - pitch + roll;
  motorPowerBackLeft =    base + yaw + pitch - roll;
  motorPowerBackRight = base - yaw + pitch + roll;
  if (motorPowerFrontLeft < 0) motorPowerBackRight -= motorPowerFrontLeft;  // fl => br
  if (motorPowerFrontRight < 0) motorPowerBackLeft -= motorPowerFrontRight; // fr => bl
  if (motorPowerBackLeft < 0) motorPowerFrontRight -= motorPowerBackLeft; // bl => fr
  if (motorPowerBackRight < 0) motorPowerFrontLeft -= motorPowerBackRight;  // br => fl*
  motorPowerFrontLeft = pow(max(0, motorPowerFrontLeft), MOTORPOWERCONSTANT);
  motorPowerFrontRight = pow(max(0, motorPowerFrontRight), MOTORPOWERCONSTANT);
  motorPowerBackLeft = pow(max(0, motorPowerBackLeft), MOTORPOWERCONSTANT);
  motorPowerBackRight = pow(max(0, motorPowerBackRight), MOTORPOWERCONSTANT);
}
void setMotorPower() {
  if (!motorArmed) {
    disableMotors();
    return;
  }
  analogWrite(MOTORFRONTLEFTPIN, min(250, (max(0, motorPowerFrontLeft) + motorIdlePower)));
  analogWrite(MOTORFRONTRIGHTPIN, min(250, (max(0, motorPowerFrontRight) + motorIdlePower)));
  analogWrite(MOTORBACKLEFTPIN, min(250, (max(0, motorPowerBackLeft) + motorIdlePower)));
  analogWrite(MOTORBACKRIGHTPIN, min(250, (max(0, motorPowerBackRight) + motorIdlePower)));
}
void calibrateGyro(int cycleCount) {
  for (int i = 0; i < cycleCount; i++) {
    readRawGyroData();
    gyroErrorX += gyroRawX;
    gyroErrorY += gyroRawY;
    gyroErrorZ += gyroRawZ;
  }
  // Berechnen des Durchschnitts zum Bestimmen des durchschnittlichen Fehlers
  gyroErrorX /= cycleCount;
  gyroErrorY /= cycleCount;
  gyroErrorZ /= cycleCount;
}
void processData() {
  timer.setTimer(READINTERVAL, processData, 1);
  static int bufferIndex = 0;
  static byte buffer[25];
  while (sbusSerial.available()) {
    byte lastReadByte;
    lastReadByte = sbusSerial.read();
    if (lastReadByte != STARTBYTE && bufferIndex == 0)
      continue;
    buffer[bufferIndex] = lastReadByte;
    bufferIndex++;
    if (bufferIndex == 25) {
      bufferIndex = 0;
      if (buffer[24] != ENDBYTE)
        continue;
      channels[0] = ((buffer[1] | buffer[2] << 8) & 0x07FF);
      channels[1] = ((buffer[2] >> 3 | buffer[3] << 5) & 0x07FF);
      channels[2] = ((buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10) & 0x07FF);
      channels[3] = ((buffer[5] >> 1 | buffer[6] << 7) & 0x07FF);
      channels[4] = ((buffer[6] >> 4 | buffer[7] << 4) & 0x07FF);
      channels[5] = ((buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9) & 0x07FF);
      channels[6] = ((buffer[9] >> 2 | buffer[10] << 6) & 0x07FF);
      channels[7] = ((buffer[10] >> 5 | buffer[11] << 3) & 0x07FF);
      channels[8] = ((buffer[12] | buffer[13] << 8) & 0x07FF);
      channels[9] = ((buffer[13] >> 3 | buffer[14] << 5) & 0x07FF);
      channels[10] = ((buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
      channels[11] = ((buffer[16] >> 1 | buffer[17] << 7) & 0x07FF);
      channels[12] = ((buffer[17] >> 4 | buffer[18] << 4) & 0x07FF);
      channels[13] = ((buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9) & 0x07FF);
      channels[14] = ((buffer[20] >> 2 | buffer[21] << 6) & 0x07FF);
      channels[15] = ((buffer[21] >> 5 | buffer[22] << 3) & 0x07FF);
      if (buffer[23] & 0x0001)
        channels[16] = 2047;
      else
        channels[16] = 0;
      if ((buffer[23] >> 1) & 0x0001)
        channels[17] = 2047;
      else
        channels[17] = 0;
      if ((buffer[23] >> 3) & 0x0001)
        failsafe = true;
      else
        failsafe = false;
      calculateNormalizedChannels();
    }
  }
}
void calculateNormalizedChannels() {
  for (int i = 1; i < 19; i++) {
    channel[i] = (channels[i - 1] - 172.0) / (1811.0 - 172.0);
  }
}
void readRawGyroData() {
  // Gyroskop Werte auslesen und Fehler abziehen
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6, true);

  gyroRawX = (Wire.read() << 8 | Wire.read());
  gyroRawY = (Wire.read() << 8 | Wire.read());
  gyroRawZ = (Wire.read() << 8 | Wire.read());
}
void computeCorrectVelocity() {
  velocityYaw = (gyroRawZ - gyroErrorZ) * RAWTORAD;
  velocityPitch = (gyroRawX - gyroErrorX) * RAWTORAD;
  velocityRoll = (gyroRawY - gyroErrorY) * RAWTORAD;
}
