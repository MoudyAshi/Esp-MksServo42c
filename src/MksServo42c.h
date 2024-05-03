// MksServo42c.h
#ifndef MKSSERVO42C_H
#define MKSSERVO42C_H

#include <stdint.h>
#include <Arduino.h>
#include <string.h>
#include "SoftwareSerial.h"

#define maxBaudRate 57600
enum command {
  readAngle, readEncoder, readPulses,  
  readErrorAngle, readShaftStatus, readEnableStatus,

  sendCalibrate, sendMotorType, sendControlMode, sendOperatingCurrent,
  sendStepSize, sendEnableType,sendMotorDirection, sendAutoScreenOff,
  sendStallProtection, sendInternalSubdivision, sendUartBaud, sendUartAddress,
  sendZeroMode, moveZeroPoint, sendZeroPoint, sendZeroSpeed, sendZeroDirection, 
  setEnable, runConstSpeed, stopMotor, saveOrClearStatus,

  setKp, setKi, setKd,
  setAcc, setMaxTo, varControl
};

class MKSServoDriver
{
  public:
  MKSServoDriver();
  
  uint8_t length;
  String feedbackMessage = "";
  void init(uint8_t RX, uint8_t TX, long const baudRate = maxBaudRate, uint8_t servoAddress=0);
  void error();
  bool waitingForACK(uint8_t len);
  void ack(uint8_t len , command order);
  uint8_t getCheckSum(uint8_t *buffer,uint8_t len);

  String feedback();
  void outputRsignal();
  void outputTsignal();
  void setConstants();
  long rotationControl(float rotations, float speedPercentage, bool direction = 0);
  float sendServoSignal(command order);
  void sendServoSignal(command order, uint8_t mode);
  void setParameters(command order, uint16_t param);
  void constantControl(command order, float value=0, bool direction = 0);
  void variableControl(uint64_t order);
  int percentageToRange(float percentage, uint16_t lowerRange, uint16_t upperRange);

  private:
  EspSoftwareSerial::UART* servoSerial;
  uint8_t txBuffer[8];
  uint8_t rxBuffer[16];
  uint8_t rxCnt;
  uint8_t mode;
  uint32_t value;
  uint8_t MStep;
  uint32_t rotationPulses;
  float stepSize;
  float modValue;
};

#endif // MKSSERVO42C_H