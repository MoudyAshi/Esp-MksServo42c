#include "MksServo42c.h"

MKSServoDriver::MKSServoDriver() 
{
  servoSerial = nullptr;
}

void MKSServoDriver::init(uint8_t RX, uint8_t TX, long const baudRate, uint8_t servoAddress) 
{
    servoSerial = new EspSoftwareSerial::UART(RX, TX);
    servoSerial->begin(baudRate);
    rxCnt=0;
    MStep = 0x08;
    stepSize = 1.8;
    servoAddress = 0xE0 + servoAddress;
    txBuffer[0] = servoAddress;
}

float MKSServoDriver::sendServoSignal(command order)
{
  length = 3;
  if(order == readEncoder){
    txBuffer[1] = 0x30;
    length = 8;
  }
  else if(order == readPulses){
    txBuffer[1] = 0x33;
    length = 6;
  }
  else if(order == readAngle){
    txBuffer[1] = 0x36;
    length =  6;
  }
  else if(order == readErrorAngle){
    txBuffer[1] = 0x39;
    length = 4;
  }
  else if(order == readEnableStatus){
    txBuffer[1] = 0x3a;
  }
  else if(order == readShaftStatus){
    txBuffer[1] = 0x3e;
  }
  else {
    error();
    return 0;
  }
  txBuffer[2] = getCheckSum(txBuffer,2);
  servoSerial->write(txBuffer,3);
  ack(length,order);
  if(order!=readEnableStatus || order!=readShaftStatus)
    return modValue;
  return 0;
}
void MKSServoDriver::sendServoSignal(command order, uint8_t mode)
{
  length = 4;
  if(order == sendCalibrate){
    txBuffer[1] = 0x80;
  }
  else if(order == sendMotorType){
    txBuffer[1] = 0x81;
    stepSize = (mode*0.9) + 0.9;
    MKSServoDriver::setConstants();
  }
  else if(order == sendControlMode){
    txBuffer[1] = 0x82;
  }
  else if(order == sendOperatingCurrent){
    txBuffer[1] = 0x83;
  }
  else if(order == sendStepSize){
    txBuffer[1] = 0x84;
    MStep = mode;
    MKSServoDriver::setConstants();
  }
  else if(order == sendEnableType){
    txBuffer[1] = 0x85;
  }
  else if(order == sendMotorDirection){
    txBuffer[1] = 0x86;
  }
  else if(order == sendAutoScreenOff){
    txBuffer[1] = 0x87;
  }
  else if(order == sendStallProtection){
    txBuffer[1] = 0x88;
  }
  else if(order == sendInternalSubdivision){
    txBuffer[1] = 0x89;
  }
  else if(order == sendUartBaud){
    txBuffer[1] = 0x8a;
  }
  else if(order == sendUartAddress){
    txBuffer[1] = 0x8b;
  }
  else if(order == sendZeroMode){
    txBuffer[1] = 0x90;
  }
  else if(order == sendZeroPoint){
    txBuffer[1] = 0x91;
  }
  else if(order == sendZeroSpeed){
    txBuffer[1] = 0x92;
  }
  else if(order == sendZeroDirection){
    txBuffer[1] = 0x93;
  }
  else if(order == moveZeroPoint){
    txBuffer[1] = 0x94;
  }
  else {
    error();
    return ;
  }
  txBuffer[2] = mode;
  txBuffer[3] = getCheckSum(txBuffer,3);
  if(order!=sendCalibrate)
    servoSerial->write(txBuffer,4);
  else
  {
    servoSerial->write(txBuffer,4);
    delay(50000);
  }
  length = 3;
  ack(length,order);
}

//Default values: Kp:0x650, Ki:0x1, Kd: 0x650, Acc: 0x11e, MaxT: 0x4B0 (max)
void MKSServoDriver::setParameters(command order, uint16_t param)
{
  if(order == setKp){
    txBuffer[1] = 0xA1;
  }
  else if(order == setKi){
    txBuffer[1] = 0xA2;
  }
  else if(order == setKd){
    txBuffer[1] = 0xA3;
  }
  else if(order == setAcc){
    txBuffer[1] = 0xA4;
  }
  else if(order == setMaxTo){
    txBuffer[1] = 0xA5;
    param = static_cast<uint16_t>(percentageToRange(param,0x0000,0x04B0));
  }
  else {
    error();
    return ;
  }
  txBuffer[2] = (param >> 8) & 0xFF;
  txBuffer[3] = param & 0xFF;
  txBuffer[4] = getCheckSum(txBuffer,4);
  servoSerial->write(txBuffer,5);
  ack(length,order);
}

void MKSServoDriver::constantControl(command order, float value, bool direction)
{
  length=4;
  if(order == setEnable){
    txBuffer[1] = 0xF3;
  }
  else if(order == runConstSpeed){
    txBuffer[1] = 0xF6;
    value = static_cast<uint8_t>((direction << 7) | percentageToRange(value, 0x00, 0x7F));
  }
  else if(order == stopMotor){
    txBuffer[1] = 0xF7;
    txBuffer[2] = getCheckSum(txBuffer,2);
    servoSerial->write(txBuffer,3);
    length = 3;
    ack(length,order);
    return ;
  }
  else if(order == saveOrClearStatus){
    txBuffer[1] = 0xFF;
  }
  else {
    error();
    return ;
  }
  txBuffer[2] = value;
  txBuffer[3] = getCheckSum(txBuffer,3);
  servoSerial->write(txBuffer,4);
  length=3;
  ack(length,order);
}

void MKSServoDriver::variableControl(uint64_t mode)
{
  txBuffer[1] = 0xFD;
  txBuffer[2] = (mode >> 32) & 0xFF;
  txBuffer[3] = (mode >> 24) & 0xFF;
  txBuffer[4] = (mode >> 16) & 0xFF;
  txBuffer[5] = (mode >> 8) & 0xFF;
  txBuffer[6] = mode & 0xFF;
  txBuffer[7] = getCheckSum(txBuffer,7);
  length = 3;
  servoSerial->write(txBuffer,8);
  command order = varControl;
  ack(length,order);
}

uint8_t MKSServoDriver::getCheckSum(uint8_t *buffer,uint8_t size)
{
  uint8_t i;
  uint16_t sum=0;
  for(i=0;i<size;i++)
    {
      sum += buffer[i];
    }
  return(sum&0xFF);
}

void MKSServoDriver::ack(uint8_t len , command order)
{
  bool ackStatus;
  ackStatus = waitingForACK(length);
  length = 3;
  if(ackStatus == true)
  {
    switch (order){
      case sendCalibrate:
      case sendMotorType:
      case sendControlMode:
      case sendOperatingCurrent:
      case sendStepSize:
      case sendEnableType:
      case sendMotorDirection:
      case sendAutoScreenOff:
      case sendStallProtection:
      case sendInternalSubdivision:
      case sendUartBaud:
      case sendUartAddress:
      case sendZeroMode:
      case moveZeroPoint:
      case sendZeroPoint:
      case sendZeroSpeed:
      case sendZeroDirection:
      case setKp:
      case setKi:
      case setKd:
      case setAcc:
      case setMaxTo:
      case setEnable:
      case runConstSpeed:
      case stopMotor:
      case saveOrClearStatus:
      case varControl:
        if(rxBuffer[1]==0x00)
          feedbackMessage = "Instruction unsuccessful";
        else
          feedbackMessage = "Instruction successful";
        break;
      
      case readEnableStatus:
        if(rxBuffer[1]==0x01)
          feedbackMessage = "Enable pin enabled";
        else if(rxBuffer[1]==0x02)
          feedbackMessage = "Enable pin disabled";
        else
          feedbackMessage = "Enable pin error";
        break;
      
      case readShaftStatus:
        if(rxBuffer[1]==0x01)
          feedbackMessage = "Shaft blocked";
        else if(rxBuffer[1]==0x02)
          feedbackMessage = "Shaft unblocked";
        else
          feedbackMessage = "Shaft error";
        break;
      
      case readAngle:
        value = ((rxBuffer[3] << 8) | rxBuffer[4]) & 0xFFFF;
        modValue = (static_cast<float>(value) / 0xFFFF)*360.0f;
        break;
        
      case readErrorAngle:
        value = ((rxBuffer[3] << 8) | rxBuffer[4]) & 0xFFFF;
        modValue = (static_cast<float>(value) / 0xFFFF)*360.0f;
        break;
      
      case readPulses:
        modValue = (rxBuffer[1] * pow(16,3)) + (rxBuffer[2] * pow(16,2)) + (rxBuffer[3] * pow(16,1)) + rxBuffer[4]* pow(16,0);
        break;

      case readEncoder:
        modValue = (rxBuffer[5] * pow(16,1)) + (rxBuffer[6] * pow(16,0));
        break;

      default:
        feedbackMessage = "Invalid case";
        break;
    }
  }
  else
  {
    feedbackMessage = "No acknowldgement";
  }
}

bool MKSServoDriver::waitingForACK(uint8_t len)
{
  bool retVal;
  unsigned long sTime;
  unsigned long time;
  uint8_t rxByte;

  sTime = millis();
  rxCnt = 0;
  while(1)
  {
    if (servoSerial->available() > 0)
    {
      rxByte = servoSerial->read();
      if(rxCnt != 0)
      {
        rxBuffer[rxCnt++] = rxByte;
      }
      else if(rxByte == 0xE0)
      {
        rxBuffer[rxCnt++] = rxByte;
      }
    }

    if(rxCnt == len)
    {
      if(rxBuffer[len-1] == getCheckSum(rxBuffer,len-1))
      {
        retVal = true;
        break;
      }
      else
      {
        rxCnt = 0;
      }
    }
    time = millis();
    if((time - sTime) > 3000)
    {
      feedbackMessage = "TimeOut";
      retVal = false;
      break;
    }
  }
  return(retVal);
}

void MKSServoDriver::error()
{
  feedbackMessage = "Error 404";
}

void MKSServoDriver::outputTsignal()
{
    Serial.print("Transmitted Signal: ");
    for (int i = 0; i < length; i++)
     { Serial.print(txBuffer[i], HEX);
       Serial.print(" ");
       }
    Serial.println("");
}

void MKSServoDriver::outputRsignal()
{
    Serial.print("Received Signal: ");
   for (int i = 0; i < length; i++)
    { Serial.print(rxBuffer[i], HEX);
      Serial.print(" ");
      }
    Serial.println("");
}

void MKSServoDriver::setConstants()
{
  rotationPulses = MStep*(360.0/stepSize);
}

long MKSServoDriver::rotationControl(float rotations, float speedPercentage, bool direction)
{
  uint8_t speed=0x00;
  uint32_t pulses;
  uint32_t time;
  uint64_t param; 
  long velocityRpm;
  pulses = static_cast<uint32_t>(rotations*rotationPulses);
  speed = static_cast<uint8_t>((direction << 7) | percentageToRange(speedPercentage, 0x00, 0x7F));
  param = static_cast<uint64_t>(speed) << 32 | pulses;
  variableControl(param);
  velocityRpm = (percentageToRange(speedPercentage, 0x00, 0x7F)*30000L/(MStep*360/stepSize));
  time = (rotations*1000/(velocityRpm/60));
  time = (((time/1000)+1) * 1000);
  return time;
}

int MKSServoDriver::percentageToRange(float percentage, uint16_t lowerRange, uint16_t upperRange)
{
  uint16_t hexValue = 0x00;
  percentage = constrain(percentage, 0, 100);
  hexValue = map(percentage * 100, 0, 10000, lowerRange, upperRange);
  return hexValue;
}

String MKSServoDriver::feedback()
{
  return feedbackMessage;
}
