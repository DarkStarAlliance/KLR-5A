#pragma once
#include "AS5600.h"       // Library for I2C control of AS5600 Magnetic Encoder
#include "Arduino.h"   
#include <Bounce2.h>      // Library for debouncing inputs 
#include "ArduPID.h"
//#include <i2c_driver_wire.h>
#include <Wire.h>

enum faultCodes  { NOFAULT,
                    NOCAL,
                    OOR_LOW,
                    OOR_HIGH,
                    OVRSPEED};

#define DEFAULT_MAX_SPEED 64



class RobotAxis{
  private:
    
    uint8_t motorAddress;
    uint8_t encoderAddress;
    uint8_t currentSpeed;
    bool    moving;
    bool    forward;
    AS5600 encoder;
    int16_t homeOffset;
    int16_t encoderSteps;
    float targetPosition;
    uint8_t targetSpeed;
    float minPosition;
    float maxPosition;
    bool faulted;
    bool atTarget;
    faultCodes faultCode;
    bool calibrated;
    
    //NYI
    int homingPin;
    int enablePin;
    int maximumSpeed;
    ArduPID positionController;
    double Kp,Ki,Kd;
    double input,output,setpoint;
    bool enabled;

  public:
    RobotAxis();
    RobotAxis(
        uint8_t UARTaddress,
        uint8_t encAddress,
        bool dirInverted,
        int16_t homeOffset,
        float minPosition,
        float maxPosition
        ); //constructor

    void calibrateSensors();
    int16_t getEncoderSteps();
    float getPositionMax();
    float getPositionMin();
    int16_t getHomeOffset();
    void setMotorZero();
    bool isCalibrated();
    bool isEnabled();
    bool isFaulted();
    bool isMoving();
    bool isForward();
    faultCodes getFault();
    void setHomeOffset(int encoderSteps);
    void disable();
    void enable();
    void setTargetPosition(float target);
    void setTargetPosition(float target,uint8_t speed);
    void rotate(uint16_t speed,bool forward);
    void moveToTarget(uint8_t speed, float target);
    void faultReset();
    void motorStop();
    void updatePosition();
    void tick();
    bool isAtTarget();
    void TCA9548A(uint8_t bus);
};//end of RobotAxis class

RobotAxis::RobotAxis(){
}

RobotAxis::RobotAxis(
    uint8_t UARTaddress,
    uint8_t encAddress,
    bool dirInverted,
    int16_t encoderOffset,
    float min,
    float max) {
      
    motorAddress = UARTaddress;      //Address used to communicate with the motor over UART
    encoderAddress = encAddress;      //Address used to communicate with the encoder via I2C Multiplexer
    homeOffset = encoderOffset;       //Encoder steps offset of home position (zero degrees)
    minPosition = min;                //Lower operating limit before fault (degrees)
    maxPosition = max;                //Upper operating limit before fault (degrees)
    maximumSpeed = DEFAULT_MAX_SPEED; //Maximum speed before fault (0-127)
    
    encoder = AS5600();               
    encoder.begin(!dirInverted);      //Begin encoder and define direction of rotation
    
    calibrated = false;
    faulted = true;
    faultCode = NOCAL;

    enabled = true;
    atTarget = false;
    moving = false;
    tick();
} //end of constructor


int16_t RobotAxis::getHomeOffset(){
  return homeOffset;
}

int16_t RobotAxis::getEncoderSteps(){
  // updatePosition();
  return encoderSteps;
}

float RobotAxis::getPositionMax(){
  return maxPosition;
}

float RobotAxis::getPositionMin(){
  return minPosition;
}

bool RobotAxis::isCalibrated(){
  return calibrated;
}

bool RobotAxis::isEnabled(){
  return enabled;
}

bool RobotAxis :: isFaulted(){
  return faulted;
}

bool RobotAxis::isMoving(){
  return moving;
}

bool RobotAxis::isAtTarget(){
    return atTarget;
}

bool RobotAxis::isForward(){
    return forward;
}

faultCodes RobotAxis::getFault(){
  return faultCode;
}

void RobotAxis::setHomeOffset(int encoderSteps){
  homeOffset = encoderSteps;
  return;
}

void RobotAxis::disable(){
  enabled = false;
  return;
}

void RobotAxis::enable(){
  enabled = true;
  return;
}

void RobotAxis::updatePosition(){
    TCA9548A(encoderAddress);
    delayMicroseconds(150);
    encoderSteps = encoder.rawAngle()-homeOffset;
    if(encoderSteps<-2048){
      encoderSteps+=4096;
    }else{
      if(encoderSteps>2048){
        encoderSteps-=4096;
      }
    }
}

void RobotAxis::setMotorZero(){
  byte zeroMessage[] = {0xe0, 0x91, 0x00, 0x00};
  zeroMessage[0]+=motorAddress;
  uint16_t checksum = 0;
  for(uint8_t i=0; i< sizeof(zeroMessage)-1; i++){
    checksum += zeroMessage[i];
  }
  zeroMessage[3] = checksum;
  //Send Zero Command
  Serial2.clear();
  Serial2.write(zeroMessage,sizeof(zeroMessage));

}


void RobotAxis::rotate(uint16_t speed,bool forwardDirection){
  //Set local movement flags
  moving = true;
  forward = forwardDirection;
  targetSpeed = speed;
  currentSpeed = targetSpeed;

  //Prepare Serial Rotate Command
  byte moveMessage[] = {0xe0,0xf6,0x00,0x00};
  moveMessage[0]+=motorAddress;
  moveMessage[2] = speed;
  if(!forwardDirection){
    moveMessage[2]+= 128;
  }
  uint16_t checksum = 0;
  for(uint8_t i=0; i< sizeof(moveMessage)-1; i++){
    checksum += moveMessage[i];
  }
  moveMessage[3] = checksum;
  //Send Rotate Command
  Serial2.clear();
  Serial2.write(moveMessage,sizeof(moveMessage));
  return;
}

void RobotAxis::moveToTarget(uint8_t speed, float target){
    updatePosition();
    targetPosition = target;
    if(encoderSteps*AS5600_RAW_TO_DEGREES!=targetPosition){
      atTarget = false;
      forward = targetPosition < encoderSteps*AS5600_RAW_TO_DEGREES;
      rotate(speed,forward);
    }else{
      atTarget = true;
    }
    return;
}

void RobotAxis::setTargetPosition(float target){
    targetPosition = target;
}

void RobotAxis::setTargetPosition(float target,uint8_t speed){
    targetPosition = target;
    targetSpeed = speed;
}

void RobotAxis::motorStop(){
    byte stopMessage[3] = {0xe0, 0xf7, 0xd7};

    stopMessage[0] += motorAddress;
    uint16_t checksum = 0;
    for(uint8_t i=0; i<sizeof(stopMessage)-1; i++){
        checksum += stopMessage[i];
    }
    stopMessage[2] = checksum;

    Serial2.clear();
    Serial2.write(stopMessage,sizeof(stopMessage));
    moving = false;
}

void RobotAxis::tick(){
    updatePosition();
   /* Serial.print("Axis ");
    Serial.print(motorAddress+1);
    Serial.print(": ");
    Serial.print(getEncoderSteps());
    Serial.print("\t");*/
    if((faulted && faultCode>1) || !enabled){
        motorStop();
        Serial.print("Axis");
        Serial.print(motorAddress+1);
        Serial.print(" faulted! FaultCode:");
        Serial.println(faultCode);
        return;
    }else{
      float position = encoderSteps*AS5600_RAW_TO_DEGREES;
      if(position >= maxPosition || position <= minPosition){
        motorStop();
        faulted = true;
        if(position>=maxPosition){
          faultCode = OOR_HIGH;
        }else{
          faultCode = OOR_LOW;
        }
        return;
      }else{
        float difference = position - targetPosition;
        if(difference == 0 || ((difference>0) && !forward) || ((difference <0) && forward)){ //Most likely this will overshoot target and come out false during a 100% speed move
            motorStop();
            atTarget= true;
            //Serial.println("AtTarget");
           /* if(difference==0){
              atTarget = true;
            }else{
              atTarget = false;
            }*/
        }else{
          //Acceleration/Deceleration Code here
        }
      }
    }
}

void RobotAxis::TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}


