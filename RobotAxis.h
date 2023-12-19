#pragma once
#include "AS5600.h"       // Library for I2C control of AS5600 Magnetic Encoder
#include "Arduino.h"   
#include <Bounce2.h>      // Library for debouncing inputs 
#include "ArduPID.h"
//#include "Wire.h"
#include <i2c_driver_wire.h>

    class RobotAxis{
      private:
        uint8_t serialAddress;
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
        
        //NYI
        int homingPin;
        int enablePin;
        int maximumSpeed;
        ArduPID positionController;
        double Kp,Ki,Kd;
        double input,output,setpoint;
        bool enabled;
        bool calibrated;
        bool faulted;
        uint8_t faultCode;
        bool atTarget;





      public:
        RobotAxis();
        RobotAxis(
            uint8_t serialAddress,
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
        uint8_t getFault();
        void setHomeOffset(int encoderSteps);
        void disable();
        void enable();
        void setTargetPosition(float target);
        void setTargetPosition(float target,uint8_t speed);
        void rotate(uint16_t speed,bool forward);    //use an enumerated type for direction
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
        int16_t encoderOffset,
        float min,
        float max) {
          
        serialAddress = UARTaddress;
        homeOffset = encoderOffset;
        minPosition = min;
        maxPosition = max;
        maximumSpeed = 10;
        encoder = AS5600();
        encoder.begin(1);
        //encoder.setDirection(1);
        calibrated = false;
        moving = false;
        faulted = false;
        enabled = true;
        atTarget = false;
        updatePosition();
     //   faultCode = 0x07; //Not Calibrated


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

    uint8_t RobotAxis::getFault(){
      return faultCode;
    }

    void RobotAxis::setHomeOffset(int encoderSteps){
      homeOffset = encoderSteps;
    }

    void RobotAxis::disable(){
      enabled = false;
      return;
    }

    void RobotAxis::enable(){
      enabled = true;
    }

    void RobotAxis::updatePosition(){
        TCA9548A(serialAddress);
        delay(1);
        encoderSteps = encoder.rawAngle()-homeOffset;
    }

    void RobotAxis::setMotorZero(){
      byte zeroMessage[] = {0xe0, 0x91, 0x00, 0x00};
      zeroMessage[0]+=serialAddress;
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
      //Prepare Serial Rotate Command
      byte moveMessage[] = {0xe0,0xf6,0x00,0x00};
      moveMessage[0]+=serialAddress;
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
        atTarget = false;
        targetPosition = target;
        forward = targetPosition < encoderSteps*AS5600_RAW_TO_DEGREES;
        rotate(speed,forward);
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

        stopMessage[0] += serialAddress;
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
        if(!enabled||faulted){
            motorStop();
            return;
        }
        if(moving){
            float position = encoderSteps*AS5600_RAW_TO_DEGREES;

            if(position >= maxPosition || position <= minPosition){
            Serial.print("Motor #:");
            Serial.print(serialAddress+1);
            Serial.print(" Position ");
            Serial.print(position);
            Serial.print(" Enabled:");
            Serial.print(enabled);
            Serial.print(" Faulted:");
            Serial.println(faulted);
                motorStop();
                enabled = false;
                faulted = true;
                faultCode = 0x09; //Out of range
            }
            float difference = position - targetPosition;
            if(difference == 0 || ((difference>0) && !forward) || ((difference <0) && forward)){
                motorStop();
                atTarget = true;
            }
        }

    }



void RobotAxis::TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}


