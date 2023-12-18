#include "AS5600.h"
//#include "Wire.h"
//#include <i2c_driver_wire.h> //Teensy-specific Replacement for Wire
#include "RobotAxis.h"

elapsedMillis lastTick;
elapsedMillis timer;
RobotAxis axis1;
RobotAxis axis2;
RobotAxis axis3;

void setup(){
    Serial.begin(115200);
    Serial2.begin(115200);

    axis1 = RobotAxis(0,1968,-60,60.0);
    axis2 = RobotAxis(1,1560,-30.0,65.0);
    axis3 = RobotAxis(2,3117,-110,30);
    //axis1.updatePosition();
    //axis2.updatePosition();
    //axis3.updatePosition();
    axis1.moveToTarget(2,0.0);
    axis2.moveToTarget(2,0.0);
    axis3.moveToTarget(4,0.0);
}

void loop(){


    if(lastTick>1){

      lastTick = 0;
      axis1.tick();
      Serial.print(axis1.getEncoderSteps()*AS5600_RAW_TO_DEGREES);
      Serial.print("\t");
      axis2.tick();
      Serial.print(axis2.getEncoderSteps()*AS5600_RAW_TO_DEGREES);
      Serial.print("\t");
      //Serial.println(axis2.isForward());
      axis3.tick();
      Serial.println(axis3.getEncoderSteps()*AS5600_RAW_TO_DEGREES);

    }

    if( (timer>10000) && axis1.isAtTarget() && axis2.isAtTarget() && axis3.isAtTarget()){
      delay(20);
      uint8_t newSpeed = random(3,6);
      axis1.moveToTarget(newSpeed,random(-10,10));
      axis2.moveToTarget(newSpeed,random(10,60));
      axis3.moveToTarget(newSpeed+3, random(-75,-45));
        timer = 0;
    }
}
