#include "AS5600.h"
//#include "Wire.h"
#include <i2c_driver_wire.h>
#include "RobotAxis.h"

elapsedMillis lastTick;
elapsedMillis timer;
RobotAxis axis1;
RobotAxis axis2;
RobotAxis axis3;
//RobotAxis axis1(0,2048,-60.0,60.0);
//RobotAxis axis2(1,2448,-60.0,60.0);

void setup(){
    Serial.begin(115200);
    Serial2.begin(115200);
    axis1 = RobotAxis(0,2048,-90.0,90.0);
    axis2 = RobotAxis(1,2448,-75.0,60.0);
    axis3 = RobotAxis(2,1000,-50,115);
    axis1.moveToTarget(2,0.0);
    axis2.moveToTarget(2,0.0);
    axis3.moveToTarget(4,0.0);
}

void loop(){
        axis1.tick();
        axis2.tick();
        axis3.tick();
    /*if(lastTick>10){
        axis3.updatePosition();
        Serial.println(axis3.getEncoderSteps());
        lastTick = 0;
    }*/

    if( (timer>10000) && axis1.isAtTarget() && axis2.isAtTarget() && axis3.isAtTarget()){
      delay(20);
      uint8_t newSpeed = random(3,7);
      axis1.moveToTarget(newSpeed,random(-20,20));
      axis2.moveToTarget(newSpeed,random(-60,-30));
      axis3.moveToTarget(newSpeed+3, random(45,110));
        timer = 0;
    }
}
