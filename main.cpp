#include "AS5600.h"
#include "RobotAxis.h"
#include "Robot.h"

elapsedMillis lastTick;
elapsedMillis timer;
Robot klarissa;
uint8_t numAxes = 3;

uint8_t axisAddress[]= {0,1,2,3,4};

float axisBottom[]= {-45.0, //Axis1
                      -10, //Axis2
                      -120.0, //Axis3
                        -2000.0, //Axis4
                        -2000.0}; //Axis5;

float axisTop[]= {45.0,    //Axis1
                   75.0,    //Axis2
                   10.0,   //Axis3
                   2000.0,     //Axis4
                   2000.0};    //Axis5;

float homeOffset[]= {1893,    //Axis1
                      1563,    //Axis2
                      3066,    //Axis3
                      0.0,     //Axis4
                      0.0};    //Axis5;

uint8_t defaultSpeed[] = {5,5,8,0,0};

void setup(){
    Serial.begin(115200);
    Serial2.begin(115200);
    
    for(int i=0;i<numAxes;i++){
      klarissa.configureAxis(axisAddress[i],homeOffset[i],axisBottom[i],axisTop[i]);
    }
    klarissa.homeAxes();
}

void loop(){
    if(lastTick>1){
      lastTick = 0;
      klarissa.tick();
    }
    Serial.println(timer);
    if( (timer>10000) && klarissa.isAtTarget()){
      float newPose[numAxes];
      uint8_t poseSpeed[numAxes];
      for(int i=0; i<numAxes;i++){
        newPose[i] = random(axisBottom[i]+15,axisTop[i]-15);
        poseSpeed[i] = random(1,defaultSpeed[i]*2);
      }
      klarissa.executePose(newPose, poseSpeed);
      timer = 0;
    }
}
