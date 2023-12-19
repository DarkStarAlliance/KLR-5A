#include "Arduino.h"   
#include <Bounce2.h>      // Library for debouncing inputs 
#include "RobotAxis.h"
#include "Joystick.h"
#include "AS5600.h"

class Robot{
  private:
   /* enum controlMode {T1,T2,AUTO,AUTOEXT};
    controlMode& operator++(controlMode& orig) {  //pre increment operator
      orig = static_cast<controlMode>(orig + 1);
      return orig;
    }
    controlMode operator++(controlMode& orig, int) {  //post increment operator
    controlMode rVal = orig;
    ++orig;
    return rVal;}
*/
    RobotAxis axis[3];
    int currentPose[3];
    int targetPose[3];
    bool estop;
    bool mstop;
    bool faulted;
    int faultCode;
    bool moving;
    bool calibrated;
    bool enabled;
    //Joystick pendant;

  public:
    Robot();
    void enableMotors();
    void disableMotors();
    
    void homeAxes();
    void homeAxes(uint8_t speed);

    void getCurrentPose();
    void getTargetPose();
    void setTargetPose();

    bool isAtTarget();
    
    void executePose(float *pose, uint8_t *poseSpeed);
    void runProgram(int programNumber);

    void configureAxis(uint8_t axisIndex,int axisOffset, float axisMin, float axisMax);
    void tick();
    


};//end of Robot class



Robot::Robot() {

      calibrated = false;
      moving = false;
      faulted = true;
      enabled = false;
      faultCode = 7; //Not Calibrated
} //end of constructor

void Robot::executePose(float *pose, uint8_t *poseSpeed){
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
  Serial.println(pose[i]);
  axis[i].moveToTarget(poseSpeed[i],pose[i]);
  } 
}

void Robot::homeAxes(){
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
    axis[i].moveToTarget(2, 0.0);
  }
  bool complete = false;
  while (!complete){
    tick();
    for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
      if(!axis[i].isAtTarget()){
        complete = false;
        continue;
        continue;
      }else{
        complete = true;
      }
    }
  }
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
    axis[i].setMotorZero();
  }
  return;
}


void Robot::homeAxes(uint8_t speed){
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
    axis[i].updatePosition();
    Serial.println(axis[i].getEncoderSteps());
  }
  bool complete = false;
  while (!complete){
    tick();
    for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
      if(!axis[i].isAtTarget()){
        complete = false;
        continue;
      }else{
        complete = true;
      }
    }
  }
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
    axis[i].setMotorZero();
  }
  return;
}


void Robot::configureAxis(uint8_t axisIndex, int axisOffset, float axisMin, float axisMax){
  axis[axisIndex] = RobotAxis(axisIndex,axisOffset,axisMin,axisMax );
  return;
}

void Robot::tick(){
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
    axis[i].tick();
  }
}

bool Robot::isAtTarget(){
  int atTarget = 0;
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
    if(axis[i].isAtTarget()){
      atTarget++;
    }
  }
  if(atTarget == sizeof(axis)/sizeof(RobotAxis)){
    Serial.println("Yep");
    return true;
  }else{
    Serial.print("Nope: ");
    Serial.println(atTarget);
    return false;
  }
  
}