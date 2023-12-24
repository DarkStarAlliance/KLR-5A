#include "Arduino.h"   
#include <Bounce2.h>      // Library for debouncing inputs 
#include "RobotAxis.h"
#include "ThreeAxisJoystick.h"
#include "AS5600.h"
#include <array>

  enum controlMode {T1,T2,AUTO,AUTOEXT};
  controlMode& operator++(controlMode& orig) {  //pre increment operator
    orig = static_cast<controlMode>(orig + 1);
    return orig;
  }
  controlMode operator++(controlMode& orig, int) {  //post increment operator
  controlMode rVal = orig;
  ++orig;
  return rVal;}

class Robot{
  
  private:


    RobotAxis axis[3];
    std::array<float,3> currentPose;
    int targetPose[3];
    bool estop;
    bool mstop;
    bool faulted;
    int faultCode;
    bool moving;
    bool calibrated;
    bool enabled;
    ThreeAxisJoystick pendant;

  public:
    Robot();
    void enableMotors();
    void disableMotors();
    
    void homeAxes();
    void homeAxes(uint8_t speed);
    std::array<float,3> getCurrentPose();
    //float* getCurrentPose();
    void getTargetPose();
    void setTargetPose();
    bool isFaulted(){ return faulted;}
    bool isAtTarget();
    
    void executePose(float *pose, uint8_t *poseSpeed);
    void runProgram(int programNumber);
    float* getAxisAngles();

    void configureAxis(uint8_t axisIndex, uint8_t encAddress, bool inverted, int axisOffset, float axisMin, float axisMax);
    void configurePendant(uint8_t pinX, uint8_t pinY, uint8_t pinZ, uint8_t pinButton);
    void configurePendant(uint8_t pinX, uint8_t pinY, uint8_t pinZ);
    void tick();
    


};//end of Robot class



Robot::Robot() {

      calibrated = false;
      moving = false;
      faulted = false;
      enabled = false;
      faultCode = 7; //Not Calibrated
} //end of constructor

void Robot::executePose(float *pose, uint8_t *poseSpeed){
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
  //Serial.println(pose[i]);
  axis[i].moveToTarget(poseSpeed[i],pose[i]);
  } 
}

void Robot::homeAxes(){
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
    axis[i].moveToTarget(6, 0.0);
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
    //Serial.println(axis[i].getEncoderSteps());
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

std::array<float,3> Robot::getCurrentPose(){
  return currentPose;
}

void Robot::configureAxis(uint8_t axisIndex, uint8_t encAddress, bool inverted, int axisOffset, float axisMin, float axisMax){
  axis[axisIndex] = RobotAxis(axisIndex,encAddress,inverted,axisOffset,axisMin,axisMax );
  return;
}

void Robot::configurePendant(uint8_t pinX, uint8_t pinY, uint8_t pinZ, uint8_t pinButton){
  int joyPins[3] = {pinX,pinY,pinZ};
  pendant = ThreeAxisJoystick(joyPins);
  pendant.setPinModes();
  delay(10);
  pendant.setHome();
}
void Robot::configurePendant(uint8_t pinX, uint8_t pinY, uint8_t pinZ){
  int joyPins[3] = {pinX,pinY,pinZ};
  pendant = ThreeAxisJoystick(joyPins);
  pendant.setPinModes();
  delay(10);
  pendant.setHome();
}

void Robot::tick(){
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
    if(axis[i].isFaulted()&&(axis[i].getFault()>1)){
      faulted = true;
      axis[i].motorStop();
      return;
    }
    axis[i].tick();
    //Serial.print(currentPose[i]);
    currentPose[i]= axis[i].getEncoderSteps()*AS5600_RAW_TO_DEGREES;
    //Serial.println(currentPose[i]);
    /*Serial.print("Axis ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(axis[i].getEncoderSteps());
    Serial.print("\t");*/
  }
  //fSerial.println("");
  /*Serial.print("X:");
  Serial.print(pendant.getNormalizedPosition(X));
  Serial.print("  Y:");
  Serial.print(pendant.getNormalizedPosition(Y));
  Serial.print("  Z:");
  Serial.println(pendant.getNormalizedPosition(Z));*/
}

bool Robot::isAtTarget(){
  int atTarget = 0;
  for(uint8_t i=0;i<sizeof(axis)/sizeof(RobotAxis);i++){
    if(axis[i].isAtTarget()){
      atTarget++;
    }
  }
  if(atTarget == sizeof(axis)/sizeof(RobotAxis)){
    //Serial.println("Yep");
    return true;
  }else{
    //Serial.print("Nope: ");
    //Serial.println(atTarget);
    return false;
  }
  
}