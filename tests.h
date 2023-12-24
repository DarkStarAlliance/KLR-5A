#pragma once
#include "Arduino.h"
#include "Robot.h"
#include "RobotAxis.h"

enum moveMode {ABSOLUTE, RELATIVE};
class tests {
  private:
    //array of pointers to test functions
    bool (*test_array[10])(RobotAxis &robotAxis, int degree, moveMode mode);
    Robot *robot_ptr;
    RobotAxis *axis1_ptr;
    RobotAxis *axis2_ptr;
    RobotAxis *axis3_ptr;
    
    //utility test functions    
    //test +45 degrees from home position of 90 degrees
    bool rotatePlus45degrees(RobotAxis &robotAxis, int degrees=45, moveMode = ABSOLUTE){
      Serial.println("Testing up by 45 degrees");
      Serial.println("First move to center vertical position (home position)");
      robot_ptr -> setHomePose();
      Serial.println("Then move Axis + 45 degrees");
      robot_ptr->setTargetPose();    
      return false;   
    }  
  
  public:
  tests(Robot &robot);
  bool runTest(int testnumber, RobotAxis &robotAxis, int degree, moveMode mode);
  bool runTest(bool(*)(RobotAxis &robotAxis, int degree, moveMode mode)); //pass a function pointer
  bool runAllTests();
};

tests::tests(Robot &robot){
  robot_ptr = &robot;
}
bool tests::runTest(int testnumber, RobotAxis &robotAxis, int degree, moveMode mode){
    return test_array[testnumber](robotAxis, degree, mode);
}
bool tests::runTest(bool(*)(RobotAxis &robotAxis, int degree, moveMode mode)){return false;}
bool tests::runAllTests(){ return false;}