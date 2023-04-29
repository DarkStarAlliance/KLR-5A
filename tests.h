#pragma once
#include "Arduino.h"
#include "Robot.h"
#include "RobotAxis.h"
//void setup(){}
//void loop(){}

class tests {
  private:
    //array of pointers to test functions
    bool (*test_array[10])(RobotAxis &robotAxis, int degree);
    Robot *robot_ptr;
    RobotAxis *axis2_ptr;
    RobotAxis *axis3_ptr;
    
    //utility test functions    
    //test Axis 2 rotate +45 degrees from home position 90 degrees
    bool rotatePlus45degrees(int degrees=45){
      Serial.println("Testing Axis 2 up by 45 degrees");
      Serial.println("First move to Axis 2 to center verticle position");
      robot_ptr -> setHomePose();
      Serial.println("Then move Axis + 45 degrees");
      robot_ptr->setTargetPose();    
      return false;   
    }  
  
  public:
  tests(Robot &robot);
  bool runTest(int testnumber, RobotAxis &robotAxis, int degree);
  bool runTest(bool(*)(RobotAxis &robotAxis, int degree)); //pass a function pointer
  bool runAllTests();
};

tests::tests(Robot &robot){
  robot_ptr = &robot;
}
bool tests::runTest(int testnumber, RobotAxis &robotAxis, int degree){
    return test_array[testnumber](robotAxis, degree);
}
bool tests::runTest(bool(*)(RobotAxis &robotAxis, int degree)){return false;}
bool tests::runAllTests(){ return false;}