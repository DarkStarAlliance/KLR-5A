#pragma once
#include "Arduino.h"   
#include <Bounce2.h>      // Library for debouncing inputs 
#include "teensystep4.h"  // Library for fast, asynchronous stepper motor control on Teensy4
#include "RobotAxis.h"
#include "Joystick.h"
using namespace TS4;      // Namespace for TeensyStep4

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
    /*enum controlMode {T1,T2,AUTO,AUTOEXT};
    controlMode& operator++(controlMode& orig) {  //pre increment operator
      orig = static_cast<controlMode>(orig + 1);
      return orig;
    }
    controlMode operator++(controlMode& orig, int) {  //post increment operator
    controlMode rVal = orig;
    ++orig;
    return rVal;}*/

    RobotAxis axis[5];
    int currentPose[5];
    int targetPose[5];
    bool estop;
    bool mstop;
    bool fault;
    int faultCode;
    bool moving;
    bool calibrated;
    JoyStick pendant;

  public:
    Robot();
    //pass in the number of axis and then a two dimension array of ints for pins [numAxis][7]
    //and a two dimensional array of doubles for pid [numAxis][2]
    Robot(int, int**, double**);
    
    void enableMotors();
    void disableMotors();

    void getCurrentPose();
    void getTargetPose();
    void setTargetPose();
    void setHomePose();
    


};//end of Robot class
Robot::Robot(){}
Robot::Robot(int numAxis, int** pins, double** pid) {
//************* would it be easier to leave first row blank, so axis 1 is in row 1?????
    
  //set up the pins and pid and everything else for all the RobotAxis      
  for(int i=0;i<numAxis;i++){
    axis[i].setPins(pins[i][0],pins[i][1],pins[i][2], pins[i][3],pins[i][4],pins[i][5],pins[i][6],pins[i][7]);
    axis[i].setPid(pid[i][0],pid[i][1],pid[i][2]);
    axis[i].setHomingSpeed(5000);
    //stepPin is [5], directionPin is [4]
    axis[i].setMotor(pins[i][5], pins[i][4]);
     // TS4::begin(); //Begin TeensyStep4 Service
    axis[i].setHomeSensor();
    axis[i].setEndStop();
    axis[i].setCalibrated(false);
    axis[i].setMoving(false);
    axis[i].setFault(true);
    axis[i].setEnabled(false);
    axis[i].setFaultCode(7); //Not Calibrated
  }

      //Check EEPROM for Hard or Soft Stop Stored Positions
      //If valid, clear fault and change faultCode to 6 for "Unverified Calibration" 
} //end of constructor

void Robot::enableMotors(){
  estop = false;
  mstop = false;
}
    
void Robot::disableMotors(){
  estop = true;
  mstop = true;
}

void Robot::getCurrentPose(){}
void Robot::getTargetPose(){}
void Robot::setTargetPose(){}
//this will place the robot 
void Robot::setHomePose(){
//this will set the Axis 2 in the home poisiton and Axis 3 in the home position
}
    