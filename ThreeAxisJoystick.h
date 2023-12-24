#include "Arduino.h"   

enum axis {X,Y,Z};
axis& operator++(axis& orig) {  //pre increment operator
  orig = static_cast<axis>(orig + 1);
  return orig;
}
axis operator++(axis& orig, int) {  //post increment operator
  axis rVal = orig;
  ++orig;
  return rVal;}

  #define DEFAULT_DEADZONE 70
  #define DEFAULT_HOME_POSITION 512
  #define DEFAULT_CUTOFF 365

class ThreeAxisJoystick{
  private:
    int pin[3];
    bool invert[3];
    int home[3];            // Home value for zeroing Joystick input [0-1023]
    int deadzone[3]; // How far off of home does joystick need to be to activate? [0-255] 
    int cutoff[3];
  public:
    ThreeAxisJoystick();
    ThreeAxisJoystick(int setPin[3]); //constructor
    void setPinModes();    
    uint16_t getHome(axis direction);
    uint8_t getDeadzone(axis selectedAxis) {return deadzone[selectedAxis];}
    uint16_t getRawPosition(axis direction); 
    int getPosition(axis selectedAxis);
    float getNormalizedPosition(axis selectedAxis);
    void getXYZ(uint16_t &x, uint16_t &y, uint16_t &z);
    void setHome();
    bool invertAxis(axis selectedAxis);
    bool inverted(axis selectedAxis) {return invert[selectedAxis];}
};//end of Joystick class

ThreeAxisJoystick::ThreeAxisJoystick(){
}

ThreeAxisJoystick::ThreeAxisJoystick(int setPin[3]) {  
  for(int i=0;i<3;i++){
    pin[i]= setPin[i];
    home[i]=DEFAULT_HOME_POSITION;
    invert[i] = false;
    deadzone[i] = DEFAULT_DEADZONE;
    cutoff[i] = DEFAULT_CUTOFF;
  }
  setPinModes();
} //end of constructor

void ThreeAxisJoystick::setPinModes() { 
  for(int i=0;i<3;i++){
    pinMode(pin[i],INPUT);
  }
} //end of setPinModes    

void ThreeAxisJoystick::getXYZ(uint16_t &x, uint16_t &y, uint16_t &z){
          x = getPosition(axis::X);
          y = getPosition(axis::Y);
          z = getPosition(axis::Z);                  
} // end of getAllPositions 

void ThreeAxisJoystick::setHome(){
  axis direction = axis::X;
  for(int i=0;i<3;i++)
    home[i] = getRawPosition(direction++);
} //end of setHome

uint16_t ThreeAxisJoystick::getHome(axis direction){
  return home[direction];
}

bool ThreeAxisJoystick::invertAxis(axis selectedAxis){
  invert[selectedAxis] = !invert[selectedAxis];
  return invert[selectedAxis];
}

int ThreeAxisJoystick::getPosition(axis selectedAxis){
  int position = analogRead(pin[selectedAxis]);

  if(abs(position-home[selectedAxis])<deadzone[selectedAxis]){
    return 0;
  }else{
    position = position-home[selectedAxis];
    if(position>0){
      position = position-deadzone[selectedAxis];
      if(position>cutoff[selectedAxis]){
        position = cutoff[selectedAxis];
      }
    }else{
      position = position+deadzone[selectedAxis];
      if(position<(0-cutoff[selectedAxis])){
        position = 0-cutoff[selectedAxis];
      }
    }
    if(invert[selectedAxis]){
      position = position *-1;
    }
    return position;
  }
}

float ThreeAxisJoystick::getNormalizedPosition(axis selectedAxis){
    return (float)getPosition(selectedAxis)/cutoff[selectedAxis];
}

uint16_t ThreeAxisJoystick::getRawPosition(axis direction){
  return analogRead(pin[direction]);
}

