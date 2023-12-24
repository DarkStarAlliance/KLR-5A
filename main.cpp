//#include "AS5600.h"
//#include "RobotAxis.h"
#include "Robot.h"                    //Custom Library for controlling robot as single object
#include "Wire.h"                     //Library for TwoWire(I2C) device communciation [Axis Encoders and Display]
//#include <i2c_driver_wire.h>
#include "Adafruit_GFX.h"             //Library for drawing objects on I2C/SPI display
#include "Adafruit_SSD1306.h"         //Library for controlling OLED display
#include <FastLED.h>                  //Library for controlling addressable LEDs (WS2812b)
#include <array>                    
#include <IRremote.hpp>               //Library for sending/receiving IR signal codes
#include <PinButton.h>                //Library for managing and debouncing pushbuttons

PinButton inputButton[] = {38,39,40}; //Defines teensy pins for pushbuttons (Low-Active)

//PinButton button1(38);
//PinButton button2(39);
//PinButton button3(40);

#define DECODE_NEC                    //Defines Infrared Signal Standard to Decode

#define NUM_LEDS 7                    //Defines number of LED Pixels in addressable strip
#define LED_PIN 33                    //Defines pin number for addressable LEDs
CRGB leds[NUM_LEDS];                  //Create array of color data for addressble LEDs

#define IR_RECEIVE_PIN    41          //Define pin for Infrared Reciever


#define SCREEN_WIDTH 128              //Define OLED display width, in pixels
#define SCREEN_HEIGHT 32              //Define OLED display height, in pixels

#define OLED_RESET     -1             //Define OLED Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C           //Specify OLED I2C address < See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, 
                         SCREEN_HEIGHT, 
                         &Wire1, 
                         OLED_RESET); //Instantiate OLED display object using the secondary I2C interface (Wire1)

elapsedMillis lastTick;               //Timer for calculating processing ticks
elapsedMillis timer;                  //General use timer
Robot klarissa;                       //Instantiate Robot object
uint8_t joystickPins[] = {22,23,21};  //Define pins for joystick analog inputs (x, y and z)


uint8_t numAxes = 3;                  //Define number of controled axes

std::array<float,3> pose;             //Define float array for storing Robot pose data

uint8_t axisAddress[]= {0,1,2};      //Define serial UART address for controlling each axis
uint8_t encoderAddress[] = {0,2,4};  //Define encoder multiplexer address
bool encoderInvert[] = {false,false,false};

float axisBottom[]= {-45.0,           //Axis1         Lower limit for acceptable operation (degrees)
                      -40,            //Axis2
                      -115.0};        //Axis3

float axisTop[]= {45.0,               //Axis1         Upper limit for acceptable operation (degrees)
                   75.0,              //Axis2
                   20.0};             //Axis3

float homeOffset[]= {1893,            //Axis1         Step offset for home/zero degree position (Encoder steps 0-4096)
                      1563,           //Axis2
                      3200};          //Axis3

uint8_t defaultSpeed[] = {20,20,35};  //Define default axis speed

void setup(){ //~~~~~~~~~~~~~~~~~~~~~~~~  START OF setup() ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Serial.begin(115200);             //Begin Serial communication on USB serial interface (debugging)
    Serial2.begin(115200);            //Begin Serial communication on Serial1 interface (Motor control)
    
    //Initialize OLED Display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){
      Serial.println(F("SSD1306 allocation failed"));
    }
    //Initialize Addressable Pixels
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);

    //Initialize Infrared Reciever
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    //Initialize Robot object
    klarissa.configurePendant(joystickPins[0],joystickPins[1],joystickPins[2]);
    for(int i=0;i<numAxes;i++){
      klarissa.configureAxis(axisAddress[i],encoderAddress[i],encoderInvert[i],homeOffset[i],axisBottom[i],axisTop[i]);
    }

    //Set Feedback LEDs to Orange before homing procedure
    for(int i=0;i<NUM_LEDS;i++){
      leds[i] = CRGB::Orange;
    }
    FastLED.show();

    //Print homing message to Display
    display.clearDisplay();
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0); 
    display.println("Homing Axes");
    display.println("Please Wait");
    display.display();

    //Call axis homing routine on Robot object
    klarissa.homeAxes();

    //Set Feedback LEDs to Green after homing procedure
    for(int i=0;i<NUM_LEDS;i++){
      leds[i] = CRGB::Green;
    }
    FastLED.show();
}//XXXXXXXXXXXXXXXXXXXXXXXXXXXX     END OF setup()   XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

void loop(){ //~~~~~~~~~~~~~~~~~~~~~START OF loop()  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //Update pushbutton status and check for click event
    for(int i=0;i<3;i++){
      inputButton[i].update();
      if(inputButton[i].isClick()){
        Serial.print("Button");
        Serial.print(i+1);
        Serial.println(" Clicked");
      }
    }


    //Normal checks only if not faulted
    if(!klarissa.isFaulted()){
        //Once per MS
        if(lastTick>1){
          lastTick = 0;

          //Call Robot Tick function
          klarissa.tick();

          //Display current axis position on OLED
          display.clearDisplay();
          pose = klarissa.getCurrentPose();
          display.setTextSize(1);             // Normal 1:1 pixel scale
          display.setTextColor(SSD1306_WHITE);        // Draw white text
          display.setCursor(0,7);             // Start at top-left corner
          for(int i=0;i<3;i++){
            display.print("Axis");
            display.print(i+1);
            display.print(" ");
            display.print(pose[i]);
            display.println(" degrees");
          }
          display.display();
        }

        //Once every 10s and after Robot has reached target position
        if( (timer>10000) && klarissa.isAtTarget()){
          timer = 0;
          //Generate new random pose within 15deg of axis bottom and 30deg of axis top
          //  with random speed between 1 and double default speed
          float newPose[numAxes];
          uint8_t poseSpeed[numAxes];
          for(int i=0; i<numAxes;i++){
            newPose[i] = random(axisBottom[i]+15,axisTop[i]-30);
            poseSpeed[i] = random(1,defaultSpeed[i]*2);
          }

          //Execute randomly-generated pose
          klarissa.executePose(newPose, poseSpeed);
        }
    }else{ //If robot is faulted
      //Set feedback LEDs to red
      for(int i=0;i<NUM_LEDS;i++){
        leds[i] = CRGB::Red;
      }
      //Display fault message on OLED
      FastLED.show();
      display.clearDisplay();
      display.setTextSize(2);             // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);        // Draw white text
      display.setCursor(0,0); 
      display.println("ROBOT");
      display.println("FAULTED!");
      display.display();
    }

//If there is a waiting IR code
if (IrReceiver.decode()) {

        //Print debug message if unrecognized signal
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
            // We have an unknown protocol here, print more info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
        }
        IrReceiver.resume(); // Enable receiving of the next value
        //Perform action based on recognized messages
        if (IrReceiver.decodedIRData.command == 0x0) {
            Serial.println("IR_Button1");
        } else if (IrReceiver.decodedIRData.command == 0x1) {
            Serial.println("IR_Button2");
        }
    }
}// XXXXXXXXXXXXXXXXXXXXX   END OF loop()   XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
