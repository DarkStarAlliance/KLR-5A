

/* Includes Files (libraries) =================================================== */
#include <SerialMenuCmd.h> //Library referred to in this example

/* Macros =============================================================================== */
#define LedOnBoard 13

/* Class (instanciation) =============================================================== */
/**
 * @brief Instanciation of the SerialMenuCmd library (Class)
 *        This library allows thez user(développeur) to implement a basic CLI (Common Line Interface).
 *        Operation is based on the serial monitor, for the user it consists of consulting a menu
 *        and then activating a command by sending the corresponding code.
 *        
 *        example menu (the text must include the corresponding code or othewise the key):
 *        a - todo command 1 
 *        b - todo command 2
 *        1 - todo command 3
 *        * - todo command ....
 * 
 *        The menu items are put together in a structure MenCmdItem, see further in the file
 */
SerialMenuCmd myMmuCmd;
int CommandReceived[] = {0xe0,0x01,0xe1};
const byte SetZero[] = {0xe0,0x91,0x00,0x71};
//const byte GetPosition[] = {0xe0,0x30,0x10};
const byte GetSteps[]= {0xe0,0x33,0x13};
const byte StopMotor[] = {0xe1,0xf7,0xd8};
const byte MoveMotor[] = {0xe0, 0xfd, 0x06, 0x00,0x00, 0xEE, 0x48, 0x19};
const byte BackMotor[] = {0xe0, 0xfd, 0x86, 0x00,0x00, 0xEE, 0x48, 0x99};
const byte SetKP[] = {0xe0, 0xa1, 0x06, 0x50, 0xd7}; //Default 0x650
const byte SetKD[]  = {0xe0,0xa3,0x06,0x50,0xd9} ;//Defualt 0x650
const byte SetKI[] = {0xe0,0xa2,0x00,0x01,0x83}; // Default 1
const byte SetAccel[] = {0xe0, 0xa4, 0x01, 0x1e, 0xA3}; //Default 0x11e
const byte SetTorque[] = {0xe0,0xa5,0x02,0x58,0xdf}; //0 to 0x4b0, default 0x4b0



/* Variables ============================================================================= */
uint8_t motorIndex=0;
uint8_t speed;
int32_t moveSteps;
int32_t posSteps;
byte shortMotorResponse[3];
int numBytes;


/* Function Prototype(s) ================================================================= */

/* CLI( Command Line Interfaface) making of================================================ */
//Declaration texts of menu
tMenuCmdTxt txt1_SetZero[] = "z - Set Zero Position";
tMenuCmdTxt txt2_GetPosition[] = "p - Get Current Position";
tMenuCmdTxt txt3_GetSteps[] = "t - Get Current Steps";
tMenuCmdTxt txt4_MoveRelative[] = "m - Move Motor Relative";
tMenuCmdTxt txt5_StopMotor[] = "x - Stop Motor";
tMenuCmdTxt txt6_MoveAbsolute[] = "b - Move Motor Absolute";
tMenuCmdTxt txt7_Status[] = "s - Status";
tMenuCmdTxt txt8_Menu[] = "n - menu";

tMenuCmdTxt txt3_DisplayMenu[] = "? - Displaying menu";

//Declaration text of prompt
tMenuCmdTxt txt_Prompt[] = "Input:";

//Prototype of function which are callback by the library
void cmd1_SetZero(void);
void cmd2_GetPosition(void);
void cmd3_GetSteps(void);
void cmd4_MoveRelative(void);
void cmd5_StopMotor(void);
void cmd6_MoveAbsolute(void);
void cmd7_Status(void);
void cmd8_Menu(void);

//Structure initialisation
//Data type
//array sMenuTxt, code (character) , function (Reminder the code character must be printable character)
stMenuCmd list[] = {
    {txt1_SetZero, 'z', cmd1_SetZero},
    {txt2_GetPosition, 'p', cmd2_GetPosition},
    {txt3_GetSteps, 't', cmd3_GetSteps},
    {txt4_MoveRelative, 'm', cmd4_MoveRelative},
    {txt5_StopMotor, 'x', cmd5_StopMotor},
    {txt6_MoveAbsolute, 'b', cmd6_MoveAbsolute},
    {txt7_Status, 's', cmd7_Status},  
    {txt8_Menu, 'n', cmd8_Menu}
};

 union fourByte{
   uint8_t b[4];
   uint32_t i;
 };

 union twoByte{
  uint8_t b[2];
  uint16_t i;
 };

//KmenuCount contains the number of command
#define NbCmds sizeof(list) / sizeof(stMenuCmd)


/* Functions Implementation =================================================== */

/**
 * @brief Standard function Arduino for initialisation
 * 
 */
void setup()
{
  //Activate and initialize the serial bus with the baudrate passed in parameter
  Serial.begin(115200);
  Serial2.begin(115200);


  //setParam();
  //Initialize the library with structure defined by the user
  if (myMmuCmd.begin(list, NbCmds, txt_Prompt) == false)
  {
    while (true)
    {
      Serial.println("Serial menu initialization failed!");
    }
  }

  //Display the menu
  myMmuCmd.ShowMenu();
  myMmuCmd.giveCmdPrompt();
 
}

void loop()
{
  uint8_t CmdCode;

  /**
   * @brief management of the interaction between the system and the user.
   * The "UserRequest" menbre function analyzes the characters transmitted by the user. 
   * If an command code is identified, its number is returned (return 0 if no command). This 
   * function is not blocking, it stores the intermediate data between 2 calls. 
   */
  CmdCode = myMmuCmd.UserRequest();

  //possible pre-treatment here

  /**
   * @brief Execute Command
   * if a command code is returned, the system executes the corresponding command. 
   * To do this, it uses the "OpsCallback" member function. this function receives 
   * the command code parameter
   * 
   * @note In this way, it is possible to carry out a preprocessing and postprocessing
   */
  if (CmdCode != 0)
  {
    myMmuCmd.ExeCommand(CmdCode);
  }
}
void cmd1_SetZero(void)
{
  while(Serial2.available()){ //Check incoming serial buffer
    Serial2.read(); // Empty if there is anything in it
  }

  Serial.println("Resetting motor zero position...");
  Serial2.write(SetZero,sizeof(SetZero)); //Send command to reset zero position of motor
  while(Serial2.available()<sizeof(CommandReceived)/sizeof(CommandReceived[0])){ //Wait for expected message length in buffer
    //delay(10);
  }
  numBytes = Serial2.available();
  for(int i = 0; i<numBytes; i++){
    shortMotorResponse[i]=Serial2.read();
  }
  if(shortMotorResponse[0]!=0xe0){
    Serial.println("Unexpected Serial Response(Address)");
    Serial.println(shortMotorResponse[0]);
  }else{
    if(shortMotorResponse[1]!=1){
      if(shortMotorResponse[1]==0){
        Serial.println("Motor Reported Action Failed!");
      }else{
        Serial.println("Unexpected Serial Response(Confirmation)");
        Serial.println(shortMotorResponse[1]);
      }
    }else{
      if((uint8_t)(shortMotorResponse[0]+shortMotorResponse[1])!=shortMotorResponse[2]){
        Serial.println("Unexpected Serial Response(Checksum)");
        Serial.println(shortMotorResponse[2]);
      }else{
        Serial.println("Reset Action Successful!");
      }
    }
  }
}

void cmd2_GetPosition(void)
{
  Serial.println("Not Yet Implemented!");  
    
  

  myMmuCmd.giveCmdPrompt();
}

void cmd3_GetSteps(void)
{
  while(Serial2.available()){
    Serial2.read();
  }
  int bytesAvailable;
  int32_t steps;
  byte incomingMsg[4];
  Serial.println("Current Steps: ");
  Serial2.write(GetSteps,sizeof(GetSteps));
  delay(20);
  bytesAvailable = Serial2.available();
  for(int i = 0; i<bytesAvailable; i++){
    if(i>0 && i<bytesAvailable-1){
      incomingMsg[i-1]=Serial2.read();
    }else{
      Serial2.read();
    } 
  }
  steps = (int32_t)(incomingMsg[0] << 24) + (incomingMsg[1] << 16) + (incomingMsg[2] << 8) + incomingMsg[3];
  Serial.println(steps);
  Serial.print("Motor Degrees:");
  Serial.println(steps*0.45);
  Serial.print("Axis Degrees:");
  Serial.println(steps*(1.0/(61000.0/360.0)));
  myMmuCmd.giveCmdPrompt();
}

void cmd4_MoveRelative(void)
{
  int8_t speed;
  float inputVal;
  String aValue = "! Enter desired movement in degrees";
  union fourByte mover;
  //                   Mtr#|Move|Sped|4 Byte Number Steps|tCHK
  byte moveMessage[8]={0xe0,0xfd,0x86,0x00,0x00,0x00,0x00,0x00};
  if(myMmuCmd.getStrValue(aValue) == true)
  {
    String aSpeed = "! Enter desired speed 0-127";
    if(myMmuCmd.getStrValue(aSpeed)==true){
      // Serial.println(F(""));
      // Serial.print(F("Duration of LED switch off = "));
        inputVal = (int32_t)round(((61000.0/360.0))*atof(aValue.c_str()));
      speed = atoi(aSpeed.c_str());
    }

  } 
  if(speed<0){
    Serial.println("Only Positive Speed Values Allowed!");
    speed = speed*-1;
  }else{
    if(speed>127){
      Serial.println("Maximum speed 127!");
      speed = 127;
    }
  }
  moveMessage[2]= speed;
  if(inputVal<0){
    inputVal = inputVal*-1;
    moveMessage[2] = speed + 128;
  }
  mover.i = inputVal;
 
  Serial.println(F(""));
  myMmuCmd.giveCmdPrompt();
  for(int i=3;i<sizeof(moveMessage)-1;i++){
    moveMessage[i] = mover.b[6-i];
  }
  uint16_t checksum =0;
  for (int i=0; i<sizeof(moveMessage)-1;i++){
    checksum += moveMessage[i];
  }
  moveMessage[7]= checksum;
  Serial2.write(moveMessage,sizeof(moveMessage));
 
 while(Serial2.available()<sizeof(CommandReceived)/sizeof(CommandReceived[0])){ //Wait for expected message length in buffer
  }
  numBytes = Serial2.available();
  for(int i = 0; i<numBytes; i++){
    shortMotorResponse[i]=Serial2.read();
  }
  if(shortMotorResponse[0]!=0xe0){
    Serial.println("Unexpected Serial Response(Address)");
    Serial.println(shortMotorResponse[0]);
  }else{
    if(shortMotorResponse[1]!=1){
      if(shortMotorResponse[1]==0){
        Serial.println("Motor Reported Action Failed!");
      }else{
        Serial.println("Unexpected Serial Response(Confirmation)");
        Serial.println(shortMotorResponse[1]);
      }
    }else{
      if((uint8_t)(shortMotorResponse[0]+shortMotorResponse[1])!=shortMotorResponse[2]){
        Serial.println("Unexpected Serial Response(Checksum)");
        Serial.println(shortMotorResponse[2]);
      }else{
        Serial.println("Motor Move Started...");
      }
    }
  }

  /*while(Serial2.available()<sizeof(CommandReceived)/sizeof(CommandReceived[0])){ //Wait for expected message length in buffer

  }
  numBytes = Serial2.available();
  for(int i = 0; i<numBytes; i++){
    shortMotorResponse[i]=Serial2.read();
  }
  if(shortMotorResponse[0]!=0xe0){
    Serial.println("Unexpected Serial Response(Address)");
    Serial.println(shortMotorResponse[0]);
  }else{
    if(shortMotorResponse[1]!=2){
      if(shortMotorResponse[1]==0){
        Serial.println("Motor Reported Action Failed!");
      }else{
        Serial.println("Unexpected Serial Response(Confirmation)");
        Serial.println(shortMotorResponse[1]);
      }
    }else{
      if((uint8_t)(shortMotorResponse[0]+shortMotorResponse[1])!=shortMotorResponse[2]){
        Serial.println("Unexpected Serial Response(Checksum)");
        Serial.println(shortMotorResponse[2]);
      }else{
        Serial.println("Motor Move Complete!");
      }
    }
  }
*/


  myMmuCmd.giveCmdPrompt();
}


void cmd5_StopMotor(void)
{
  Serial.println("Stopping Motor...");
  Serial2.write(StopMotor,sizeof(StopMotor));
 while(Serial2.available()<sizeof(CommandReceived)/sizeof(CommandReceived[0])){ //Wait for expected message length in buffer
    //delay(10);
  }
  numBytes = Serial2.available();

  for(int i = 0; i<numBytes; i++){
    shortMotorResponse[i]=Serial2.read();
  }
  if(shortMotorResponse[0]!=0xe0){
    Serial.println("Unexpected Serial Response(Address)");
    Serial.println(shortMotorResponse[0]);
  }else{
    if(shortMotorResponse[1]!=1){
      if(shortMotorResponse[1]==0){
        Serial.println("Motor Reported Action Failed!");
      }else{
        Serial.println("Unexpected Serial Response(Confirmation)");
        Serial.println(shortMotorResponse[1]);
      }
    }else{
      if((uint8_t)(shortMotorResponse[0]+shortMotorResponse[1])!=shortMotorResponse[2]){
        Serial.println("Unexpected Serial Response(Checksum)");
        Serial.println(shortMotorResponse[2]);
      }else{
        Serial.println("Stop Action Successful!");
      }
    }
  }

  myMmuCmd.giveCmdPrompt();

}

void cmd6_MoveAbsolute(void)
{
int8_t speed;
  float inputVal;
  String aValue = "! Enter desired target in degrees";
  union fourByte mover;
  //                   Mtr#|Move|Sped|4 Byte Number Steps|tCHK
  byte moveMessage[8]={0xe0,0xfd,0x86,0x00,0x00,0x00,0x00,0x00};
  if(myMmuCmd.getStrValue(aValue) == true)
  {
    String aSpeed = "! Enter desired speed 0-127";
    if(myMmuCmd.getStrValue(aSpeed)==true){
      // Serial.println(F(""));
      // Serial.print(F("Duration of LED switch off = "));
        inputVal = (int32_t)round(((61000.0/360.0))*atof(aValue.c_str())); //Desired target location in degrees
      speed = atoi(aSpeed.c_str()); //Desired target speed to location
    }

  } 
  if(speed<0){
    Serial.println("Only Positive Speed Values Allowed!");
    speed = speed*-1;
  }else{
    if(speed>127){
      Serial.println("Maximum speed 127!");
      speed = 127;
    }
  }
  moveMessage[2]= speed;
  if(inputVal<0){
    inputVal = inputVal*-1;
    moveMessage[2] = speed + 128;
  }
  mover.i = inputVal;
 
  Serial.println(F(""));
  myMmuCmd.giveCmdPrompt();
  for(int i=3;i<sizeof(moveMessage)-1;i++){
    moveMessage[i] = mover.b[6-i];
  }
  uint16_t checksum =0;
  for (int i=0; i<sizeof(moveMessage)-1;i++){
    checksum += moveMessage[i];
  }
  moveMessage[7]= checksum;
  Serial2.write(moveMessage,sizeof(moveMessage));
 
 while(Serial2.available()<sizeof(CommandReceived)/sizeof(CommandReceived[0])){ //Wait for expected message length in buffer
  }
  numBytes = Serial2.available();
  for(int i = 0; i<numBytes; i++){
    shortMotorResponse[i]=Serial2.read();
  }
  if(shortMotorResponse[0]!=0xe0){
    Serial.println("Unexpected Serial Response(Address)");
    Serial.println(shortMotorResponse[0]);
  }else{
    if(shortMotorResponse[1]!=1){
      if(shortMotorResponse[1]==0){
        Serial.println("Motor Reported Action Failed!");
      }else{
        Serial.println("Unexpected Serial Response(Confirmation)");
        Serial.println(shortMotorResponse[1]);
      }
    }else{
      if((uint8_t)(shortMotorResponse[0]+shortMotorResponse[1])!=shortMotorResponse[2]){
        Serial.println("Unexpected Serial Response(Checksum)");
        Serial.println(shortMotorResponse[2]);
      }else{
        Serial.println("Motor Move Started...");
      }
    }
  }

  /*while(Serial2.available()<sizeof(CommandReceived)/sizeof(CommandReceived[0])){ //Wait for expected message length in buffer

  }
  numBytes = Serial2.available();
  for(int i = 0; i<numBytes; i++){
    shortMotorResponse[i]=Serial2.read();
  }
  if(shortMotorResponse[0]!=0xe0){
    Serial.println("Unexpected Serial Response(Address)");
    Serial.println(shortMotorResponse[0]);
  }else{
    if(shortMotorResponse[1]!=2){
      if(shortMotorResponse[1]==0){
        Serial.println("Motor Reported Action Failed!");
      }else{
        Serial.println("Unexpected Serial Response(Confirmation)");
        Serial.println(shortMotorResponse[1]);
      }
    }else{
      if((uint8_t)(shortMotorResponse[0]+shortMotorResponse[1])!=shortMotorResponse[2]){
        Serial.println("Unexpected Serial Response(Checksum)");
        Serial.println(shortMotorResponse[2]);
      }else{
        Serial.println("Motor Move Complete!");
      }
    }
  }
*/


  myMmuCmd.giveCmdPrompt();
}

void cmd7_Status(void)
{
}

void cmd8_Menu(void)
{
  myMmuCmd.ShowMenu();
  myMmuCmd.giveCmdPrompt();
}

void setParam(void){
  Serial.println("Setting KP...");
  Serial2.write(SetAccel,sizeof(SetAccel));
 while(Serial2.available()<sizeof(CommandReceived)/sizeof(CommandReceived[0])){ //Wait for expected message length in buffer
    //delay(10);
  }
  numBytes = Serial2.available();

  for(int i = 0; i<numBytes; i++){
    shortMotorResponse[i]=Serial2.read();
  }
  if(shortMotorResponse[0]!=0xe0){
    Serial.println("Unexpected Serial Response(Address)");
    Serial.println(shortMotorResponse[0]);
  }else{
    if(shortMotorResponse[1]!=1){
      if(shortMotorResponse[1]==0){
        Serial.println("Motor Reported Action Failed!");
      }else{
        Serial.println("Unexpected Serial Response(Confirmation)");
        Serial.println(shortMotorResponse[1]);
      }
    }else{
      if((uint8_t)(shortMotorResponse[0]+shortMotorResponse[1])!=shortMotorResponse[2]){
        Serial.println("Unexpected Serial Response(Checksum)");
        Serial.println(shortMotorResponse[2]);
      }else{
        Serial.println("KP Set Successfully!");
      }
    }
  }
}