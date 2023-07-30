#include "Arduino.h"

class Motor{
    private uint8_t motorAddress; //Motor offset from 0xe0
    private uint8_t speed; //Motor speed (0-127)
    private uint32_t lastKnownPosition; //Position offset from zero (steps)
    private bool isMoving;
    private float reductionRatio;

    // Serial Command Byte Prefixes

    //  Get Parameters
    private const byte Get_Steps =      {0x33}; // Returns int32_t number of pulses
    private const byte Get_Position =   {0x30}; // Returns uint16_t (documentation is wrong, need to test)
    private const byte Get_Angle =      {0x36}; // Returns int32_t shaft angle
    private const byte Get_Error =      {0x39}; // Returns int16_t angle error (also think doc is wrong on this)
    private const byte Is_Enabled =     {0x3a}; // Returns uint8_t: 1-Enable, 2-Disable, 0-Error
    private const byte Shaft_Blocked =  {0x3e}; // Returns uint8_t: 1-Blocked, 2-Unblocked, 0-Error

    //  Set Parameters
    private const byte Calibrate[] =    {0x80,0x00}; // Returns Success/Failure after calibration
    private const byte Set_Motor_Type = {0x81}; // Send uint8_t: 0-0.9deg, 1-1.8deg
    private const byte Set_Work_Mode =  {0x82}; // Send uint8_t: 0-OpenLoop, 1-ClosedLoop, 2-UART
    private const byte Set_Current =    {0x83}; // Send uint8_t: 0-0mA, 1-200mA, 2-400mA, 3- 600mA
                                                    // 4-800mA, 5-1000mA, 6-1200mA, 7-1400mA, 8-1600mA
                                                    // 9-1800mA, a-2000mA, b-2200mA, c-2400mA, d-2600mA
                                                    // e-2800mA, f-3000mA
    private const byte Set_Microsteps = {0x84}; // Send uint8_t: 1-256 microsteps (00 is 256)
    private const byte Set_En_Type =    {0x85}; // Send uint8_t: 0-AcivteLow, 1-ActiveHigh, 2-AlwaysActive
    private const byte Set_Rot_Dir =    {0x86}; // Send uint8_t: 0-CW, 1-CCW
    private const byte Set_Screen_Off = {0x87}; // Send uint8_t: 0-Disable, 1-Enable
    private const byte Set_Protect =    {0x88}; // Send uint8_t: 0-Disable, 1-Enable
    private const byte Set_Multiply =   {0x89}; // Send uint8_t: 0-Disable, 1-Enable
    private const byte Set_Baud_Rate =  {0x8a}; // Send uint8_t: 0-Off, 1-9600, 2-19200, 3-25000
                                                    // 4-38400, 5-57600, 6-115200
    private const byte Set_UART_Addr =  {0x8b}; // Send uint8_t: 0-e0, 1-e1, ..., 9-e9

    //  Set Zero Parameters
    private const byte Set_Zero_Mode =  {0x90}; // Send uint8_t: 0-Disable, 1-DirMode, 2-NearMode
    private const byte Set_New_Zero[] = {0x91,0x00}; // Returns Success/Failure Upon Reset
    private const byte Set_Zero_Speed = {0x92}; // Send uint8_t: 0-4 (lower is faster)
    private const byte Set_Zero_Dir =   {0x93}; // Send uint8_t: 0-CW, 1-CCW
    private const byte Go_To_Zero[] =   {0x94,0x00}; //Returns Success/Failure upon Completion

    //  Set PID/Accel/Torque Parameters
    private const byte Set_KP =         {0xa1}; // Send uint16_t, default 0x120
    private const byte Set_KD =         {0xa3}; // Send uint16_t, default 0x650
    private const byte Set_KI =         {0xa2}; // Send uint16_t, default 0x01
    private const byte Set_Accel =      {0xa4}; // Send uint16_t, default 0x11e
    private const byte Set_Max_Torq =   {0xa5}; // Send uint16_t, 0x0-0x480, default 0x480

    //  Reset All Parameters to Factory
    private const byte Reset_Params =   {0x3f}; // Returns Success/Failure on Completion
    
    //  UART Motor Control Commands
    private const byte UART_Enable =    {0xf3}; // Send uint8_t: 0-Disable, 1-Enable
    private const byte Rotate_Speed =   {0xf6}; // Send int8_t: -127 to 127, negative is reverse
    private const byte Auto_Rotate =    {0xff}; // Send uint8_t: c8-Save, ca-Clear
    private const byte Stop_Motor =     {0xf7}; // Returns Success/Failure on Completion
    private const byte Move_Steps =     {0xfd}; // Send int8_t speed, int32_t steps




    uint8_t SetZero(void);
    int32_t GetPosition(void);
    uint8_t MoveRelative(int32_t steps);
    uint8_t MoveAbsolute(int32_t steps);
    uint8_t StartRotation(int8_t speed);
    bool    IsMoving(void);
    uint8_t Stop(void);


}