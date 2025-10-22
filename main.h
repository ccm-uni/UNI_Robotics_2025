#ifndef main
#define main

#include <Bluepad32.h>
#include <RoboClaw.h>
#include <ESP32Servo.h>

#include "HUSKYLENS/HUSKYLENS.h"  // Camera

//
#define driver1Addr 0x80  // RoboClaw 1 Address
#define driver2Addr 0x81  // RoboClaw 2 Address

// Motor maximum speed
#define maxSpeed 3300

#define servo1Pin 13
#define servo2Pin 12
#define servo3Pin 14

#define swingArmPin 27


// Need this for the controllers to work
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Husky MV Camera object
HUSKYLENS camera;

//
RoboClaw motorDriver(&Serial2, 5000);

Servo
  swingArm,
  servo1,
  servo2,
  servo3;


// Need this because nested structs not allowed in ArduninC, apparently
struct MotorStruct {
  int16_t
    FL_Motor = 0,
    FR_Motor = 0,
    BL_Motor = 0,
    BR_Motor = 0;
};

struct ServoStruct {
  byte
    complientPos = 0,
    sortGatePos = 0,
    clawPos = 0;
};


// Struct to store motor data
// Nested structs not allowed
struct RobotStruct {
  MotorStruct motor;
  ServoStruct servo;
};


// Easy way to reach the controller values
struct ControllerStruct {
  int16_t
    LX = 0,
    LY = 0,
    RX = 0,
    RY = 0,
    L2 = 0,
    R2 = 0;

  bool
    dpad_up,
    dpad_down,
    dpad_left,
    dpad_right;

  bool
    cross = false,
    circle = false,
    triangle = false,
    square = false;

  bool
    L1 = false,
    R1 = false;

  bool
    connected = false;

  byte
    colorBar[3] = {0, 0, 0};

  byte deadzone = 16;
};



ControllerStruct controller;
RobotStruct robot;

// Speed mode
bool slowMode = false;
bool fastMode = false;


void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void dumpGamepad(ControllerPtr ctl);
void processGamepad(ControllerPtr ctl);
void processControllers();
bool safetyLoop();

#endif