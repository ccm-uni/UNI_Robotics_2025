#ifndef main
#define main

#include <Bluepad32.h>
#include <RoboClaw.h>
#include <ESP32Servo.h>

#include "HUSKYLENS/HUSKYLENS.h"  // Camera

//
#define driver1Addr 0x80 // RoboClaw 1 Address
#define driver2Addr 0x81 // RoboClaw 2 Address

// Motor maximum speed
#define maxSpeed 3300

#define servo1Pin 26


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

HUSKYLENS camera;

// 
RoboClaw motorDriver(&Serial2, 5000);

Servo
  swingArm,
  servo1,
  servo2,
  servo3;


// Need this because nested structs not allowed in ArduninC, apparently
struct MotorStuff {
  int16_t
    FL_Motor = 0,
    FR_Motor = 0,
    BL_Motor = 0,
    BR_Motor = 0;
};


// Struct to store motor data
// Nested structs not allowed
struct MotorStruct {
  MotorStuff targetPow;
};

// Easy way to reach the controller values
struct ControllerButtons {
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

  byte deadzone = 16;
};



ControllerButtons control;
MotorStruct motors;

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