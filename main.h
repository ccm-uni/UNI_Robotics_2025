#include <sys/_stdint.h>
#ifndef main
#define main

#include <Bluepad32.h>
#include <RoboClaw.h>
#include <ESP32Servo.h>

#include "HUSKYLENS/HUSKYLENS.h"  // Camera

#define SetWORDval(arg) (uint8_t)(((uint16_t)arg) >> 8), (uint8_t)arg

#define driver1Addr 0x80
#define driver2Addr 0x81

#define ctrlDeadzone 16
#define maxSpeed 3300

#define servo1Pin 26


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

HUSKYLENS camera;

RoboClaw motorDriver(&Serial2, 5000);

Servo servo1;



/**
 * Stores the current output powers of the motors
 * 0 - FLMotor
 * 1 - BLMotor
 * 2 - BRMotor
 * 3 - FRMotor
*/
int motorPowers[4] = { 0, 0, 0, 0 };


// Easy way to reach the controller values
struct ControllerButtons {
  int16_t
    LX = 0,
    LY = 0,
    RX = 0,
    RY = 0,
    L2 = 0,
    R2 = 0;

  uint16_t
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
};


/**
 * Stores the current target powers of the motors
 *
 * 0 - FLMotor
 * 1 - FRMotor
 * 2 - BLMotor
 * 3 - BRMotor
*/
int desiredPowers[4] = { 0, 0, 0, 0 };


// Timer for controller update
unsigned long prevTimeBTUpdate = 0;

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