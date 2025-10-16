#include <sys/_types.h>
#ifndef main
#define main

#include <Bluepad32.h>
#include <sys/_stdint.h>
#include <RoboClaw.h>

#include "HUSKYLENS/HUSKYLENS.h"  // Camera

#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

#define driver1Addr 0x80
#define driver2Addr 0x81

#define contDeadzone 16


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

HUSKYLENS camera;

RoboClaw motorDriver(&Serial2, 5000);


/**
 * Motor pins
 * 0 - FLMotor
 * 1 - BLMotor
 * 2 - BRMotor
 * 3 - FR Motor
 */
byte motorPins[4] = { 0, 1, 2, 3 };

/**
 * Stores the current output powers of the motors
 * 0 - FLMotor
 * 1 - BLMotor
 * 2 - BRMotor
 * 3 - FRMotor
*/
int8_t motorPowers[4] = { 0, 0, 0, 0 };


// Easy way to reach the controller values
struct ControllerButtons {
  int16_t
    LX = 0,
    LY = 0,
    RX = 0,
    RY = 0;

  bool
    cross = false,
    circle = false,
    triangle = false,
    square = false;

  uint16_t
    dpad_up,
    dpad_down,
    dpad_left,
    dpad_right;

    bool
    connected = false;
};

/**
 * Stores the current target powers of the motors
 *
 * 0 - FLMotor
 * 1 - BLMotor
 * 2 - BRMotor
 * 3 - FRMotor
*/
int8_t desiredPowers[4] = { 0, 0, 0, 0 };

unsigned long prevTime = 0;

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void dumpGamepad(ControllerPtr ctl);
void processGamepad(ControllerPtr ctl);
void processControllers();
bool safetyLoop();

#endif