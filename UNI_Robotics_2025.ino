#include "main.h"


// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}


// When a controller disconnects
void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}


/**
  * Print all Gamepad data to Serial
  */
void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}


/**
  * Map all BT32 controller values to local struct
  * 
  */
void processGamepad(ControllerPtr ctl) {

  // Connection Good
  ctl->setColorLED(control.colorBar[0], control.colorBar[1], control.colorBar[2]);

  control.LX = ctl->axisX();
  control.LY = -ctl->axisY();
  control.RX = ctl->axisRX();
  control.RY = -ctl->axisRY();

  control.L2 = ctl->l2();
  control.R2 = ctl->r2();

  control.dpad_up = ctl->dpad() & 0x01;
  control.dpad_down = ctl->dpad() & 0x02;
  control.dpad_right = ctl->dpad() & 0x04;
  control.dpad_left = ctl->dpad() & 0x08;

  control.cross = ctl->a();
  control.circle = ctl->b();
  control.triangle = ctl->y();
  control.square = ctl->x();

  control.L1 = ctl->l1();
  control.R1 = ctl->r1();


  // Another way to query controller data is by getting the buttons() function.
  // See how the different "dump*" functions dump the Controller info.
  // dumpGamepad(ctl);
}


/**
  * Determine which controller proccessor to use
  * Makes sure the controller is sending proper data
  */
void processControllers() {

  // While trying to process the controller, if it doesn't get the proper data, makes this false
  control.connected = false;

  for (auto myController : myControllers) {

    // Controller variable error
    if (!myController) {

    } else if (!myController->isConnected()) {  // No Controller connected
      Serial.println("ERROR: Controller NOT Connected");

    } else if (!myController->hasData()) {  // No data
      Serial.println("ERROR: Controller has no Data");

    } else if (!myController->isGamepad()) {  // Not a supported gamepad
      Serial.println("ERROR: Controller is NOT a Gamepad");

    } else {  // Regular run
      processGamepad(myController);
      control.connected = true;
    }
  }
}


/**
  * Safety loop to check if stuff is still connected
  * Shuts stuff off if not
  */
bool safetyLoop() {

  // If the controller is connected
  if (!control.connected) {
    Serial.println("ALERT: PS4 Controller disconnected");
    return false;
  }


  /*
  if (camera.request()) {
    Serial.println("ALERT: Camera disconnected");
    return false;
  }
  */
  return true;
}


/**
  * Calculates motor powers based on mechanum wheel matrix
  * y - Forward/Reverse
  * x - Left/Right
  * rot - Rotation
  * Stores data in desiredPowers[] array
  */
void calculateMech(int y, int x, int rot) {

  // Applies deadzones to inputs
  if (y > -control.deadzone && y < control.deadzone) y = 0;
  if (x > -control.deadzone && x < control.deadzone) x = 0;
  if (rot > -control.deadzone && rot < control.deadzone) rot = 0;

  int speedMulti = 3;
  if (slowMode) {
    speedMulti = 1;
  } else if (fastMode) {
    speedMulti = 6;
  }

  // Mechanum calculation matrix
  motors.targetPow.FL_Motor = (y + x + rot) * speedMulti;
  motors.targetPow.FR_Motor = (y - x - rot) * speedMulti;
  motors.targetPow.BL_Motor = (y - x + rot) * speedMulti;
  motors.targetPow.BR_Motor = (y + x - rot) * speedMulti;
}


/**
  * Moves the robot based on direct motor control input
  */
void drive(int powFL, int powFR, int powBL, int powBR) {

  /*
  // Working on this
  // Trying to make it so that if any of the wheels are commanded to go more than max speed, it scales back the other ones to the appropriate speed
  int powMax = max(max(powFL, powFR), max(powBL, powBR));

  if (powFL > maxSpeed || powFR > maxSpeed || powBL > maxSpeed || powBR > maxSpeed) {

  }
  */


  // FLMotor
  motorDriver.SpeedM1(driver1Addr, powFL);

  // FRMotor
  motorDriver.SpeedM2(driver1Addr, powFR);

  // BLMotor
  motorDriver.SpeedM1(driver2Addr, powBL);

  // BRMotor
  motorDriver.SpeedM2(driver2Addr, powBR);
}


/**
  * Arduino setup function. Runs in CPU 1
  */
void setup() {

  Serial.begin(115200);
  motorDriver.begin(38400);  // Initialize the motor drivers

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);


  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);


  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);

  swingArm.attach(swingArmPin);

  /*
  Wire.begin();
  while (!camera.begin(Wire)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  */
}


/**
  * Arduino loop function. Runs in CPU 1.
  */
void loop() {


  // Updates the controller once a timer is done.
  if (BP32.update()) {
    processControllers();
  }


  control.colorBar[0] = 0;
  control.colorBar[1] = 255;
  control.colorBar[2] = 0;


  if (control.L1 || control.R1) {
    if (control.L1 && !slowMode) {
      fastMode = true;
      control.colorBar[0] = 255;
      control.colorBar[1] = 0;
      control.colorBar[2] = 0;

    } else if (control.R1 && !fastMode) {
      slowMode = true;
      control.colorBar[0] = 0;
      control.colorBar[1] = 255;
      control.colorBar[2] = 0;
    }

  } else {
    slowMode = false;
    fastMode = false;
    control.colorBar[0] = 0;
    control.colorBar[1] = 0;
    control.colorBar[2] = 255;
  }

  // Calculate mechanum wheel powers from xy controlls
  calculateMech(control.LY, control.LX, control.RX);


  /*
  Serial.printf("R1: %d, Slow: %d, Fast: %d || ", control.R1, slowMode, fastMode);

  Serial.printf("R: %d, G: %d, B: %d", control.colorBar[0], control.colorBar[1], control.colorBar[2]);

  
  Serial.printf("LY: %d, LX: %d, RX: %d || ", control.LY, control.LX, control.RX);

  Serial.printf("FL: %d, FR: %d, BL: %d, BR: %d\n", motors.targetPow.FL_Motor, motors.targetPow.FR_Motor, motors.targetPow.BL_Motor, motors.targetPow.BR_Motor);
*/


  


  // Check if everything is still connected
  if (!safetyLoop()) {
    motors.targetPow.FL_Motor = 0;
    motors.targetPow.FR_Motor = 0;
    motors.targetPow.BL_Motor = 0;
    motors.targetPow.BR_Motor = 0;
  }

  // Send calculated powers to the motor
  drive(
    motors.targetPow.FL_Motor,
    motors.targetPow.FR_Motor,
    motors.targetPow.BL_Motor,
    motors.targetPow.BR_Motor);

  Serial.printf("\n");
}