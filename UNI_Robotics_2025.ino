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
  ctl->setColorLED(controller.colorBar[0], controller.colorBar[1], controller.colorBar[2]);

  controller.LX = ctl->axisX();
  controller.LY = -ctl->axisY();
  controller.RX = ctl->axisRX();
  controller.RY = -ctl->axisRY();

  controller.L2 = ctl->l2();
  controller.R2 = ctl->r2();

  controller.dpad_up = ctl->dpad() & 0x01;
  controller.dpad_down = ctl->dpad() & 0x02;
  controller.dpad_right = ctl->dpad() & 0x04;
  controller.dpad_left = ctl->dpad() & 0x08;

  controller.cross = ctl->a();
  controller.circle = ctl->b();
  controller.triangle = ctl->y();
  controller.square = ctl->x();

  controller.L1 = ctl->l1();
  controller.R1 = ctl->r1();


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
  controller.connected = false;

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
      controller.connected = true;
    }
  }
}


/**
  * Safety loop to check if stuff is still connected
  * Shuts stuff off if not
  */
bool safetyLoop() {

  // If the controller is connected
  if (!controller.connected) {
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
  if (y > -controller.deadzone && y < controller.deadzone) y = 0;
  if (x > -controller.deadzone && x < controller.deadzone) x = 0;
  if (rot > -controller.deadzone && rot < controller.deadzone) rot = 0;

  int speedMulti = 1;
  if (mediumSpeed) {
    speedMulti = 2;
  } else if (fastSpeed) {
    speedMulti = 4;
  }

  // Mechanum calculation matrix
  robot.motor.FL_Motor = (y + x + rot) * speedMulti;
  robot.motor.FR_Motor = (y - x - rot) * speedMulti;
  robot.motor.BL_Motor = (y - x + rot) * speedMulti;
  robot.motor.BR_Motor = (y + x - rot) * speedMulti;
}


/**
  * Moves the robot based on direct motor controller input
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


  complientServo.attach(servo1Pin);
  sortGateServo.attach(servo2Pin);
  clawServo.attach(servo3Pin);

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


  controller.colorBar[0] = 0;
  controller.colorBar[1] = 255;
  controller.colorBar[2] = 0;


  // Adjust drive speed based on controller shoulder buttons
  if (controller.L1 || controller.R1) {

    // Left trigger - Fast speed
    if (controller.L1 && !mediumSpeed) {
      fastSpeed = true;
      controller.colorBar[0] = 255;
      controller.colorBar[1] = 0;
      controller.colorBar[2] = 0;

      // Right Trigger - Medium speed
    } else if (controller.R1 && !fastSpeed) {
      mediumSpeed = true;
      controller.colorBar[0] = 0;
      controller.colorBar[1] = 255;
      controller.colorBar[2] = 0;
    }

    // No trigger pressed - Slow speed
  } else {
    mediumSpeed = false;
    fastSpeed = false;
    controller.colorBar[0] = 0;
    controller.colorBar[1] = 0;
    controller.colorBar[2] = 255;
  }

  // Calculate mechanum wheel powers from xy controlls
  calculateMech(controller.LY, controller.LX, controller.RX);


  /*
  Serial.printf("R1: %d, Slow: %d, Fast: %d || ", controller.R1, slowMode, fastMode);

  Serial.printf("R: %d, G: %d, B: %d", controller.colorBar[0], controller.colorBar[1], controller.colorBar[2]);

  
  Serial.printf("LY: %d, LX: %d, RX: %d || ", controller.LY, controller.LX, controller.RX);

  Serial.printf("FL: %d, FR: %d, BL: %d, BR: %d\n", robot.motor.FL_Motor, robot.motor.FR_Motor, robot.motor.BL_Motor, robot.motor.BR_Motor);
*/


  if (controller.cross) {
    robot.servo.complientPos = 180;  // Eject
  } else if (controller.triangle) {
    robot.servo.complientPos = 0;  // Reverse
  } else {
    robot.servo.complientPos = 90;  // Center
  }

  if (controller.circle) {
    robot.servo.sortGatePos = 50;  // Up - Gate Open
  } else {
    robot.servo.sortGatePos = 100;  // Down - Gate Closed
  }

  if (controller.square) {
    robot.servo.clawPos = 45;  // Open Claw
  } else {
    robot.servo.clawPos = 90;  // Closse Claw / Open Gate
  }


  // Write servo positions
  complientServo.write(robot.servo.complientPos);
  sortGateServo.write(robot.servo.sortGatePos);
  clawServo.write(robot.servo.clawPos);


  int armSpeed = map(-controller.L2 + controller.R2, -1, 1, 75, 105);
  swingArm.write(armSpeed);

  Serial.printf("Swing Speed: %d", armSpeed);

  // Check if everything is still connected
  if (!safetyLoop()) {
    robot.motor.FL_Motor = 0;
    robot.motor.FR_Motor = 0;
    robot.motor.BL_Motor = 0;
    robot.motor.BR_Motor = 0;
  }

  // Send calculated powers to the motor
  drive(
    robot.motor.FL_Motor,
    robot.motor.FR_Motor,
    robot.motor.BL_Motor,
    robot.motor.BR_Motor);

  Serial.printf("\n");
}