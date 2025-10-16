#include "main.h"

ControllerButtons control;

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


// Print all Gamepad data to Serial
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

// Handle BT Gamepad
void processGamepad(ControllerPtr ctl) {

  // Connection Good
  //ctl->setColorLED(0, 255, 0);

  control.LX = ctl->axisX();
  control.LY = ctl->axisY();
  control.RX = ctl->axisRX();
  control.RY = ctl->axisRY();

  control.cross = ctl->a();
  control.circle = ctl->b();
  control.triangle = ctl->y();
  control.square = ctl->x();


  control.dpad_up = ctl->dpad() & 0x01;
  control.dpad_down = ctl->dpad() & 0x02;
  control.dpad_right = ctl->dpad() & 0x04;
  control.dpad_left = ctl->dpad() & 0x08;


  // Another way to query controller data is by getting the buttons() function.
  // See how the different "dump*" functions dump the Controller info.
  // dumpGamepad(ctl);
}


// Determine which controller proccessor to use
void processControllers() {

  // While trying to process the controller, if it doesn't get the proper data, makes this false
  control.connected;

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
 * y - Forward/Reverse
 * x - Left/Right
 * rot - Rotation
 * Stores data in desiredPowers[] array
 */
void calculateMech(int y, int x, int rot) {

  // Applies deadzones to inputs
  if (y > -contDeadzone && y < contDeadzone) {
    y = 0;
  }
  if (x > -contDeadzone && x < contDeadzone) {
    x = 0;
  }
  if (rot > -contDeadzone && rot < contDeadzone) {
    rot = 0;
  }

  // Mechanum calculation matrix
  desiredPowers[0] = (x + y + rot) * 8;
  desiredPowers[1] = (x + y + rot) * 8;
  desiredPowers[2] = (x + y + rot) * 8;
  desiredPowers[3] = (x + y + rot) * 8;
}


void drive(int powFL, int powBL, int powBR, int powFR) {

  // FLMotor
  motorDriver.SpeedM1(driver1Addr, powFL);

  //BLMotor
  motorDriver.SpeedM2(driver1Addr, powBL);

  // BRMotor
  motorDriver.SpeedM1(driver2Addr, powBR);

  // FRMotor
  motorDriver.SpeedM2(driver2Addr, powFR);
}


// Arduino setup function. Runs in CPU 1
void setup() {

  Serial.begin(115200);
  motorDriver.begin(38400);  // Initialize the motor drivers

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  /*
  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  //BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  */


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


// Arduino loop function. Runs in CPU 1.
void loop() {

  // Updates the controller once a timer is done.
  if (millis() - prevTime >= 15) {
    BP32.update();
    processControllers();
    prevTime = millis();
  }

  /*
  if (!safetyLoop()) {
    for (int i = 0; i < 4; i++) {
      motorPowers[i] = 0;
    }
  }
  */

  // Calculate mechanum wheel powers from xy controlls
  //calculateMech(control.LY, control.LX, control.RX);

  // Send calculated powers to the motor
  //drive(desiredPowers[0], desiredPowers[1], desiredPowers[2], desiredPowers[3]);


  if (control.LX < 16 || control.LX > -16) {
    motorDriver.SpeedM1(driver2Addr, control.LX * 7);
  } else {
    motorDriver.SpeedM1(driver2Addr, control.LX * 7);
  }


  delay(25);
}