#include <KerbalSimpit.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SerLCD.h>
#include <PayloadStructs.h>


// Shift register pin configuration
const int LATCH_PIN = 15;  // Pin connected to ST_CP (Latch Pin) of 74HC595
const int CLOCK_PIN = 14; // Pin connected to SH_CP (Clock Pin) of 74HC595
const int DATA_PIN = 16;  // Pin connected to DS (Data Pin) of 74HC595

// Variables to store LED states
byte ledStates1 = 0x00;  // LEDs 1-8 (first shift register)
byte ledStates2 = 0x00;  // LEDs 9-16 (second shift register)

// Define each LED with its corresponding bit position in the shift registers
const int LED_SAS = 0;             // First shift register
const int LED_AUTO_PILOT = 1;
const int LED_ANTI_NORMAL = 5;
const int LED_NORMAL = 3;
const int LED_RETRO_GRADE = 4;
const int LED_PRO_GRADE = 6;
const int LED_MANAUVER = 2;
const int LED_STABILITY_ASSIST = 7;

// Second shift register
const int LED_RADIAL_OUT = 8;      // Second shift register
const int LED_RADIAL_IN = 9;
const int LED_TARGET = 10;
const int LED_ANTI_TARGET = 11;
const int LED_NAVIGATION = 12;
const int LED_RCS = 13;
const int LED_GEARS = 14;
const int LED_BRAKES = 15;

// Constants
const int THROTTLE_PIN = A0;       // the pin used for controlling throttle
const int PITCH_PIN = A1;          // the pin used for controlling pitch
const int ROLL_PIN = A2;           // the pin used for controlling roll
const int YAW_PIN = A3;            // the pin used for controlling yaw
const int TRANSLATE_X_PIN = A5;    // the pin used for controlling translation X
const int TRANSLATE_Y_PIN = A6;    // the pin used for controlling translation Y
const int TRANSLATE_Z_PIN = A4;    // the pin used for controlling translation Z
const int POT_PIN = A7;        // Potentiometer for changing SAS mode
const int BRAKE_SWITCH = 6;
const int GEAR_SWITCH = 7;
const int RCS_SWITCH = 8;
const int SAS_SWITCH = 9;
const int RADS_SWITCH = 10;
const int SOLAR_SWITCH = 11;
const int LIGHTS_SWITCH = 12;
const int LIGHTS_SWITCH_LED = 13;
const int JOYSTICK_BUTTON_TRANSLATION = 5;
const int JOYSTICK_BUTTON_ROTATION = 4;
const int LCD_SWITCH_PIN_LEFT = 2;
const int LCD_SWITCH_PIN_RIGHT = 3;
const unsigned long DEBOUNCE_DELAY = 50; // Debounce delay in milliseconds
const unsigned int LCD_UPDATE_INTERVAL = 125;  // LCD update frequency
const unsigned int SEND_INTERVAL = 1500;
const int DEADZONE = 20; // Deadzone for joystick inputs
const int DEADZONE_CAMERA_COMMANDS = 50;  // Deadzone for joystick Camera inputs
const int SMALL_INCREMENT = 300; // Adjust this camera responsivenes value as needed

// Define key codes for Camera control in KSP
const int LEFT_KEY = 0x25;   // Left arrow key
const int RIGHT_KEY = 0x27;  // Right arrow key
const int PITCH_UP_KEY = 0x26;   // Up arrow key (Zoom In)
const int PITCH_DOWN_KEY = 0x28; // Down arrow key (Zoom Out)
const int ZOOM_IN_KEY = 0x21;   // Page Up key (Zoom In)
const int ZOOM_OUT_KEY = 0x22;  // Page Down key (Zoom Out)

// Track whether keys are currently pressed
bool left_pressed = false, right_pressed = false;
bool zoom_in_pressed = false, zoom_out_pressed = false;
bool pitch_up_pressed = false, pitch_down_pressed = false;

    
// Enum for button states
enum ButtonState {
  BUTTON_HIGH,
  BUTTON_LOW
};




bool echoReceived = false;


// Global Variables
KerbalSimpit mySimpit(Serial);
SerLCD lcd;
bool isConnected = false;


unsigned long lastLCDUpdate = 0;

// Variables to store the last debounce time
unsigned long lastDebounceTimeRight = 0;
unsigned long lastDebounceTimeLeft = 0;
unsigned long lastDebounceTimeJoystickTranslation = 0;
unsigned long lastDebounceTimeJoystickRotation = 0;
unsigned long lastDebounceTimeBrakeSwitch = 0;
unsigned long lastDebounceTimeGearSwitch = 0;
unsigned long lastDebounceTimeRCSSwitch = 0;
unsigned long lastDebounceTimeSASSwitch = 0;
unsigned long lastDebounceTimeRADSSwitch = 0;
unsigned long lastDebounceTimeSolarSwitch = 0;
unsigned long lastDebounceTimeLightsSwitch = 0;

// Variables to store the current and previous readings
int lastSASSwitchState = HIGH;  // Assume switch is not pressed initially
int lastLightsSwitchState = HIGH;  // Assume switch is not pressed initially
int lastGearSwitchState = HIGH;
int lastBrakeSwitchState = HIGH;
int lastRCSSwitchState = HIGH;
int lastRADSSwitchState = HIGH;
int lastSolarSwitchState = HIGH;

int lcdScreenCase = 0;
int lcdScreenCaseBeforeAlarm = 0;
bool lcdAlarmState = false;
bool lcdAlarmStateOverride = false;

bool translationButtonPressed = false;

// Global variable declarations

airspeedMessage myAirspeed;
deltaVMessage myDeltaV;
altitudeMessage myAltitude;
velocityMessage myVelocity;
vesselPointingMessage myRotation;
tempLimitMessage myTemplimits;
atmoConditionsMessage myAtmoConditions;
resourceMessage myElectric;

// Custom LCD symbols
byte deltaChar[8] = {
  0b00000,
  0b00100,
  0b01010,
  0b10001,
  0b10001,
  0b11111,
  0b00000,
  0b00000
};

// Function Prototypes
void connectToKSP();
void handleJoystickButtons(unsigned long now);
void handleSwitches(unsigned long now);
void handleLCDButtons(unsigned long now);
void handleTempAlarm();
void updateLCD();
void sendThrottleCommands();
void sendCameraCommands();
void sendTranslationCommands();
void sendRotationCommands();
void messageHandler(byte messageType, byte msg[], byte msgSize);


// Setup function
void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud
  Wire.begin();
  
  // Initialize LCD
  lcd.begin(Wire);
  lcd.setFastBacklight(255, 255, 255);
  lcd.createChar(0, deltaChar); // Create the custom character
  Wire.setClock(400000); // Optional - set I2C SCL to High Speed Mode of 400kHz
  
  // Set pin modes
  pinMode(LCD_SWITCH_PIN_RIGHT, INPUT_PULLUP);
  pinMode(LCD_SWITCH_PIN_LEFT, INPUT_PULLUP);
  pinMode(JOYSTICK_BUTTON_TRANSLATION, INPUT_PULLUP);
  pinMode(JOYSTICK_BUTTON_ROTATION, INPUT_PULLUP);
  pinMode(BRAKE_SWITCH, INPUT_PULLUP);
  pinMode(GEAR_SWITCH, INPUT_PULLUP);
  pinMode(RCS_SWITCH, INPUT_PULLUP);
  pinMode(SAS_SWITCH, INPUT_PULLUP);
  pinMode(RADS_SWITCH, INPUT_PULLUP);
  pinMode(SOLAR_SWITCH, INPUT_PULLUP);
  pinMode(LIGHTS_SWITCH, INPUT_PULLUP);
  pinMode(LIGHTS_SWITCH_LED, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

    // Read initial state of switches
  lastSASSwitchState = digitalRead(SAS_SWITCH);
  lastLightsSwitchState = digitalRead(LIGHTS_SWITCH);
  lastGearSwitchState = digitalRead(GEAR_SWITCH);
  lastBrakeSwitchState = digitalRead(BRAKE_SWITCH);
  lastRCSSwitchState = digitalRead(RCS_SWITCH);
  lastRADSSwitchState = digitalRead(RADS_SWITCH);
  lastSolarSwitchState = digitalRead(SOLAR_SWITCH);
  
  lcd.clear();
  lcd.print("KSP CONTROLLER!");
  lcd.setCursor(0, 1);
  lcd.print("Ready to connect");

  connectToKSP();
}

// Main loop function
void loop() {
  unsigned long now = millis();
  handleSwitches(now);
  handleLCDButtons(now);
  handleJoystickButtons(now);
  handleTempAlarm();
  SAS_mode_pot();

  if (now - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    updateLCD();
    lastLCDUpdate = now; // Update the last LCD update time
  }
  
  // Reconnect if disconnected
  if (!isConnected) {
    connectToKSP();
  }
  
  // Send at each loop a message to control the throttle and the pitch/roll axis.
  sendRotationCommands();
  sendThrottleCommands();

  // Send either translation or camera commands based on the button state
  if (translationButtonPressed) {
    sendCameraCommands();
  } else {
    sendTranslationCommands();
  }

  mySimpit.update();

}

// Function to connect to Kerbal Space Program
void connectToKSP() {
  while (!mySimpit.init()) {
    delay(100);
  }
  isConnected = true;
  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  lcd.clear();
  lcd.print("CONNECTED!");

  mySimpit.inboundHandler(messageHandler);
  mySimpit.registerChannel(AIRSPEED_MESSAGE);
  mySimpit.registerChannel(ALTITUDE_MESSAGE);
  mySimpit.registerChannel(VELOCITY_MESSAGE);
  mySimpit.registerChannel(ROTATION_DATA_MESSAGE);
  mySimpit.registerChannel(TEMP_LIMIT_MESSAGE);
  mySimpit.registerChannel(DELTAV_MESSAGE);
  mySimpit.registerChannel(ATMO_CONDITIONS_MESSAGE);
  mySimpit.registerChannel(ELECTRIC_MESSAGE);
  mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);
  mySimpit.registerChannel(SAS_MODE_INFO_MESSAGE);
  
}



void handleSwitches(unsigned long now) {
  // Read switch states
  int readingBrakeSwitch = digitalRead(BRAKE_SWITCH);
  int readingGearSwitch = digitalRead(GEAR_SWITCH);
  int readingRCSSwitch = digitalRead(RCS_SWITCH);
  int readingSASSwitch = digitalRead(SAS_SWITCH);
  int readingRADSSwitch = digitalRead(RADS_SWITCH);
  int readingSolarSwitch = digitalRead(SOLAR_SWITCH);
  int readingLightsSwitch = digitalRead(LIGHTS_SWITCH);

  // Handle SOLAR Switch
  if (readingSolarSwitch != lastSolarSwitchState && (now - lastDebounceTimeSolarSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeSolarSwitch = now;

    if (readingSolarSwitch == LOW) {  // Button pressed
      mySimpit.deactivateCAG(9);
      //mySimpit.toggleCAG(9);
    } else {  // Button released
     
      //mySimpit.toggleCAG(9);
      mySimpit.activateCAG(9);
    }

    // Update the last state
    lastSolarSwitchState = readingSolarSwitch;
  }



  // Handle RADS Switch
  if (readingRADSSwitch != lastRADSSwitchState && (now - lastDebounceTimeRADSSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeRADSSwitch = now;

    if (readingRADSSwitch == LOW) {  // Button pressed
      
      mySimpit.deactivateCAG(10);
      

    } else {  // Button released
     mySimpit.activateCAG(10);

      
    }

    // Update the last state
    lastRADSSwitchState = readingRADSSwitch;
  }

  // Handle RCS Switch
  if (readingRCSSwitch != lastRCSSwitchState && (now - lastDebounceTimeRCSSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeRCSSwitch = now;

    if (readingRCSSwitch == LOW) {  // Button pressed
      
      mySimpit.deactivateAction(RCS_ACTION);
      setLED(LED_RCS, false);

    } else {  // Button released
     
      mySimpit.activateAction(RCS_ACTION);
      setLED(LED_RCS, true);
    }

    // Update the last state
    lastRCSSwitchState = readingRCSSwitch;
  }

  // Handle Brake Switch
  if (readingBrakeSwitch != lastBrakeSwitchState && (now - lastDebounceTimeBrakeSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeBrakeSwitch = now;

    if (readingBrakeSwitch == LOW) {  // Button pressed
      
      mySimpit.deactivateAction(BRAKES_ACTION);
      setLED(LED_BRAKES, false);

    } else {  // Button released
     
      mySimpit.activateAction(BRAKES_ACTION);
      setLED(LED_BRAKES, true);
    }

    // Update the last state
    lastBrakeSwitchState = readingBrakeSwitch;
  }


     // Handle Gear Switch
  if (readingGearSwitch != lastGearSwitchState && (now - lastDebounceTimeGearSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeGearSwitch = now;

    if (readingGearSwitch == LOW) {  // Button pressed
      
      mySimpit.deactivateAction(GEAR_ACTION);
      setLED(LED_GEARS, false);

    } else {  // Button released
     
      mySimpit.activateAction(GEAR_ACTION);
      setLED(LED_GEARS, true);
    }

    // Update the last state
    lastGearSwitchState = readingGearSwitch;
  }

   // Handle SAS Switch
  if (readingSASSwitch != lastSASSwitchState && (now - lastDebounceTimeSASSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeSASSwitch = now;


    if (readingSASSwitch == LOW) {  // Button pressed
      
    mySimpit.deactivateAction(SAS_ACTION);
    mySimpit.printToKSP("SAS Deactivated", PRINT_TO_SCREEN);
    setLED(LED_SAS, false);

    } else {  // Button released
    mySimpit.activateAction(SAS_ACTION);
    mySimpit.printToKSP("SAS Activated", PRINT_TO_SCREEN);
    setLED(LED_SAS, true);

 
    }

    // Update the last state
    lastSASSwitchState = readingSASSwitch;
  }

  // Handle the Lights Switch with LED
  if (readingLightsSwitch != lastLightsSwitchState && (now - lastDebounceTimeLightsSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeLightsSwitch = now;

    if (readingLightsSwitch == LOW) {  // Button pressed
      
      mySimpit.deactivateAction(LIGHT_ACTION); 
    } else {  // Button released
      
      mySimpit.activateAction(LIGHT_ACTION);
    }

    // Update the last state
    lastLightsSwitchState = readingLightsSwitch;
  }
}

// Function to handle joystick buttons with debouncing
void handleJoystickButtons(unsigned long now) {
  int readingJoystickButtonTranslation = digitalRead(JOYSTICK_BUTTON_TRANSLATION);
  int readingJoystickButtonRotation = digitalRead(JOYSTICK_BUTTON_ROTATION);

    // Handle the translation button (inverted logic for pull-up)
      if (readingJoystickButtonTranslation == LOW && (now - lastDebounceTimeJoystickTranslation) > DEBOUNCE_DELAY) {
          // Toggle the translationButtonPressed state
          translationButtonPressed = !translationButtonPressed;
          if (translationButtonPressed == true) {
              mySimpit.printToKSP(F("Camera mode"), PRINT_TO_SCREEN);
          } else {
              mySimpit.printToKSP(F("translation mode"), PRINT_TO_SCREEN);
          }
          lastDebounceTimeJoystickTranslation = now;
    }


      // Handle the rotation button (inverted logic for pull-up)
      if (readingJoystickButtonRotation == LOW && (now - lastDebounceTimeJoystickRotation) > DEBOUNCE_DELAY) {
        mySimpit.activateAction(STAGE_ACTION);
        lastDebounceTimeJoystickRotation = now;
      }
}

// Function to handle LCD buttons with debouncing
void handleLCDButtons(unsigned long now) {
  int readingLCDSwitchPinRight = digitalRead(LCD_SWITCH_PIN_RIGHT);
  int readingLCDSwitchPinLeft = digitalRead(LCD_SWITCH_PIN_LEFT);

  // Handle the right button
  if (readingLCDSwitchPinRight == LOW && (now - lastDebounceTimeRight) > DEBOUNCE_DELAY) {
    if (lcdScreenCase < 8) {
      lcdScreenCase++;
      lcdScreenCaseBeforeAlarm = lcdScreenCase;
      lcd.clear(); // clear screen for new display info
      if (lcdAlarmState) {
        lcdAlarmStateOverride = false;
        lcdAlarmState = false;
        lcdScreenCase = 0;
        lcd.clear();
        lcd.setBacklight(255, 255, 255);
      }
    }
    lastDebounceTimeRight = now;
  }

  // Handle the left button
  if (readingLCDSwitchPinLeft == LOW && (now - lastDebounceTimeLeft) > DEBOUNCE_DELAY) {
    if (lcdScreenCase > 0) {
      lcdScreenCase--;
      lcdScreenCaseBeforeAlarm = lcdScreenCase;
      lcd.clear(); // clear screen for new display info
      if (lcdAlarmState) {
        lcdAlarmStateOverride = true;
        lcdAlarmState = false;
        lcdScreenCase = 0;
        lcd.clear();
        lcd.setBacklight(255, 255, 255);
      }
    }
    lastDebounceTimeLeft = now;
  }
}

// Function to handle temperature alarms
void handleTempAlarm() {
  if (myTemplimits.skinTempLimitPercentage > 40 || myTemplimits.tempLimitPercentage > 40) {
    if (!lcdAlarmState && !lcdAlarmStateOverride) {
      lcdScreenCaseBeforeAlarm = lcdScreenCase; // Save the current state before alarm
      lcdScreenCase = 98;
      lcdAlarmState = true;
      lcd.setBacklight(255, 0, 0);
      lcd.clear();
    }
  } else {
    if (lcdAlarmState) {
      lcdAlarmState = false;
      lcdAlarmStateOverride = false;
      lcdScreenCase = lcdScreenCaseBeforeAlarm;
      lcd.setBacklight(255, 255, 255);
      lcd.clear();
    }
  }
}

// Function to update LCD display
void updateLCD() {
  switch (lcdScreenCase) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("MACH: ");
      lcd.print(myAirspeed.mach);
      lcd.setCursor(0, 1);
      lcd.print("Airspeed: ");
      lcd.print(round(myAirspeed.IAS));
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("Sealevel: ");
      lcd.print(round(myAltitude.sealevel));
      lcd.setCursor(0, 1);
      lcd.print("Surface: ");
      lcd.print(round(myAltitude.surface));
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("m/s: ");
      lcd.print(round(myVelocity.surface));
      lcd.setCursor(0, 1);
      lcd.print("km/h: ");
      lcd.print(round((myVelocity.surface * 3.6)));
      break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("Heading: ");
      lcd.print(myRotation.heading);
      lcd.setCursor(0, 1);
      lcd.print("Pitch: ");
      lcd.print(myRotation.pitch);
      break;
    case 4:
      lcd.setCursor(0, 0);
      lcd.print("Part Temp %");
      lcd.print(myTemplimits.tempLimitPercentage);
      lcd.setCursor(0, 1);
      lcd.print("Skin Temp %");
      lcd.print(myTemplimits.skinTempLimitPercentage);
      break;
    case 5:
      lcd.setCursor(0, 0);
      lcd.write(byte(0)); // Write the custom Delta character
      lcd.print("V Stage ");
      lcd.print(round(myDeltaV.stageDeltaV));
      lcd.setCursor(0, 1);
      lcd.write(byte(0)); // Write the custom Delta character
      lcd.print("V Ship ");
      lcd.print(round(myDeltaV.totalDeltaV));
      break;
    case 6:
      lcd.setCursor(0, 0);
      lcd.print("Air Temp ");
      lcd.print(myAtmoConditions.temperature - 273.15);
      lcd.setCursor(0, 1);
      lcd.print("Air Density ");
      lcd.print(myAtmoConditions.airDensity);
      break;
    case 7:
      lcd.setCursor(0, 0);
      lcd.print("Air Pres ");
      lcd.print(myAtmoConditions.pressure);
      lcd.setCursor(0, 1);
      lcd.print("G-Forces  ");
      lcd.print(myAirspeed.gForces);
      break;
    case 8:
      lcd.setCursor(0, 0);
      lcd.print("Power ");
      lcd.print(myElectric.available);
      break;
    case 98:
      lcd.setCursor(0, 0);
      lcd.print("PART TEMP!: ");
      lcd.print(myTemplimits.tempLimitPercentage);
      lcd.setCursor(0, 1);
      lcd.print("SKIN TEMP!: ");
      lcd.print(myTemplimits.skinTempLimitPercentage);
      break;
  }
}

// Function to send throttle commands
void sendThrottleCommands() {
  throttleMessage throttleMsg;
  int reading = analogRead(THROTTLE_PIN);
  throttleMsg.throttle = map(reading, 0, 1023, INT16_MAX, 0);
  mySimpit.send(THROTTLE_MESSAGE, throttleMsg);
}

// Function to send camera commands
void sendCameraCommands() {
  // Read joystick inputs
  int readingYaw = analogRead(TRANSLATE_X_PIN);
  int readingPitch = analogRead(TRANSLATE_Z_PIN);
  int readingZoom = analogRead(TRANSLATE_Y_PIN);

  // Handle yaw (Left)
  if (readingYaw > (512 + DEADZONE_CAMERA_COMMANDS) && !left_pressed) {
    keyboardEmulatorMessage yawMsg(LEFT_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, yawMsg);
    left_pressed = true;
  } else if (readingYaw <= (512 + DEADZONE_CAMERA_COMMANDS) && left_pressed) {
    keyboardEmulatorMessage yawMsg(LEFT_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, yawMsg);
    left_pressed = false;
  }

  // Handle yaw (Right)
  if (readingYaw < (512 - DEADZONE_CAMERA_COMMANDS) && !right_pressed) {
    keyboardEmulatorMessage yawMsg(RIGHT_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, yawMsg);
    right_pressed = true;
  } else if (readingYaw >= (512 - DEADZONE_CAMERA_COMMANDS) && right_pressed) {
    keyboardEmulatorMessage yawMsg(RIGHT_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, yawMsg);
    right_pressed = false;
  }

  // Handle pitch (Up)
  if (readingPitch > (512 + DEADZONE_CAMERA_COMMANDS) && !pitch_up_pressed) {
    keyboardEmulatorMessage pitchMsg(PITCH_UP_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, pitchMsg);
    pitch_up_pressed = true;
  } else if (readingPitch <= (512 + DEADZONE_CAMERA_COMMANDS) && pitch_up_pressed) {
    keyboardEmulatorMessage pitchMsg(PITCH_UP_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, pitchMsg);
    pitch_up_pressed = false;
  }

  // Handle pitch (Down)
  if (readingPitch < (512 - DEADZONE_CAMERA_COMMANDS) && !pitch_down_pressed) {
    keyboardEmulatorMessage pitchMsg(PITCH_DOWN_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, pitchMsg);
    pitch_down_pressed = true;
  } else if (readingPitch >= (512 - DEADZONE_CAMERA_COMMANDS) && pitch_down_pressed) {
    keyboardEmulatorMessage pitchMsg(PITCH_DOWN_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, pitchMsg);
    pitch_down_pressed = false;
  }

  // Handle zoom (In)
  if (readingZoom > (512 + DEADZONE_CAMERA_COMMANDS) && !zoom_in_pressed) {
    keyboardEmulatorMessage zoomMsg(ZOOM_IN_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, zoomMsg);
    zoom_in_pressed = true;
  } else if (readingZoom <= (512 + DEADZONE_CAMERA_COMMANDS) && zoom_in_pressed) {
    keyboardEmulatorMessage zoomMsg(ZOOM_IN_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, zoomMsg);
    zoom_in_pressed = false;
  }

  // Handle zoom (Out)
  if (readingZoom < (512 - DEADZONE_CAMERA_COMMANDS) && !zoom_out_pressed) {
    keyboardEmulatorMessage zoomMsg(ZOOM_OUT_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, zoomMsg);
    zoom_out_pressed = true;
  } else if (readingZoom >= (512 - DEADZONE_CAMERA_COMMANDS) && zoom_out_pressed) {
    keyboardEmulatorMessage zoomMsg(ZOOM_OUT_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, zoomMsg);
    zoom_out_pressed = false;
  }
}

// Function to send translation commands
void sendTranslationCommands() {
  translationMessage transMsg;
  int readingX = analogRead(TRANSLATE_X_PIN);
  int readingY = analogRead(TRANSLATE_Y_PIN);
  int readingZ = analogRead(TRANSLATE_Z_PIN);

  int16_t translateX = 0;
  if (readingX > (512 + DEADZONE)) {
    translateX = map(readingX, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (readingX < (512 - DEADZONE)) {
    translateX = map(readingX, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  int16_t translateY = 0;
  if (readingY > (512 + DEADZONE)) {
    translateY = map(readingY, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (readingY < (512 - DEADZONE)) {
    translateY = map(readingY, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  int16_t translateZ = 0;
  if (readingZ > (512 + DEADZONE)) {
    translateZ = map(readingZ, 512 + DEADZONE, 1023, 0, INT16_MIN);
  } else if (readingZ < (512 - DEADZONE)) {
    translateZ = map(readingZ, 0, 512 - DEADZONE, INT16_MAX, 0);
  }

  transMsg.setX(translateX);
  transMsg.setY(translateY);
  transMsg.setZ(translateZ);
  mySimpit.send(TRANSLATION_MESSAGE, transMsg);
}

// Function to send rotation commands
void sendRotationCommands() {
  rotationMessage rotMsg;
  int readingPitch = analogRead(PITCH_PIN);
  int readingRoll = analogRead(ROLL_PIN);
  int readingYaw = analogRead(YAW_PIN);

  int16_t pitch = 0;
  if (readingPitch > (512 + DEADZONE)) {
    pitch = map(readingPitch, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (readingPitch < (512 - DEADZONE)) {
    pitch = map(readingPitch, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  int16_t roll = 0;
  if (readingRoll > (512 + DEADZONE)) {
    roll = map(readingRoll, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (readingRoll < (512 - DEADZONE)) {
    roll = map(readingRoll, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  int16_t yaw = 0;
  if (readingYaw > (512 + DEADZONE)) {
    yaw = map(readingYaw, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (readingYaw < (512 - DEADZONE)) {
    yaw = map(readingYaw, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  rotMsg.setPitch(pitch);
  rotMsg.setRoll(roll);
  rotMsg.setYaw(yaw);
  mySimpit.send(ROTATION_MESSAGE, rotMsg);
}

// Message handler for Kerbal Simpit
void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch (messageType) {
    case ATMO_CONDITIONS_MESSAGE:
      if (msgSize == sizeof(atmoConditionsMessage)) {
        myAtmoConditions = parseMessage<atmoConditionsMessage>(msg);
      }
      break;
    case AIRSPEED_MESSAGE:
      if (msgSize == sizeof(airspeedMessage)) {
        myAirspeed = parseMessage<airspeedMessage>(msg);
      }
      break;
    case ALTITUDE_MESSAGE:
      if (msgSize == sizeof(altitudeMessage)) {
        myAltitude = parseMessage<altitudeMessage>(msg);
      }
      break;
    case DELTAV_MESSAGE:
      if (msgSize == sizeof(deltaVMessage)) {
        myDeltaV = parseMessage<deltaVMessage>(msg);
      }
      break;
    case VELOCITY_MESSAGE:
      if (msgSize == sizeof(velocityMessage)) {
        myVelocity = parseMessage<velocityMessage>(msg);
      }
      break;
    case ROTATION_DATA_MESSAGE:
      if (msgSize == sizeof(vesselPointingMessage)) {
        myRotation = parseMessage<vesselPointingMessage>(msg);
      }
      break;
    case ELECTRIC_MESSAGE:
      if (msgSize == sizeof(resourceMessage)) {
        myElectric = parseMessage<resourceMessage>(msg);
      }
      break;
    case TEMP_LIMIT_MESSAGE:
      if (msgSize == sizeof(tempLimitMessage)) {
        myTemplimits = parseMessage<tempLimitMessage>(msg);
      }
      break;

    case ACTIONSTATUS_MESSAGE:
        if (msgSize == 1) {
        bool game_SAS_State = msg[0] & SAS_ACTION;
        
      }
      break;


  }
  
}

void SAS_mode_pot() {
  // Read the potentiometer value (0 to 1023)
  int potValue = analogRead(POT_PIN);

  // Clear only the LEDs used in this function
  clearSASModeLEDs();

  // Define custom potentiometer ranges for each LED and turn them on accordingly
  if (potValue >= 0 && potValue < 10) {
    setLED(LED_AUTO_PILOT, true);
  } else if (potValue >= 10 && potValue < 95) {
    setLED(LED_ANTI_NORMAL, true);
  } else if (potValue >= 95 && potValue < 195) {
    setLED(LED_NORMAL, true);
  } else if (potValue >= 195 && potValue < 310) {
    setLED(LED_RETRO_GRADE, true);
  } else if (potValue >= 310 && potValue < 420) {
    setLED(LED_PRO_GRADE, true);
  } else if (potValue >= 420 && potValue < 507) {
    setLED(LED_MANAUVER, true);
  } else if (potValue >= 507 && potValue < 600) {
    setLED(LED_STABILITY_ASSIST, true);
  } else if (potValue >= 600 && potValue < 690) {
    setLED(LED_RADIAL_OUT, true);
  } else if (potValue >= 690 && potValue < 790) {
    setLED(LED_RADIAL_IN, true);
  } else if (potValue >= 790 && potValue < 875) {
    setLED(LED_TARGET, true);
  } else if (potValue >= 875 && potValue < 975) {
    setLED(LED_ANTI_TARGET, true);
  } else if (potValue >= 975 && potValue < 1024) {
    setLED(LED_NAVIGATION, true);
  }

  // Update shift registers to reflect the changes
  updateShiftRegisters();
}

// Function to clear only the LEDs used in SAS_mode_pot()
void clearSASModeLEDs() {
  setLED(LED_AUTO_PILOT, false);
  setLED(LED_ANTI_NORMAL, false);
  setLED(LED_NORMAL, false);
  setLED(LED_RETRO_GRADE, false);
  setLED(LED_PRO_GRADE, false);
  setLED(LED_MANAUVER, false);
  setLED(LED_STABILITY_ASSIST, false);
  setLED(LED_RADIAL_OUT, false);
  setLED(LED_RADIAL_IN, false);
  setLED(LED_TARGET, false);
  setLED(LED_ANTI_TARGET, false);
  setLED(LED_NAVIGATION, false);
}

// Function to set the state of a specific LED (on or off)
void setLED(int led, bool state) {
  if (led < 8) {  // First shift register (LEDs 0-7)
    if (state) {
      ledStates1 |= (1 << led);   // Set bit to 1 (turn on)
    } else {
      ledStates1 &= ~(1 << led);  // Set bit to 0 (turn off)
    }
  } else {        // Second shift register (LEDs 8-15)
    int shiftRegisterLED = led - 8;
    if (state) {
      ledStates2 |= (1 << shiftRegisterLED);  // Set bit to 1 (turn on)
    } else {
      ledStates2 &= ~(1 << shiftRegisterLED); // Set bit to 0 (turn off)
    }
  }
}

// Function to update the shift registers with current LED states
void updateShiftRegisters() {
  digitalWrite(LATCH_PIN, LOW);         // Prepare to send data
  shiftOut(DATA_PIN, CLOCK_PIN, ledStates2);  // Send data for SR2 (LEDs 9-16)
  shiftOut(DATA_PIN, CLOCK_PIN, ledStates1);  // Send data for SR1 (LEDs 1-8)
  digitalWrite(LATCH_PIN, HIGH);        // Latch the data (output to LEDs)
}

// Function to shift out data to the shift registers (74HC595)
void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(myClockPin, LOW);
    digitalWrite(myDataPin, (myDataOut & (1 << i)) ? HIGH : LOW);
    digitalWrite(myClockPin, HIGH);
  }
  digitalWrite(myClockPin, LOW);
}
