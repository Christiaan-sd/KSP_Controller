#include <KerbalSimpit.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SerLCD.h>
#include <PayloadStructs.h>

// Shift register pin configuration
const int LATCH_PIN = 15;  // Pin connected to ST_CP (Latch Pin) of 74HC595
const int CLOCK_PIN = 14;  // Pin connected to SH_CP (Clock Pin) of 74HC595
const int DATA_PIN = 16;   // Pin connected to DS (Data Pin) of 74HC595

// Variables to store LED states
byte ledStates1 = 0x00;  // LEDs 1-8 (first shift register)
byte ledStates2 = 0x00;  // LEDs 9-16 (second shift register)
byte ledStates3 = 0x00;  // LEDs 17-24 (third shift register)
byte ledStates4 = 0x00;  // LEDs 25-32 (fourth shift register)

// Define each LED with its corresponding bit position in the shift registers
// First shift register
const int LED_SAS = 0;
const int LED_AUTO_PILOT = 1;
const int LED_MANAUVER = 2;
const int LED_NORMAL = 3;
const int LED_RETRO_GRADE = 4;
const int LED_ANTI_NORMAL = 5;
const int LED_PRO_GRADE = 6;
const int LED_STABILITY_ASSIST = 7;

// Second shift register
const int LED_RADIAL_OUT = 8;
const int LED_RADIAL_IN = 9;
const int LED_TARGET = 10;
const int LED_ANTI_TARGET = 11;
const int LED_NAVIGATION = 12;
const int LED_RCS = 13;
const int LED_GEARS = 14;
const int LED_BRAKES = 15;

// Third shift register (LEDs 17-24)
const int LED_ROCKET_MODE = 16;
const int LED_ARM_ABORT = 17;
const int LED_ACTION_GROUP = 18;
const int LED_DOCKING_MODE = 19;
const int LED_EVA_MODE = 20;
const int LED_PRECISION = 21;
const int LED_PlANE_MODE = 22;
const int LED_ROVER_MODE = 23;

// Fourth shift register (LEDs 25-32)
const int LED_STAGE = 24;
const int LED_RECOVER = 25;
const int LED_ATMOS = 26;
const int LED_LOW_ELECTRICITY = 27;
const int LED_LOW_FUEL = 28;
const int LED_TEMPRATURE = 29;
const int LED_MASTER_ALARM = 30;
const int LED_OXYGEN = 31;

// LEDs not on shift register
const int LED_SCIENCE = 23;  // Digital pin 13
const int LED_RADS = 17;     // Digital pin 17
const int LED_SOLAR = 18;    // Digital pin 0
const int LED_LIGHTS = 19;   // Digital pin 1

// Constants for control pins
const int THROTTLE_PIN = A0;
const int PITCH_PIN = A1;
const int ROLL_PIN = A2;
const int YAW_PIN = A3;
const int TRANSLATE_Z_PIN = A4;
const int TRANSLATE_X_PIN = A5;
const int TRANSLATE_Y_PIN = A6;
const int POT_SAS_PIN = A7;
const int POT_CONTROL_PIN = A8;

// Switches
const int BRAKE_SWITCH = 6;
const int GEAR_SWITCH = 7;
const int RCS_SWITCH = 8;
const int SAS_SWITCH = 9;
const int RADS_SWITCH = 10;
const int SOLAR_SWITCH = 11;
const int LIGHTS_SWITCH = 12;
const int LIGHTS_SWITCH_LED = 13;
const int STAGE_ARM_SWITCH = 25;
const int ABORT_ARM_SWITCH = 53;

// Buttons
const int LCD_BUTTON_PIN_LEFT = 2;
const int LCD_BUTTON_PIN_RIGHT = 3;
const int JOYSTICK_BUTTON_TRANSLATION = 5;
const int JOYSTICK_BUTTON_ROTATION = 4;
const int SCIENCE_BUTTON_PIN = 24;
const int STAGE_BUTTON_PIN = 27;
const int TRIM_BUTTON_PIN = 29;
const int RESET_TRIM_BUTTON_PIN = 28;
const int LOAD_BUTTON_PIN = 30;
const int SAVE_BUTTON_PIN = 31;
const int SHIPS_BUTTON_PIN = 32;
const int CAMERA_BUTTON_PIN = 33;
const int MAP_BUTTON_PIN = 34;
const int TIMEWARP_MINUS_BUTTON_PIN = 35;
const int TIMEWARP_PAUSE_BUTTON_PIN = 36;
const int TIMEWARP_PERIAPSIS_BUTTON_PIN = 37;
const int TIMEWARP_MANEUVER_BUTTON_PIN = 38;
const int TIMERWARP_APOAPSIS_BUTTON_PIN = 26;
const int TIMEWARP_NORMAL_BUTTON_PIN = 40;
const int TIMEWARP_PLUS_BUTTON_PIN = 41;
const int ACTION_GROUP_10_BUTTON_PIN = 42;
const int ACTION_GROUP_9_BUTTON_PIN = 43;
const int ACTION_GROUP_8_BUTTON_PIN = 44;
const int ACTION_GROUP_7_BUTTON_PIN = 45;
const int ACTION_GROUP_6_BUTTON_PIN = 46;
const int ACTION_GROUP_5_BUTTON_PIN = 47;
const int ACTION_GROUP_4_BUTTON_PIN = 48;
const int ACTION_GROUP_3_BUTTON_PIN = 49;
const int ACTION_GROUP_2_BUTTON_PIN = 50;
const int ACTION_GROUP_1_BUTTON_PIN = 51;
const int ABORT_BUTTON_PIN = 52;

// Reading variables for buttons
int readingTrimButton;
int readingResetTrimButton;
int readingLoadButton;
int readingSaveButton;
int readingShipsButton;
int readingCameraButton;
int readingMapButton;
int readingTimewarpMinusButton;
int readingTimewarpPauseButton;
int readingTimewarpPeriapsisButton;
int readingTimewarpManeuverButton;
int readingTimewarpApoapsisButton;
int readingTimewarpNormalButton;
int readingTimewarpPlusButton;
int readingAbortButton;
int readingActionGroup1Button;
int readingActionGroup2Button;
int readingActionGroup3Button;
int readingActionGroup4Button;
int readingActionGroup5Button;
int readingActionGroup6Button;
int readingActionGroup7Button;
int readingActionGroup8Button;
int readingActionGroup9Button;
int readingActionGroup10Button;
int throttlePercentage;
// Timing constants
const unsigned long DEBOUNCE_DELAY = 50; // Debounce delay in milliseconds
const unsigned int LCD_UPDATE_INTERVAL = 125;  // LCD update frequency
const unsigned int SEND_INTERVAL = 1500;

// Deadzone constants
const int DEADZONE = 100; // Deadzone for joystick inputs
const int DEADZONE_CAMERA_COMMANDS = 50;  // Deadzone for joystick Camera inputs
const int SMALL_INCREMENT = 300; // Adjust this camera responsiveness value as needed

// Key codes for Camera control in KSP
const int LEFT_KEY = 0x25;   // Left arrow key
const int RIGHT_KEY = 0x27;  // Right arrow key
const int PITCH_UP_KEY = 0x26;   // Up arrow key (Zoom In)
const int PITCH_DOWN_KEY = 0x28; // Down arrow key (Zoom Out)
const int ZOOM_IN_KEY = 0x21;    // Page Up key (Zoom In)
const int ZOOM_OUT_KEY = 0x22;   // Page Down key (Zoom Out)
const int LOAD_KEY = 0x78;      // F9 Key for loading
const int SAVE_KEY = 0x74;      // F5 Key for loading
const int SHIPS_KEY = 0xDD;     // ] key for cycling ships
const int MAP_KEY = 0x4D;     // M key for cycling ships
const int CAMERA_KEY = 0x56;     // V key for cycling ships
const int PAUSE_KEY = 0x1B;     // ESC key for pause/menu

// Track whether keys are currently pressed
bool left_pressed = false, right_pressed = false;
bool zoom_in_pressed = false, zoom_out_pressed = false;
bool pitch_up_pressed = false, pitch_down_pressed = false;

// Enum for button states
enum ButtonState {
  BUTTON_HIGH,
  BUTTON_LOW
};

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
unsigned long lastDebounceTimeStageArmSwitch = 0;
unsigned long lastDebounceTimeAbortArmSwitch = 0;
unsigned long lastDebounceTimeScienceButton = 0;
unsigned long lastDebounceTimeStageButton = 0;
unsigned long lastDebounceTimeTrimButton = 0;
unsigned long lastDebounceTimeResetTrimButton = 0;
unsigned long lastDebounceTimeLoadButton = 0;
unsigned long lastDebounceTimeSaveButton = 0;
unsigned long lastDebounceTimeShipsButton = 0;
unsigned long lastDebounceTimeCameraButton = 0;
unsigned long lastDebounceTimeMapButton = 0;
unsigned long lastDebounceTimeTimewarpMinusButton = 0;
unsigned long lastDebounceTimeTimewarpPauseButton = 0;
unsigned long lastDebounceTimeTimewarpPeriapsisButton = 0;
unsigned long lastDebounceTimeTimewarpManeuverButton = 0;
unsigned long lastDebounceTimeTimewarpApoapsisButton = 0;
unsigned long lastDebounceTimeTimewarpNormalButton = 0;
unsigned long lastDebounceTimeTimewarpPlusButton = 0;
unsigned long lastDebounceTimeAbortButton = 0;
unsigned long lastDebounceTimeActionGroup1Button = 0;
unsigned long lastDebounceTimeActionGroup2Button = 0;
unsigned long lastDebounceTimeActionGroup3Button = 0;
unsigned long lastDebounceTimeActionGroup4Button = 0;
unsigned long lastDebounceTimeActionGroup5Button = 0;
unsigned long lastDebounceTimeActionGroup6Button = 0;
unsigned long lastDebounceTimeActionGroup7Button = 0;
unsigned long lastDebounceTimeActionGroup8Button = 0;
unsigned long lastDebounceTimeActionGroup9Button = 0;
unsigned long lastDebounceTimeActionGroup10Button = 0;

// Variables to store the current and previous readings
int lastSASSwitchState = HIGH;  // Assume switch is not pressed initially
int lastLightsSwitchState = HIGH;  // Assume switch is not pressed initially
int lastStageArmSwitchState = HIGH;
int lastAbortArmSwitchState = HIGH;
int lastGearSwitchState = HIGH;
int lastBrakeSwitchState = HIGH;
int lastRCSSwitchState = HIGH;
int lastRADSSwitchState = HIGH;
int lastSolarSwitchState = HIGH;
int lastScienceButtonState = HIGH;  // Last stable state of the button
bool scienceButtonPressed = false;  // Track whether the button is pressed
bool StageArmend = false;
bool AbortArmend = false;
int lastStageButtonState = HIGH;
int lastTrimButtonState = HIGH;
int lastResetTrimButtonState = HIGH;
int lastLoadButtonState = HIGH;
int lastSaveButtonState = HIGH;
int lastShipsButtonState = HIGH;
int lastCameraButtonState = HIGH;
int lastMapButtonState = HIGH;
int lastTimewarpMinusButtonState = HIGH;
int lastTimewarpPauseButtonState = HIGH;
int lastTimewarpPeriapsisButtonState = HIGH;
int lastTimewarpManeuverButtonState = HIGH;
int lastTimewarpApoapsisButtonState = HIGH;
int lastTimewarpNormalButtonState = HIGH;
int lastTimewarpPlusButtonState = HIGH;
int lastAbortButtonState = HIGH;
int lastActionGroup1ButtonState = HIGH;
int lastActionGroup2ButtonState = HIGH;
int lastActionGroup3ButtonState = HIGH;
int lastActionGroup4ButtonState = HIGH;
int lastActionGroup5ButtonState = HIGH;
int lastActionGroup6ButtonState = HIGH;
int lastActionGroup7ButtonState = HIGH;
int lastActionGroup8ButtonState = HIGH;
int lastActionGroup9ButtonState = HIGH;
int lastActionGroup10ButtonState = HIGH;

int LastSASModePotValue = 0;
int LastControlModePotValue = 0;

int lcdScreenCase = 0;
int lcdScreenCaseBeforeAlarm = 0;
bool lcdAlarmState = false;
bool lcdAlarmStateOverride = false;

bool translationButtonPressed = false;

int readingPitchTrim = 0;
int readingYawTrim = 0;
int readingRollTrim = 0;

// Global variable declarations
airspeedMessage myAirspeed;
deltaVMessage myDeltaV;
altitudeMessage myAltitude;
velocityMessage myVelocity;
vesselPointingMessage myRotation;
tempLimitMessage myTemplimits;
atmoConditionsMessage myAtmoConditions;
resourceMessage myElectric;
flightStatusMessage myFlightStatus;

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

byte AdvancedActionStatusMessage[10];


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
  pinMode(LCD_BUTTON_PIN_RIGHT, INPUT_PULLUP);
  pinMode(LCD_BUTTON_PIN_LEFT, INPUT_PULLUP);
  pinMode(SCIENCE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STAGE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TRIM_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_TRIM_BUTTON_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_BUTTON_TRANSLATION, INPUT_PULLUP);
  pinMode(JOYSTICK_BUTTON_ROTATION, INPUT_PULLUP);
  pinMode(LOAD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SAVE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SHIPS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CAMERA_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MAP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIMEWARP_MINUS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIMEWARP_PAUSE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIMEWARP_PERIAPSIS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIMEWARP_MANEUVER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIMERWARP_APOAPSIS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIMEWARP_NORMAL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIMEWARP_PLUS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_10_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_9_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_8_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_7_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_6_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_5_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_4_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_3_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_2_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTION_GROUP_1_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ABORT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BRAKE_SWITCH, INPUT_PULLUP);
  pinMode(GEAR_SWITCH, INPUT_PULLUP);
  pinMode(RCS_SWITCH, INPUT_PULLUP);
  pinMode(SAS_SWITCH, INPUT_PULLUP);
  pinMode(RADS_SWITCH, INPUT_PULLUP);
  pinMode(SOLAR_SWITCH, INPUT_PULLUP);
  pinMode(LIGHTS_SWITCH, INPUT_PULLUP);
  pinMode(STAGE_ARM_SWITCH, INPUT_PULLUP);
  pinMode(ABORT_ARM_SWITCH, INPUT_PULLUP);  
  pinMode(LIGHTS_SWITCH_LED, OUTPUT);
  pinMode(POT_SAS_PIN, INPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LED_SCIENCE, OUTPUT);
  pinMode(LED_RADS, OUTPUT);
  pinMode(LED_SOLAR, OUTPUT);
  pinMode(LED_LIGHTS, OUTPUT);

    // Read initial state of switches
  lastSASSwitchState = digitalRead(SAS_SWITCH);
  lastLightsSwitchState = digitalRead(LIGHTS_SWITCH);
  lastStageArmSwitchState = digitalRead(STAGE_ARM_SWITCH);
  lastAbortArmSwitchState = digitalRead(ABORT_ARM_SWITCH);
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
  mySimpit.update();
  unsigned long now = millis();
  handleSwitches(now);
  handleButtons(now);
  handleLCDButtons(now);
  handleJoystickButtons(now);
  handleTempAlarm();
  SAS_mode_pot();
  Control_mode_pot();
  LEDS_ALARM_PANEL();
  
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
  mySimpit.registerChannel(FLIGHT_STATUS_MESSAGE);
  mySimpit.registerChannel(ADVANCED_ACTIONSTATUS_MESSAGE);
  
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
  int readingStageArmSwitch = digitalRead(STAGE_ARM_SWITCH);
  int readingAbortArmSwitch = digitalRead(ABORT_ARM_SWITCH);

  // Handle SOLAR Switch
  if (readingSolarSwitch != lastSolarSwitchState && (now - lastDebounceTimeSolarSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeSolarSwitch = now;

    if (readingSolarSwitch == LOW) {  // Button pressed
      mySimpit.deactivateCAG(9);
      //mySimpit.toggleCAG(9);
      digitalWrite(LED_SOLAR, LOW);
    } else {  // Button released
     
      //mySimpit.toggleCAG(9);
      mySimpit.activateCAG(9);
      digitalWrite(LED_SOLAR, HIGH);
    }

    // Update the last state
    lastSolarSwitchState = readingSolarSwitch;
  }



  // Handle RADS Switch
  if (readingRADSSwitch != lastRADSSwitchState && (now - lastDebounceTimeRADSSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeRADSSwitch = now;

    if (readingRADSSwitch == LOW) {  // Button pressed
      
      mySimpit.deactivateCAG(10);
      digitalWrite(LED_RADS, LOW);

    } else {  // Button released
     mySimpit.activateCAG(10);
      digitalWrite(LED_RADS, HIGH);
      
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
      digitalWrite(LED_LIGHTS, LOW);

    } else {  // Button released
      
      mySimpit.activateAction(LIGHT_ACTION);
      digitalWrite(LED_LIGHTS, HIGH);
    }

    // Update the last state
    lastLightsSwitchState = readingLightsSwitch;
  }

 // Handle the STAGE ARM Switch with LED
  if (readingStageArmSwitch != lastStageArmSwitchState && (now - lastDebounceTimeStageArmSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeStageArmSwitch = now;

    if (readingStageArmSwitch == LOW) {  // Button pressed
      
      setLED(LED_STAGE, true);
      StageArmend = true;

    } else {  // Button released
      
      StageArmend = false;
      setLED(LED_STAGE, false);
    }

    // Update the last state
    lastStageArmSwitchState = readingStageArmSwitch;
  }

// Handle the Abort ARM Switch with LED
  if (readingAbortArmSwitch != lastAbortArmSwitchState && (now - lastDebounceTimeAbortArmSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeAbortArmSwitch = now;

    if (readingAbortArmSwitch == LOW) {  // Button pressed
      
      setLED(LED_ARM_ABORT, true);
      AbortArmend = true;

    } else {  // Button released
      
      AbortArmend = false;
      setLED(LED_ARM_ABORT, false);
    }

    // Update the last state
    lastAbortArmSwitchState = readingAbortArmSwitch;
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
        if (StageArmend == true) {
        mySimpit.activateAction(STAGE_ACTION);
        }
        lastDebounceTimeJoystickRotation = now;
      }
}

void handleButtons(unsigned long now) {
  int readingScienceButton = digitalRead(SCIENCE_BUTTON_PIN);
  int readingStageButton = digitalRead(STAGE_BUTTON_PIN);
  int readingTrimButton = digitalRead(TRIM_BUTTON_PIN);
  int readingResetTrimButton = digitalRead(RESET_TRIM_BUTTON_PIN);

  //--------------------
  // Check for state change and debounce for Science button
  if (readingScienceButton != lastScienceButtonState && (now - lastDebounceTimeScienceButton) > DEBOUNCE_DELAY) {
    if (readingScienceButton == LOW) {
      scienceButtonPressed = !scienceButtonPressed;
      mySimpit.printToKSP("Science button pressed", PRINT_TO_SCREEN);
      mySimpit.toggleCAG(8);
      blinkLedFor5Seconds();
      

      
    }
    lastDebounceTimeScienceButton = now;
  }
  lastScienceButtonState = readingScienceButton;
  //--------------------

  //--------------------
  // Check for state change and debounce for Stage button
  if (readingStageButton != lastStageButtonState && (now - lastDebounceTimeStageButton) > DEBOUNCE_DELAY) {
    if (readingStageButton == LOW) {
      
        if (StageArmend == true) {
        mySimpit.activateAction(STAGE_ACTION);
        mySimpit.printToKSP("Stage button pressed", PRINT_TO_SCREEN);
        }
    }
    lastDebounceTimeStageButton = now;
  }
  lastStageButtonState = readingStageButton;
  //--------------------

  //--------------------
  // Trim button debouncing logic
  readingTrimButton = digitalRead(TRIM_BUTTON_PIN);
  
  if (readingTrimButton != lastTrimButtonState && (now - lastDebounceTimeTrimButton) > DEBOUNCE_DELAY) {
    if (readingTrimButton == LOW) {
      mySimpit.printToKSP("Trim button pressed", PRINT_TO_SCREEN);
      readingPitchTrim = analogRead(PITCH_PIN) -512;
      readingRollTrim = analogRead(ROLL_PIN) -512;
      readingYawTrim = analogRead(YAW_PIN) -512;
    }
    lastDebounceTimeTrimButton = now;
  }
  lastTrimButtonState = readingTrimButton;

  // Reset Trim button debouncing logic
  readingResetTrimButton = digitalRead(RESET_TRIM_BUTTON_PIN);
  if (readingResetTrimButton != lastResetTrimButtonState && (now - lastDebounceTimeResetTrimButton) > DEBOUNCE_DELAY) {
    if (readingResetTrimButton == LOW) {
      mySimpit.printToKSP("ResetTrim button pressed", PRINT_TO_SCREEN);
      readingPitchTrim = 0;
      readingRollTrim = 0;
      readingYawTrim = 0;
    }
    lastDebounceTimeResetTrimButton = now;
  }
  lastResetTrimButtonState = readingResetTrimButton;

  // Load button debouncing logic
  readingLoadButton = digitalRead(LOAD_BUTTON_PIN);
  if (readingLoadButton != lastLoadButtonState && (now - lastDebounceTimeLoadButton) > DEBOUNCE_DELAY) {
    if (readingLoadButton == LOW) {
      mySimpit.printToKSP("Load button pressed", PRINT_TO_SCREEN);
      keyboardEmulatorMessage loadMsg(LOAD_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, loadMsg);
    }
    lastDebounceTimeLoadButton = now;
  }
  lastLoadButtonState = readingLoadButton;

  // Save button debouncing logic
  readingSaveButton = digitalRead(SAVE_BUTTON_PIN);
  if (readingSaveButton != lastSaveButtonState && (now - lastDebounceTimeSaveButton) > DEBOUNCE_DELAY) {
    if (readingSaveButton == LOW) {
      mySimpit.printToKSP("Save button pressed", PRINT_TO_SCREEN);
      keyboardEmulatorMessage SaveMsg(SAVE_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, SaveMsg);
    }
    keyboardEmulatorMessage SaveMsg(SAVE_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, SaveMsg);
    lastDebounceTimeSaveButton = now;
  }
  lastSaveButtonState = readingSaveButton;

  // Ships button debouncing logic
  readingShipsButton = digitalRead(SHIPS_BUTTON_PIN);
  if (readingShipsButton != lastShipsButtonState && (now - lastDebounceTimeShipsButton) > DEBOUNCE_DELAY) {
    if (readingShipsButton == LOW) {
      mySimpit.printToKSP("Ships button pressed", PRINT_TO_SCREEN);
      keyboardEmulatorMessage ShipsMsg(SHIPS_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, ShipsMsg);
    }
    lastDebounceTimeShipsButton = now;
    keyboardEmulatorMessage ShipsMsg(SHIPS_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, ShipsMsg);
  }
  lastShipsButtonState = readingShipsButton;

  // Camera button debouncing logic
  readingCameraButton = digitalRead(CAMERA_BUTTON_PIN);
  if (readingCameraButton != lastCameraButtonState && (now - lastDebounceTimeCameraButton) > DEBOUNCE_DELAY) {
    if (readingCameraButton == LOW) {
      mySimpit.printToKSP("Camera button pressed", PRINT_TO_SCREEN);
      keyboardEmulatorMessage CameraMsg(CAMERA_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, CameraMsg);
    }
    keyboardEmulatorMessage CameraMsg(CAMERA_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, CameraMsg);
    lastDebounceTimeCameraButton = now;
  }
  lastCameraButtonState = readingCameraButton;

  // Map button debouncing logic
  readingMapButton = digitalRead(MAP_BUTTON_PIN);
  if (readingMapButton != lastMapButtonState && (now - lastDebounceTimeMapButton) > DEBOUNCE_DELAY) {
    if (readingMapButton == LOW) {
      mySimpit.printToKSP("Map button pressed", PRINT_TO_SCREEN);
      keyboardEmulatorMessage MapMsg(MAP_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, MapMsg);
    }
    keyboardEmulatorMessage MapMsg(MAP_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, MapMsg);
    lastDebounceTimeMapButton = now;

  }
  lastMapButtonState = readingMapButton;

  // Timewarp Minus button debouncing logic
  readingTimewarpMinusButton = digitalRead(TIMEWARP_MINUS_BUTTON_PIN);
  if (readingTimewarpMinusButton != lastTimewarpMinusButtonState && (now - lastDebounceTimeTimewarpMinusButton) > DEBOUNCE_DELAY) {
    if (readingTimewarpMinusButton == LOW) {
      mySimpit.printToKSP("Timewarp Minus button pressed", PRINT_TO_SCREEN);
      timewarpMessage tw_msg_down;
      tw_msg_down.command = TIMEWARP_DOWN;
      mySimpit.send(TIMEWARP_MESSAGE, tw_msg_down);
    }
    lastDebounceTimeTimewarpMinusButton = now;
  }
  lastTimewarpMinusButtonState = readingTimewarpMinusButton;

  // Timewarp Pause button debouncing logic
  readingTimewarpPauseButton = digitalRead(TIMEWARP_PAUSE_BUTTON_PIN);
  if (readingTimewarpPauseButton != lastTimewarpPauseButtonState && (now - lastDebounceTimeTimewarpPauseButton) > DEBOUNCE_DELAY) {
    if (readingTimewarpPauseButton == LOW) {
      mySimpit.printToKSP("Timewarp Pause button pressed", PRINT_TO_SCREEN);
      keyboardEmulatorMessage PauseMsg(PAUSE_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, PauseMsg);
    }
    keyboardEmulatorMessage PauseMsg(PAUSE_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, PauseMsg);
    lastDebounceTimeTimewarpPauseButton = now;
  }
  lastTimewarpPauseButtonState = readingTimewarpPauseButton;

  // Timewarp Periapsis button debouncing logic
  readingTimewarpPeriapsisButton = digitalRead(TIMEWARP_PERIAPSIS_BUTTON_PIN);
  if (readingTimewarpPeriapsisButton != lastTimewarpPeriapsisButtonState && (now - lastDebounceTimeTimewarpPeriapsisButton) > DEBOUNCE_DELAY) {
    if (readingTimewarpPeriapsisButton == LOW) {
      mySimpit.printToKSP("Timewarp Periapsis button pressed", PRINT_TO_SCREEN);
      timewarpToMessage twTo_msg(TIMEWARP_TO_PERIAPSIS, -15);
      mySimpit.send(TIMEWARP_TO_MESSAGE, twTo_msg);
    }
    lastDebounceTimeTimewarpPeriapsisButton = now;
  }
  lastTimewarpPeriapsisButtonState = readingTimewarpPeriapsisButton;

  // Timewarp Maneuver button debouncing logic
  readingTimewarpManeuverButton = digitalRead(TIMEWARP_MANEUVER_BUTTON_PIN);
  if (readingTimewarpManeuverButton != lastTimewarpManeuverButtonState && (now - lastDebounceTimeTimewarpManeuverButton) > DEBOUNCE_DELAY) {
    if (readingTimewarpManeuverButton == LOW) {
      mySimpit.printToKSP("Timewarp Maneuver button pressed", PRINT_TO_SCREEN);
      timewarpToMessage twTo_msg(TIMEWARP_TO_NEXT_MANEUVER, -15);
      mySimpit.send(TIMEWARP_TO_MESSAGE, twTo_msg);
    }
    lastDebounceTimeTimewarpManeuverButton = now;
  }
  lastTimewarpManeuverButtonState = readingTimewarpManeuverButton;

  // Timewarp Apoapsis button debouncing logic
  readingTimewarpApoapsisButton = digitalRead(TIMERWARP_APOAPSIS_BUTTON_PIN);
  if (readingTimewarpApoapsisButton != lastTimewarpApoapsisButtonState && (now - lastDebounceTimeTimewarpApoapsisButton) > DEBOUNCE_DELAY) {
    if (readingTimewarpApoapsisButton == LOW) {
      mySimpit.printToKSP("Timewarp Apoapsis button pressed", PRINT_TO_SCREEN);
      timewarpToMessage twTo_msg(TIMEWARP_TO_APOAPSIS, -15);
      mySimpit.send(TIMEWARP_TO_MESSAGE, twTo_msg);
    }
    lastDebounceTimeTimewarpApoapsisButton = now;
  }
  lastTimewarpApoapsisButtonState = readingTimewarpApoapsisButton;

  // Timewarp Normal button debouncing logic
  readingTimewarpNormalButton = digitalRead(TIMEWARP_NORMAL_BUTTON_PIN);
  if (readingTimewarpNormalButton != lastTimewarpNormalButtonState && (now - lastDebounceTimeTimewarpNormalButton) > DEBOUNCE_DELAY) {
    if (readingTimewarpNormalButton == LOW) {
      mySimpit.printToKSP("Timewarp Normal button pressed", PRINT_TO_SCREEN);
      timewarpMessage tw_msg_x1;
      tw_msg_x1.command = TIMEWARP_X1;
      mySimpit.send(TIMEWARP_MESSAGE, tw_msg_x1);
      timewarpMessage tw_msg_cancel;
      tw_msg_cancel.command = TIMEWARP_CANCEL_AUTOWARP;
      mySimpit.send(TIMEWARP_MESSAGE, tw_msg_cancel);

      
    }
    lastDebounceTimeTimewarpNormalButton = now;
  }
  lastTimewarpNormalButtonState = readingTimewarpNormalButton;

  // Timewarp Plus button debouncing logic
  readingTimewarpPlusButton = digitalRead(TIMEWARP_PLUS_BUTTON_PIN);
  if (readingTimewarpPlusButton != lastTimewarpPlusButtonState && (now - lastDebounceTimeTimewarpPlusButton) > DEBOUNCE_DELAY) {
    if (readingTimewarpPlusButton == LOW) {
      mySimpit.printToKSP("Timewarp Plus button pressed", PRINT_TO_SCREEN);
      timewarpMessage tw_msg_up;
      tw_msg_up.command = TIMEWARP_UP;
      mySimpit.send(TIMEWARP_MESSAGE, tw_msg_up);

    }
    lastDebounceTimeTimewarpPlusButton = now;
  }
  lastTimewarpPlusButtonState = readingTimewarpPlusButton;

  // Action Group 1 button debouncing logic
  readingActionGroup1Button = digitalRead(ACTION_GROUP_1_BUTTON_PIN);
  if (readingActionGroup1Button != lastActionGroup1ButtonState && (now - lastDebounceTimeActionGroup1Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup1Button == LOW) {
      mySimpit.printToKSP("Action Group 1 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
       mySimpit.toggleCAG(1);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
        
    }
    lastDebounceTimeActionGroup1Button = now;
  }
  lastActionGroup1ButtonState = readingActionGroup1Button;

  // Action Group 2 button debouncing logic
  readingActionGroup2Button = digitalRead(ACTION_GROUP_2_BUTTON_PIN);
  if (readingActionGroup2Button != lastActionGroup2ButtonState && (now - lastDebounceTimeActionGroup2Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup2Button == LOW) {
      mySimpit.printToKSP("Action Group 2 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
      mySimpit.toggleCAG(2);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
    }
    lastDebounceTimeActionGroup2Button = now;
  }
  lastActionGroup2ButtonState = readingActionGroup2Button;

  // Action Group 3 button debouncing logic
  readingActionGroup3Button = digitalRead(ACTION_GROUP_3_BUTTON_PIN);
  if (readingActionGroup3Button != lastActionGroup3ButtonState && (now - lastDebounceTimeActionGroup3Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup3Button == LOW) {
      mySimpit.printToKSP("Action Group 3 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
      mySimpit.toggleCAG(3);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
    }
    lastDebounceTimeActionGroup3Button = now;
  }
  lastActionGroup3ButtonState = readingActionGroup3Button;

  // Action Group 4 button debouncing logic
  readingActionGroup4Button = digitalRead(ACTION_GROUP_4_BUTTON_PIN);
  if (readingActionGroup4Button != lastActionGroup4ButtonState && (now - lastDebounceTimeActionGroup4Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup4Button == LOW) {
      mySimpit.printToKSP("Action Group 4 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
      mySimpit.toggleCAG(4);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
    }
    lastDebounceTimeActionGroup4Button = now;
  }
  lastActionGroup4ButtonState = readingActionGroup4Button;

  // Action Group 5 button debouncing logic
  readingActionGroup5Button = digitalRead(ACTION_GROUP_5_BUTTON_PIN);
  if (readingActionGroup5Button != lastActionGroup5ButtonState && (now - lastDebounceTimeActionGroup5Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup5Button == LOW) {
      mySimpit.printToKSP("Action Group 5 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
      mySimpit.toggleCAG(5);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
    }
    lastDebounceTimeActionGroup5Button = now;
  }
  lastActionGroup5ButtonState = readingActionGroup5Button;

  // Action Group 6 button debouncing logic
  readingActionGroup6Button = digitalRead(ACTION_GROUP_6_BUTTON_PIN);
  if (readingActionGroup6Button != lastActionGroup6ButtonState && (now - lastDebounceTimeActionGroup6Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup6Button == LOW) {
      mySimpit.printToKSP("Action Group 6 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
      mySimpit.toggleCAG(6);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
    }
    lastDebounceTimeActionGroup6Button = now;
  }
  lastActionGroup6ButtonState = readingActionGroup6Button;

  // Action Group 7 button debouncing logic
  readingActionGroup7Button = digitalRead(ACTION_GROUP_7_BUTTON_PIN);
  if (readingActionGroup7Button != lastActionGroup7ButtonState && (now - lastDebounceTimeActionGroup7Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup7Button == LOW) {
      mySimpit.printToKSP("Action Group 7 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
      mySimpit.toggleCAG(7);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
    }
    lastDebounceTimeActionGroup7Button = now;
  }
  lastActionGroup7ButtonState = readingActionGroup7Button;

  // Action Group 8 button debouncing logic
  readingActionGroup8Button = digitalRead(ACTION_GROUP_8_BUTTON_PIN);
  if (readingActionGroup8Button != lastActionGroup8ButtonState && (now - lastDebounceTimeActionGroup8Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup8Button == LOW) {
      mySimpit.printToKSP("Action Group 8 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
      mySimpit.toggleCAG(8);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
    }
    lastDebounceTimeActionGroup8Button = now;
  }
  lastActionGroup8ButtonState = readingActionGroup8Button;

  // Action Group 9 button debouncing logic
  readingActionGroup9Button = digitalRead(ACTION_GROUP_9_BUTTON_PIN);
  if (readingActionGroup9Button != lastActionGroup9ButtonState && (now - lastDebounceTimeActionGroup9Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup9Button == LOW) {
      mySimpit.printToKSP("Action Group 9 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
      mySimpit.toggleCAG(9);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
    }
    lastDebounceTimeActionGroup9Button = now;
  }
  lastActionGroup9ButtonState = readingActionGroup9Button;

  // Action Group 10 button debouncing logic
  readingActionGroup10Button = digitalRead(ACTION_GROUP_10_BUTTON_PIN);
  if (readingActionGroup10Button != lastActionGroup10ButtonState && (now - lastDebounceTimeActionGroup10Button) > DEBOUNCE_DELAY) {
    if (readingActionGroup10Button == LOW) {
      mySimpit.printToKSP("Action Group 10 button pressed", PRINT_TO_SCREEN);
      setLED(LED_ACTION_GROUP, true);
      mySimpit.toggleCAG(10);
    } else {
        // If button is released (HIGH)
        setLED(LED_ACTION_GROUP, false); // Turn off the LED
        
    }
    lastDebounceTimeActionGroup10Button = now;
  }
  lastActionGroup10ButtonState = readingActionGroup10Button;

  // Abort button debouncing logic
  readingAbortButton = digitalRead(ABORT_BUTTON_PIN);
  if (readingAbortButton != lastAbortButtonState && (now - lastDebounceTimeAbortButton) > DEBOUNCE_DELAY) {
    if (readingAbortButton == LOW) {
      
      if (AbortArmend == true) {
      mySimpit.printToKSP("Abort button pressed", PRINT_TO_SCREEN);
      mySimpit.activateAction(ABORT_ACTION);
      }
      
    }
    lastDebounceTimeAbortButton = now;
  }
  lastAbortButtonState = readingAbortButton;
}



// Function to handle LCD buttons with debouncing
void handleLCDButtons(unsigned long now) {
  int readingLCDSwitchPinRight = digitalRead(LCD_BUTTON_PIN_RIGHT);
  int readingLCDSwitchPinLeft = digitalRead(LCD_BUTTON_PIN_LEFT);

  // Handle the right button
  if (readingLCDSwitchPinRight == LOW && (now - lastDebounceTimeRight) > DEBOUNCE_DELAY) {
    if (lcdScreenCase < 8) {
      lcdScreenCase++;
      lcdScreenCaseBeforeAlarm = lcdScreenCase;
      lcd.clear(); // clear screen for new display info
      if (lcdAlarmState) {
        lcdAlarmStateOverride = false;
        lcdAlarmState = false;
        setLED(LED_MASTER_ALARM, false);
        setLED(LED_TEMPRATURE, false);
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
        setLED(LED_MASTER_ALARM, false);
        setLED(LED_TEMPRATURE, false);
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
      setLED(LED_MASTER_ALARM, true);
      setLED(LED_TEMPRATURE, true);
      lcd.setBacklight(255, 0, 0);
      lcd.clear();
    }
  } else {
    if (lcdAlarmState) {
      lcdAlarmState = false;
      lcdAlarmStateOverride = false;
      lcdScreenCase = lcdScreenCaseBeforeAlarm;\
      setLED(LED_MASTER_ALARM, false);
      setLED(LED_TEMPRATURE, false);
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

  int16_t throttlePercentage = 0; // Calculate throttle in 0-100%

// Interpolate the throttle percentage based on observed ranges
if (reading >= 990) {
  throttlePercentage = 0; // Explicitly set to zero at maximum reading
} else if (reading >= 900) {
  throttlePercentage = 0 + (995 - reading) * 25 / (995 - 900);
} else if (reading >= 500) {
  throttlePercentage = 25 + (900 - reading) * 25 / (900 - 500);
} else if (reading >= 118) {
  throttlePercentage = 50 + (500 - reading) * 25 / (500 - 118);
} else if (reading >= 8) {
  throttlePercentage = 75 + (118 - reading) * 25 / (118 - 8);
} else {
  throttlePercentage = 100;
}

  // Scale throttlePercentage (0-100) to 0 to INT16_MAX (32767)
  throttleMsg.throttle = map(throttlePercentage, 0, 100, 0, INT16_MAX);

  // Send throttle message
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

  //adding Trim vallues
  readingPitch += readingPitchTrim; // Add the trim value
  readingPitch = constrain(readingPitch, 0, 1023); // Limit the value to 0-1023

  readingRoll += readingRollTrim;
  readingRoll = constrain(readingRoll, 0, 1023);

  readingYaw += readingYawTrim;
  readingYaw = constrain(readingYaw, 0, 1023);


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
        LEDS_ALARM_PANEL();
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

    case FLIGHT_STATUS_MESSAGE: {
        if (msgSize == sizeof(flightStatusMessage)) {
          myFlightStatus = parseMessage<flightStatusMessage>(msg);
          LEDS_ALARM_PANEL();
        }
      } break;

    case ADVANCED_ACTIONSTATUS_MESSAGE: {
        if (msgSize == sizeof(advancedActionStatusMessage) )
        {
          advancedActionStatusMessage actionStatusMsg = parseMessage<advancedActionStatusMessage>(msg);
          for(int i = 0; i < 10; i++) //There are 9 action groups (stage, gear, ...), they are listed in the AdvancedActionGroupIndexes enum. Some are only available for KSP2
          {
            AdvancedActionStatusMessage[i] = actionStatusMsg.getActionStatus(i);
          }
        }
      } break;


  }
  
}

void SAS_mode_pot() {
  // Read the potentiometer value (0 to 1023)
  int POT_SAS_VALUE = analogRead(POT_SAS_PIN);
  // Check if the potentiometer value has changed by more than 10
  if (abs(LastSASModePotValue - POT_SAS_VALUE) > 3) {
    LastSASModePotValue = POT_SAS_VALUE;
    mySimpit.printToKSP(" update SAS MODE", PRINT_TO_SCREEN);

    // Clear only the LEDs used in this function
    clearSASModeLEDs();

    // Define custom potentiometer ranges for each LED and turn them on accordingly
    if (POT_SAS_VALUE >= 0 && POT_SAS_VALUE < 10) {
      setLED(LED_AUTO_PILOT, true);
    } else if (POT_SAS_VALUE >= 10 && POT_SAS_VALUE < 95) {
      setLED(LED_ANTI_NORMAL, true);
      mySimpit.setSASMode(AP_ANTINORMAL);
    } else if (POT_SAS_VALUE >= 95 && POT_SAS_VALUE < 195) {
      setLED(LED_NORMAL, true);
      mySimpit.setSASMode(AP_NORMAL);
    } else if (POT_SAS_VALUE >= 195 && POT_SAS_VALUE < 310) {
      setLED(LED_RETRO_GRADE, true);
      mySimpit.setSASMode(AP_RETROGRADE);
    } else if (POT_SAS_VALUE >= 310 && POT_SAS_VALUE < 420) {
      setLED(LED_PRO_GRADE, true);
      mySimpit.setSASMode(AP_PROGRADE);
    } else if (POT_SAS_VALUE >= 420 && POT_SAS_VALUE < 507) {
      setLED(LED_MANAUVER, true);
      mySimpit.setSASMode(AP_MANEUVER);
    } else if (POT_SAS_VALUE >= 507 && POT_SAS_VALUE < 600) {
      setLED(LED_STABILITY_ASSIST, true);
      mySimpit.setSASMode(AP_STABILITYASSIST);
    } else if (POT_SAS_VALUE >= 600 && POT_SAS_VALUE < 690) {
      setLED(LED_RADIAL_OUT, true);
      mySimpit.setSASMode(AP_RADIALOUT);
    } else if (POT_SAS_VALUE >= 690 && POT_SAS_VALUE < 790) {
      setLED(LED_RADIAL_IN, true);
      mySimpit.setSASMode(AP_RADIALIN);
    } else if (POT_SAS_VALUE >= 790 && POT_SAS_VALUE < 875) {
      setLED(LED_TARGET, true);
      mySimpit.setSASMode(AP_TARGET);
    } else if (POT_SAS_VALUE >= 875 && POT_SAS_VALUE < 975) {
      setLED(LED_ANTI_TARGET, true);
      mySimpit.setSASMode(AP_ANTITARGET);
    } else if (POT_SAS_VALUE >= 975 && POT_SAS_VALUE < 1024) {
      setLED(LED_NAVIGATION, true);
      mySimpit.printToKSP(F("Navigation"), PRINT_TO_SCREEN);
    }
  }
}

void Control_mode_pot() {
  // Read the potentiometer value (0 to 1023)
  int POT_CONTROL_VALUE = analogRead(POT_CONTROL_PIN);
  // Check if the potentiometer value has changed by more than 10
  if (abs(LastControlModePotValue - POT_CONTROL_VALUE) > 3) {
    LastControlModePotValue = POT_CONTROL_VALUE;
    mySimpit.printToKSP(" update CONTROL MODE", PRINT_TO_SCREEN);
    mySimpit.printToKSP(" update CONTROL MODE", PRINT_TO_SCREEN);
    // Clear only the LEDs used in this function
    clearControlModeLEDs();

    // Define custom potentiometer ranges for each LED and turn them on accordingly
    if (POT_CONTROL_VALUE >= 0 && POT_CONTROL_VALUE < 170) {
      
      setLED(LED_PlANE_MODE, true);

    } else if (POT_CONTROL_VALUE >= 170 && POT_CONTROL_VALUE < 340) {
      setLED(LED_ROVER_MODE, true);
 
    } else if (POT_CONTROL_VALUE >= 340 && POT_CONTROL_VALUE < 510) {
      setLED(LED_ROCKET_MODE, true);

    } else if (POT_CONTROL_VALUE >= 510 && POT_CONTROL_VALUE < 680) {
      
      setLED(LED_PRECISION, true);

    } else if (POT_CONTROL_VALUE >= 680 && POT_CONTROL_VALUE < 850) {
      
      setLED(LED_EVA_MODE, true);
     
    } else if (POT_CONTROL_VALUE >= 920 && POT_CONTROL_VALUE < 1024) {
      
      setLED(LED_DOCKING_MODE, true);
    } 
  }
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

// Function to clear only the LEDs used in Control_mode_pot()
void clearControlModeLEDs() {
  setLED(LED_DOCKING_MODE, false);
  setLED(LED_EVA_MODE, false);
  setLED(LED_PRECISION, false);
  setLED(LED_ROCKET_MODE, false);
  setLED(LED_ROVER_MODE, false);
  setLED(LED_PlANE_MODE, false);
}

// Function to set the state of a specific LED (on or off)
void setLED(int led, bool state) {
  if (led < 8) {  // First shift register (LEDs 0-7)
    if (state) {
      ledStates1 |= (1 << led);   // Set bit to 1 (turn on)
    } else {
      ledStates1 &= ~(1 << led);  // Set bit to 0 (turn off)
    }
  } else if (led < 16) {  // Second shift register (LEDs 8-15)
    int shiftRegisterLED = led - 8;
    if (state) {
      ledStates2 |= (1 << shiftRegisterLED);  // Set bit to 1 (turn on)
    } else {
      ledStates2 &= ~(1 << shiftRegisterLED); // Set bit to 0 (turn off)
    }
  } else if (led < 24) {  // Third shift register (LEDs 16-23)
    int shiftRegisterLED = led - 16;
    if (state) {
      ledStates3 |= (1 << shiftRegisterLED);  // Set bit to 1 (turn on)
    } else {
      ledStates3 &= ~(1 << shiftRegisterLED); // Set bit to 0 (turn off)
    }
  } else {  // Fourth shift register (LEDs 24-31)
    int shiftRegisterLED = led - 24;
    if (state) {
      ledStates4 |= (1 << shiftRegisterLED);  // Set bit to 1 (turn on)
    } else {
      ledStates4 &= ~(1 << shiftRegisterLED); // Set bit to 0 (turn off)
    }
  }
  updateShiftRegisters();
}

// Function to update the shift registers with current LED states
void updateShiftRegisters() {
  digitalWrite(LATCH_PIN, LOW);         // Prepare to send data
  shiftOut(DATA_PIN, CLOCK_PIN, ledStates4);  // Send data for SR4 (LEDs 25-32)
  shiftOut(DATA_PIN, CLOCK_PIN, ledStates3);  // Send data for SR3 (LEDs 17-24)
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




// Function to enable all LEDs in all four shift registers (LEDs 1-32)
void ALL_LEDS_ON() {
  // Set all bits to 1 in all shift register variables
  ledStates1 = 0xFF;  // All LEDs in the first shift register (LEDs 1-8) ON
  ledStates2 = 0xFF;  // All LEDs in the second shift register (LEDs 9-16) ON
  ledStates3 = 0xFF;  // All LEDs in the third shift register (LEDs 17-24) ON
  ledStates4 = 0xFF;  // All LEDs in the fourth shift register (LEDs 25-32) ON

  // Turn on non-shift-register LEDs
  digitalWrite(LED_LIGHTS, HIGH);
  digitalWrite(LED_SOLAR, HIGH);
  digitalWrite(LED_RADS, HIGH);
  digitalWrite(LED_SCIENCE, HIGH);
  // Update shift registers to apply the changes
  updateShiftRegisters();
}

// Function to turn off all LEDs in all four shift registers (LEDs 1-32)
// and the non-shift-register LEDs.
void ALL_LEDS_OFF() {
  // Set all bits to 0 in all shift register variables
  ledStates1 = 0x00;  // All LEDs in the first shift register (LEDs 1-8) OFF
  ledStates2 = 0x00;  // All LEDs in the second shift register (LEDs 9-16) OFF
  ledStates3 = 0x00;  // All LEDs in the third shift register (LEDs 17-24) OFF
  ledStates4 = 0x00;  // All LEDs in the fourth shift register (LEDs 25-32) OFF

  
  digitalWrite(LED_LIGHTS, LOW);
  digitalWrite(LED_SOLAR, LOW);
  digitalWrite(LED_RADS, LOW);
  digitalWrite(LED_SCIENCE, LOW);
  // Update shift registers to apply the changes
  updateShiftRegisters();
}

void LEDS_ALARM_PANEL(){
  if (myAtmoConditions.isVesselInAtmosphere()) {
    setLED(LED_ATMOS,true);
    if (myAtmoConditions.hasOxygen()) {
    setLED(LED_OXYGEN,true);
  } else {
    setLED(LED_OXYGEN,false);
  }
  } else {
    setLED(LED_ATMOS,false);
    setLED(LED_OXYGEN,false);
  }


  if (round(myVelocity.surface) < 3){
    if (myFlightStatus.isRecoverable()){
    setLED(LED_RECOVER,true);
    }else {
    setLED(LED_RECOVER,false);
  } 
  } else {
    setLED(LED_RECOVER,false);
  } 

}



