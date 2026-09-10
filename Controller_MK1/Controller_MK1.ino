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

// EVA Mode WASD key codes and tracking
const int EVA_W_KEY = 0x57;  // W key - Backward
const int EVA_A_KEY = 0x41;  // A key - Left
const int EVA_S_KEY = 0x53;  // S key - Forward
const int EVA_D_KEY = 0x44;  // D key - Right
const int EVA_Q_KEY = 0x51;  // Q key - Yaw Left
const int EVA_E_KEY = 0x45;  // E key - Yaw Right
const int EVA_SHIFT_KEY = 0x10;  // SHIFT key
const int EVA_SPACE_KEY = 0x20;  // SPACE key
const int EVA_R_KEY = 0x52;  // R key - Boost toggle
bool eva_w_pressed = false;  // Backward (W)
bool eva_a_pressed = false;  // Left (A)
bool eva_s_pressed = false;  // Forward (S)
bool eva_d_pressed = false;  // Right (D)
bool eva_q_pressed = false;  // Yaw Left (Q)
bool eva_e_pressed = false;  // Yaw Right (E)
bool eva_shift_space_pressed = false;  // Shift+Space boost
unsigned long lastDoubleRPressTime = 0;  // Tracking time for double R press
const long DOUBLE_PRESS_TIMEOUT = 300;  // milliseconds for double press window
int lastRotationButtonState = HIGH;  // Track previous rotation button state
bool rotationButtonHeld = false;  // Track if button is being held

// Enum for button states
enum ButtonState {
  BUTTON_HIGH,
  BUTTON_LOW
};

// Global Variables
KerbalSimpit mySimpit(Serial);
SerLCD lcd;
bool isConnected = false;

// Connection watchdog
// Set to 1 to let the controller declare the link dead and re-handshake.
// Set to 0 to only measure (heartbeat + diagnostics on LCD screen 9).
// Left at 0: on this rig it produced spurious disconnects. Do not enable it
// until screen 9 shows an Age peak that stays well under CONNECTION_TIMEOUT.
#define LINK_WATCHDOG 0

unsigned long lastInboundMessageTime = 0;
unsigned long lastHeartbeatSent = 0;
unsigned long lastReconnectAttempt = 0;
const unsigned long HEARTBEAT_INTERVAL = 2000;  // Send an echo request this often
const unsigned long CONNECTION_TIMEOUT  = 6000;  // Three missed heartbeats = link lost
const unsigned long RECONNECT_INTERVAL  = 2000;  // init() blocks ~1.1s on failure, so don't retry faster

// Set to true the first time KSP answers an echo request. Until that happens the
// watchdog stays disarmed: outside the flight scene Simpit sends nothing at all,
// so silence on its own is not proof of a dead link.
bool echoSupported = false;

// Largest gap ever seen between inbound packets, in ms. This is the number that
// decides whether the watchdog can be trusted: it keeps counting even while the
// link looks fine, so real gaps cannot hide.
unsigned long ageMaxMs = 0;

extern byte solarStatus;
extern byte radiatorStatus;
extern bool scienceAvailable;
extern bool scienceLedState;
extern unsigned long lastScienceBlink;
extern const unsigned long SCIENCE_BLINK_INTERVAL;

// Milliseconds since the last inbound packet. Always read millis() fresh here:
// lastInboundMessageTime is updated from messageHandler part way through the
// loop, so a cached "now" from the top of the loop can be older than it and the
// unsigned subtraction would wrap to ~4294967295.
unsigned long inboundAgeMs() {
  unsigned long stamp = lastInboundMessageTime;
  unsigned long current = millis();
  if (current < stamp) {
    return 0;
  }
  return current - stamp;
}

void sendAdvancedAction(byte actionIndex, ActionGroupSettings setting) {
  setSingleActionGroupMessage action(actionIndex, setting);
  mySimpit.send(SETSINGLE_AG_MESSAGE, action);
}

void updateScienceLED(unsigned long now) {
  if (!scienceAvailable) {
    scienceLedState = false;
    digitalWrite(LED_SCIENCE, LOW);
    return;
  }

  if (now - lastScienceBlink >= SCIENCE_BLINK_INTERVAL) {
    lastScienceBlink = now;
    scienceLedState = !scienceLedState;
    digitalWrite(LED_SCIENCE, scienceLedState ? HIGH : LOW);
  }
}

void updateAdvancedActionLEDs() {
  digitalWrite(LED_SOLAR, solarStatus == 1 ? HIGH : LOW);
  digitalWrite(LED_RADS, radiatorStatus == 1 ? HIGH : LOW);
}

// Shift register batching: setLED() only marks bits dirty, the loop writes once
bool shiftRegisterDirty = false;

// DIAGNOSTIC SWITCH. Set to 0 to stop sending throttle/rotation/translation/
// wheel entirely. The controller becomes useless for flying, but it isolates
// whether the dropped packets are caused by what we transmit or purely by
// what we receive. Set back to 1 afterwards.
#define AXIS_SENDS 1

// Axis command rate limit.
// Throttle/rotation/translation/wheel are sent continuously, so they must be
// capped: the loop runs thousands of times per second but the serial link only
// carries ~11.5 kB/s. Sending every loop fills the TX buffer, blocks
// Serial.write, and starves simpit.update() until inbound packets are corrupted.
// 20 ms = 50 Hz, far more than enough for flight control.
const unsigned long AXIS_SEND_INTERVAL = 20;
unsigned long lastAxisSend = 0;

// Loop timing diagnostics (LCD screen 9)
unsigned long loopTimeUs = 0;
unsigned long loopTimeMaxUs = 0;

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

// The mode pots are compared by mode index, not by raw ADC value. A few counts
// of ADC noise on a panel pot used to re-fire these every loop, and each firing
// sent a printToKSP plus a setSASMode, which filled the TX buffer and blocked
// Serial.write for a long time.
int lastSASModeIndex = -1;
int lastControlModeIndex = -1;

// Per-section loop timing, shown on LCD screen 10.
const int SECTION_COUNT = 6;
unsigned long sectionMaxUs[SECTION_COUNT] = {0, 0, 0, 0, 0, 0};
unsigned long sectionStartUs = 0;

int lcdScreenCase = 0;
int lcdScreenCaseBeforeAlarm = 0;
bool lcdAlarmState = false;
bool lcdAlarmStateOverride = false;
bool translationButtonPressed = false;
bool ROCKET_MODE = false;
bool DOCKING_MODE = false;
bool EVA_MODE = false;
bool PRECISION = false;
bool PlANE_MODE = false;
bool ROVER_MODE = false;
bool roverReverseMode = false;  // Rover gearshift: false = forward, true = reverse

// SAS animation variables
bool sasAnimationActive = false;
unsigned long sasAnimationStartTime = 0;
const unsigned long SAS_ANIMATION_DURATION = 3000;  // 3 seconds for animation
int sasAnimationIndex = 0;
unsigned long lastSASAnimationUpdate = 0;
const unsigned long SAS_ANIMATION_STEP = 100;  // Update every 100ms

// Alarm thresholds (ratio of remaining resource)
const float LOW_ELECTRICITY_THRESHOLD = 0.10f;
const float LOW_FUEL_THRESHOLD = 0.10f;  // Uses deltaV as proxy
float maxTotalDeltaVSeen = 0.0f;

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
advancedActionStatusMessage myAdvancedActions;

byte solarStatus = 0;
byte radiatorStatus = 0;
byte scienceStatus = 0;
bool scienceAvailable = false;
bool scienceLedState = false;
unsigned long lastScienceBlink = 0;
const unsigned long SCIENCE_BLINK_INTERVAL = 400;

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
void registerChannels();
bool tryConnect();
void checkConnection(unsigned long now);
void flushLEDs();
unsigned long inboundAgeMs();
void lcdForceRedraw();
void lcdShowNow(const char *a, const char *b);
int getControlModeIndexFromPot(int POT_CONTROL_VALUE);
void sectionBegin();
void sectionEnd(int index);
void handleJoystickButtons(unsigned long now);
void handleSwitches(unsigned long now);
void handleLCDButtons(unsigned long now);
void handleTempAlarm();
void updateLCD();
void sendThrottleCommands();
void sendCameraCommands();
void sendTranslationCommands();
void sendRotationCommands();
void sendWheelCommands();
void updateSASAnimation(unsigned long now);
int getSASModeIndexFromPot(int POT_SAS_VALUE);
void updateAlarmLEDs(unsigned long now);
void messageHandler(byte messageType, byte msg[], byte msgSize);
void sendAdvancedAction(byte actionIndex, ActionGroupSettings setting);
void updateScienceLED(unsigned long now);
void updateAdvancedActionLEDs();


// Setup function
void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud
  Wire.begin();
  
  // Initialize LCD
  lcd.begin(Wire);
  lcd.setFastBacklight(255, 255, 255);
  lcd.createChar(1, deltaChar); // Custom delta glyph, slot 1 (slot 0 would be a NUL in strings)
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
  lcdForceRedraw();
  lcdShowNow("KSP CONTROLLER!", "Ready to connect");

  connectToKSP();
}

// Main loop function
void loop() {
  unsigned long loopStartUs = micros();

  mySimpit.update();
  unsigned long now = millis();

  sectionBegin();
  handleSwitches(now);
  handleButtons(now);
  sectionEnd(0);
  mySimpit.update();

  sectionBegin();
  handleLCDButtons(now);
  handleJoystickButtons(now);
  handleTempAlarm();
  sectionEnd(1);
  mySimpit.update();

  sectionBegin();
  updateSASAnimation(now);
  SAS_mode_pot();
  Control_mode_pot();
  LEDS_ALARM_PANEL();
  updateAlarmLEDs(now);
  updateAdvancedActionLEDs();
  updateScienceLED(now);

  // All LED changes above only touched the state bytes. Write them out once.
  flushLEDs();
  sectionEnd(2);
  mySimpit.update();

  if (now - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    sectionBegin();
    updateLCD();
    sectionEnd(3);
    lastLCDUpdate = now; // Update the last LCD update time
    mySimpit.update();
  }

  // Watchdog: detect a dead link and reconnect without blocking forever
  sectionBegin();
  checkConnection(now);
  sectionEnd(4);

  // Axis commands, rate limited. Camera and EVA keyboard messages are edge
  // triggered elsewhere, so they are not throttled here.
#if AXIS_SENDS
  if (now - lastAxisSend >= AXIS_SEND_INTERVAL) {
    lastAxisSend = now;
    sectionBegin();

    sendRotationCommands();
    sendThrottleCommands();

    // Send either translation or camera commands based on the button state
    if (translationButtonPressed) {
      sendCameraCommands();
    } else {
      sendTranslationCommands();
    }

    // Send wheel commands if in ROVER_MODE
    if (ROVER_MODE) {
      sendWheelCommands();
    }
    sectionEnd(5);
  }
#endif

  unsigned long inboundAge = inboundAgeMs();
  if (inboundAge > ageMaxMs) {
    ageMaxMs = inboundAge;
  }

  loopTimeUs = micros() - loopStartUs;
  if (loopTimeUs > loopTimeMaxUs) {
    loopTimeMaxUs = loopTimeUs;
  }
}

// Loop section timing helpers. Sections do not nest, so one start stamp is enough.
void sectionBegin() {
  sectionStartUs = micros();
}

void sectionEnd(int index) {
  unsigned long elapsed = micros() - sectionStartUs;
  if (elapsed > sectionMaxUs[index]) {
    sectionMaxUs[index] = elapsed;
  }
}

// Subscribe to every channel this controller actually reads in messageHandler()
void registerChannels() {
  mySimpit.registerChannel(AIRSPEED_MESSAGE);
  mySimpit.registerChannel(ALTITUDE_MESSAGE);
  mySimpit.registerChannel(VELOCITY_MESSAGE);
  mySimpit.registerChannel(ROTATION_DATA_MESSAGE);
  mySimpit.registerChannel(TEMP_LIMIT_MESSAGE);
  mySimpit.registerChannel(DELTAV_MESSAGE);
  mySimpit.registerChannel(ATMO_CONDITIONS_MESSAGE);
  mySimpit.registerChannel(ELECTRIC_MESSAGE);
  mySimpit.registerChannel(FLIGHT_STATUS_MESSAGE);
  mySimpit.registerChannel(ADVANCED_ACTIONSTATUS_MESSAGE);
}

// One handshake attempt. init() blocks up to ~1.1s when KSP does not answer.
bool tryConnect() {
  if (!mySimpit.init()) {
    return false;
  }

  isConnected = true;
  lastInboundMessageTime = millis();
  lastHeartbeatSent = millis();

  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  mySimpit.inboundHandler(messageHandler);
  registerChannels();

  lcd.clear();
  lcdShowNow("CONNECTED!", "");
  return true;
}

// Function to connect to Kerbal Space Program (blocking, used at startup only)
void connectToKSP() {
  while (!tryConnect()) {
    delay(100);
  }
}

// Watchdog: KSP answers echo requests in any scene, so a heartbeat every couple
// of seconds should keep traffic flowing even when no flight data is streaming.
// The timeout only counts once an echo has actually come back at least once.
void checkConnection(unsigned long now) {
  if (isConnected) {
    if (now - lastHeartbeatSent >= HEARTBEAT_INTERVAL) {
      lastHeartbeatSent = now;
      byte ping = 0;
      mySimpit.send(ECHO_REQ_MESSAGE, ping);
    }

#if LINK_WATCHDOG
    // Only conclude the link is dead if echo is proven to work. Without a
    // working echo, silence means nothing: most Simpit channels only send
    // inside the flight scene, and some only on change.
    if (echoSupported && inboundAgeMs() > CONNECTION_TIMEOUT) {
      isConnected = false;
      lastReconnectAttempt = now;
    }
#endif
    return;
  }

  // Disconnected: retry the handshake at a fixed interval. Nothing else can
  // work anyway, so the ~1.1s init() timeout is acceptable here.
  if (now - lastReconnectAttempt >= RECONNECT_INTERVAL) {
    lastReconnectAttempt = now;
    tryConnect();
  }
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
      sendAdvancedAction(ADVANCED_SOLAR_ACTION, AG_ACTION_DEACTIVATE);
      digitalWrite(LED_SOLAR, LOW);
    } else {  // Button released

      sendAdvancedAction(ADVANCED_SOLAR_ACTION, AG_ACTION_ACTIVATE);
      digitalWrite(LED_SOLAR, HIGH);
    }

    // Update the last state
    lastSolarSwitchState = readingSolarSwitch;
  }



  // Handle RADS Switch
  if (readingRADSSwitch != lastRADSSwitchState && (now - lastDebounceTimeRADSSwitch) > DEBOUNCE_DELAY) {
    lastDebounceTimeRADSSwitch = now;

    if (readingRADSSwitch == LOW) {  // Button pressed
      
      sendAdvancedAction(ADVANCED_RADIATOR_ACTION, AG_ACTION_DEACTIVATE);
      digitalWrite(LED_RADS, LOW);

    } else {  // Button released
     sendAdvancedAction(ADVANCED_RADIATOR_ACTION, AG_ACTION_ACTIVATE);
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


    if (readingSASSwitch == LOW) {  // Button pressed - SAS OFF
      
    mySimpit.deactivateAction(SAS_ACTION);
    mySimpit.printToKSP("SAS Deactivated", PRINT_TO_SCREEN);
    setLED(LED_SAS, false);
    
    // Clear all SAS mode LEDs
    clearSASModeLEDs();
    sasAnimationActive = false;

    } else {  // Button released - SAS ON
    mySimpit.activateAction(SAS_ACTION);
    mySimpit.printToKSP("SAS Activated", PRINT_TO_SCREEN);
    setLED(LED_SAS, true);
    
    // Start animation
    sasAnimationActive = true;
    sasAnimationStartTime = now;
    sasAnimationIndex = 0;

 
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
      if (readingJoystickButtonRotation == LOW && lastRotationButtonState == HIGH) {
        // Button just pressed (transition from HIGH to LOW)
        if (ROVER_MODE) {
          // In rover mode: toggle direction (gearshift forward/reverse)
          roverReverseMode = !roverReverseMode;
          if (roverReverseMode) {
            mySimpit.printToKSP(F("Rover: REVERSE"), PRINT_TO_SCREEN);
          } else {
            mySimpit.printToKSP(F("Rover: FORWARD"), PRINT_TO_SCREEN);
          }
          lastDebounceTimeJoystickRotation = now;
        } else if (EVA_MODE) {
          // Check if this is a double press within the timeout window
          if ((now - lastDoubleRPressTime) < DOUBLE_PRESS_TIMEOUT) {
            // Double press detected: Send R R (quick double tap)
            keyboardEmulatorMessage rMsg(EVA_R_KEY, KEY_DOWN_MOD);
            mySimpit.send(KEYBOARD_EMULATOR, rMsg);
            delay(50);  // Brief press
            keyboardEmulatorMessage rMsgUp(EVA_R_KEY, KEY_UP_MOD);
            mySimpit.send(KEYBOARD_EMULATOR, rMsgUp);
            delay(50);  // Brief gap
            keyboardEmulatorMessage rMsg2(EVA_R_KEY, KEY_DOWN_MOD);
            mySimpit.send(KEYBOARD_EMULATOR, rMsg2);
            delay(50);
            keyboardEmulatorMessage rMsg2Up(EVA_R_KEY, KEY_UP_MOD);
            mySimpit.send(KEYBOARD_EMULATOR, rMsg2Up);
            lastDoubleRPressTime = 0;  // Reset timer after double press
            rotationButtonHeld = false;  // Don't hold SHIFT+SPACE
          } else {
            // First press: Record the time and wait to see if it's a double-press
            lastDoubleRPressTime = now;
            rotationButtonHeld = true;  // Mark that button is held, will activate SHIFT+SPACE on timeout
          }
        } else {
          // In other modes: Stage action
          if (StageArmend == true) {
            mySimpit.activateAction(STAGE_ACTION);
          }
        }
        lastDebounceTimeJoystickRotation = now;
      } else if (readingJoystickButtonRotation == LOW && rotationButtonHeld && EVA_MODE) {
        // Button still held - check if we've exceeded the double-press timeout
        if ((now - lastDoubleRPressTime) >= DOUBLE_PRESS_TIMEOUT && !eva_shift_space_pressed) {
          // Timeout reached, activate SHIFT+SPACE hold
          keyboardEmulatorMessage shiftMsg(EVA_SHIFT_KEY, KEY_DOWN_MOD);
          mySimpit.send(KEYBOARD_EMULATOR, shiftMsg);
          keyboardEmulatorMessage spaceMsg(EVA_SPACE_KEY, KEY_DOWN_MOD);
          mySimpit.send(KEYBOARD_EMULATOR, spaceMsg);
          eva_shift_space_pressed = true;
        }
      } else if (readingJoystickButtonRotation == HIGH && lastRotationButtonState == LOW) {
        // Button just released (transition from LOW to HIGH)
        if (eva_shift_space_pressed && EVA_MODE) {
          // Release SHIFT+SPACE
          keyboardEmulatorMessage shiftMsg(EVA_SHIFT_KEY, KEY_UP_MOD);
          mySimpit.send(KEYBOARD_EMULATOR, shiftMsg);
          keyboardEmulatorMessage spaceMsg(EVA_SPACE_KEY, KEY_UP_MOD);
          mySimpit.send(KEYBOARD_EMULATOR, spaceMsg);
          eva_shift_space_pressed = false;
        }
        rotationButtonHeld = false;
      }
      lastRotationButtonState = readingJoystickButtonRotation;
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
      sendAdvancedAction(ADVANCED_SCIENCE_ACTION, AG_ACTION_TOGGLE);
     
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
    if (lcdScreenCase < 10) {
      lcdScreenCase++;
      lcdScreenCaseBeforeAlarm = lcdScreenCase;
      if (lcdScreenCase == 9) {
        loopTimeMaxUs = 0;  // Fresh peak measurement each time you open the screen
      }
      if (lcdScreenCase == 10) {
        for (int i = 0; i < SECTION_COUNT; i++) {
          sectionMaxUs[i] = 0;  // Fresh per-section measurement
        }
      }
      lcdForceRedraw(); // full 32 char redraw overwrites the old screen, no clear needed
      if (lcdAlarmState) {
        lcdAlarmStateOverride = false;
        lcdAlarmState = false;
        setLED(LED_MASTER_ALARM, false);
        setLED(LED_TEMPRATURE, false);
        lcdScreenCase = 0;
        lcdForceRedraw();
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
      lcdForceRedraw(); // full 32 char redraw overwrites the old screen, no clear needed
      if (lcdAlarmState) {
        lcdAlarmStateOverride = true;
        lcdAlarmState = false;
        setLED(LED_MASTER_ALARM, false);
        setLED(LED_TEMPRATURE, false);
        lcdScreenCase = 0;
        lcdForceRedraw();
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
      lcdForceRedraw();
    }
  } else {
    if (lcdAlarmState) {
      lcdAlarmState = false;
      lcdAlarmStateOverride = false;
      lcdScreenCase = lcdScreenCaseBeforeAlarm;\
      setLED(LED_MASTER_ALARM, false);
      setLED(LED_TEMPRATURE, false);
      lcd.setBacklight(255, 255, 255);
      lcdForceRedraw();
    }
  }
}

// LCD rendering.
// The SerLCD library blocks: every print() ends with delay(10) and every
// setCursor() with delay(50). A screen built from six print calls therefore
// costs well over 100ms of frozen loop. So both rows are composed into one
// 32 character buffer, sent with a single print, and skipped entirely when
// nothing changed since the last redraw.
char lcdBuf[33];
char lcdShown[33];
char lcdRowA[17];
char lcdRowB[17];
char lcdNumA[16];
char lcdNumB[16];

// Force the next updateLCD() to redraw, e.g. after lcd.clear() wiped the panel
void lcdForceRedraw() {
  lcdShown[0] = '\0';
}

void lcdSetLines(const char *a, const char *b) {
  snprintf(lcdBuf, sizeof(lcdBuf), "%-16.16s%-16.16s", a, b);
}

// The SerLCD wraps text from the end of row 0 to the start of row 1, and from
// the end of row 1 back to the start of row 0. Writing exactly 32 characters
// therefore leaves the cursor where it began and setCursor() is never needed,
// which saves its 50ms delay. Every write in this sketch goes through here, so
// the cursor stays on that 32 character grid.
// Set to 0 if the display turns out not to wrap: then each redraw costs 50ms
// more but the position is set explicitly.
#define LCD_ASSUME_WRAP 1

void lcdCommit() {
  if (strcmp(lcdBuf, lcdShown) == 0) {
    return;  // Nothing changed, no I2C traffic and no library delays at all
  }
  strcpy(lcdShown, lcdBuf);
#if !LCD_ASSUME_WRAP
  lcd.setCursor(0, 0);
#endif
  lcd.print(lcdBuf);
}

// Write a screen immediately, bypassing the change check. Used for the boot and
// connection messages so they also land on the 32 character grid.
void lcdShowNow(const char *a, const char *b) {
  lcdSetLines(a, b);
  lcdForceRedraw();
  lcdCommit();
}

// Function to update LCD display
void updateLCD() {
  switch (lcdScreenCase) {
    case 0:
      dtostrf(myAirspeed.mach, 0, 2, lcdNumA);
      snprintf(lcdRowA, sizeof(lcdRowA), "MACH: %s", lcdNumA);
      snprintf(lcdRowB, sizeof(lcdRowB), "Airspeed: %ld", (long)round(myAirspeed.IAS));
      break;
    case 1:
      snprintf(lcdRowA, sizeof(lcdRowA), "Sealevel: %ld", (long)round(myAltitude.sealevel));
      snprintf(lcdRowB, sizeof(lcdRowB), "Surface: %ld", (long)round(myAltitude.surface));
      break;
    case 2:
      snprintf(lcdRowA, sizeof(lcdRowA), "m/s: %ld", (long)round(myVelocity.surface));
      snprintf(lcdRowB, sizeof(lcdRowB), "km/h: %ld", (long)round(myVelocity.surface * 3.6));
      break;
    case 3:
      dtostrf(myRotation.heading, 0, 2, lcdNumA);
      dtostrf(myRotation.pitch, 0, 2, lcdNumB);
      snprintf(lcdRowA, sizeof(lcdRowA), "Heading: %s", lcdNumA);
      snprintf(lcdRowB, sizeof(lcdRowB), "Pitch: %s", lcdNumB);
      break;
    case 4:
      snprintf(lcdRowA, sizeof(lcdRowA), "Part Temp %%%d", myTemplimits.tempLimitPercentage);
      snprintf(lcdRowB, sizeof(lcdRowB), "Skin Temp %%%d", myTemplimits.skinTempLimitPercentage);
      break;
    case 5:
      // \x01 is the custom delta glyph, registered as character 1 in setup().
      // It cannot be character 0, because a zero byte would end the string.
      snprintf(lcdRowA, sizeof(lcdRowA), "\x01V Stage %ld", (long)round(myDeltaV.stageDeltaV));
      snprintf(lcdRowB, sizeof(lcdRowB), "\x01V Ship %ld", (long)round(myDeltaV.totalDeltaV));
      break;
    case 6:
      dtostrf(myAtmoConditions.temperature - 273.15, 0, 2, lcdNumA);
      dtostrf(myAtmoConditions.airDensity, 0, 2, lcdNumB);
      snprintf(lcdRowA, sizeof(lcdRowA), "Air Temp %s", lcdNumA);
      snprintf(lcdRowB, sizeof(lcdRowB), "Air Dens %s", lcdNumB);
      break;
    case 7:
      dtostrf(myAtmoConditions.pressure, 0, 2, lcdNumA);
      dtostrf(myAirspeed.gForces, 0, 2, lcdNumB);
      snprintf(lcdRowA, sizeof(lcdRowA), "Air Pres %s", lcdNumA);
      snprintf(lcdRowB, sizeof(lcdRowB), "G-Forces %s", lcdNumB);
      break;
    case 8:
      dtostrf(myElectric.available, 0, 2, lcdNumA);
      snprintf(lcdRowA, sizeof(lcdRowA), "Power %s", lcdNumA);
      lcdRowB[0] = '\0';
      break;
    case 9:
      // D       : corrupted inbound packets since boot.
      // L       : peak loop time in us since this screen was opened.
      // Age a/b : seconds since last inbound packet / largest gap ever seen.
      // E       : does KSP answer echo heartbeats?
      snprintf(lcdRowA, sizeof(lcdRowA), "D:%u L:%lu",
               mySimpit.packetDroppedNbr, loopTimeMaxUs);
      snprintf(lcdRowB, sizeof(lcdRowB), "Age:%lu/%lus E:%c",
               inboundAgeMs() / 1000, ageMaxMs / 1000, echoSupported ? 'Y' : 'N');
      break;
    case 10: {
      // Which block of the loop is the slow one.
      // S0 switches+buttons  S1 lcd buttons+joystick+temp alarm
      // S2 pots+LEDs         S3 LCD redraw
      // S4 connection        S5 axis sends
      int worst = 0;
      for (int i = 1; i < SECTION_COUNT; i++) {
        if (sectionMaxUs[i] > sectionMaxUs[worst]) {
          worst = i;
        }
      }
      snprintf(lcdRowA, sizeof(lcdRowA), "Slowest: S%d", worst);
      snprintf(lcdRowB, sizeof(lcdRowB), "%luus", sectionMaxUs[worst]);
    } break;
    case 98:
      snprintf(lcdRowA, sizeof(lcdRowA), "PART TEMP!: %d", myTemplimits.tempLimitPercentage);
      snprintf(lcdRowB, sizeof(lcdRowB), "SKIN TEMP!: %d", myTemplimits.skinTempLimitPercentage);
      break;
    default:
      return;
  }

  lcdSetLines(lcdRowA, lcdRowB);
  lcdCommit();
}

// Function to send throttle commands
void sendThrottleCommands() {
  throttleMessage throttleMsg;
  int reading = analogRead(THROTTLE_PIN);

  int16_t throttlePercentage = 0; // Calculate throttle in 0-100%

// Interpolate the throttle percentage based on observed ranges
if (reading >= 980) {
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
  // Check if EVA_MODE is active
  if (EVA_MODE) {
    // EVA Mode: Use joystick to control WASD keys and QE for yaw
    // Pitch (Y-axis): Up = S (forward), Down = W (backward)
    // Roll (X-axis): Left = A (left), Right = D (right)
    // Yaw (Z-axis): Left = Q, Right = E
    
    int readingPitch = analogRead(PITCH_PIN);
    int readingRoll = analogRead(ROLL_PIN);
    int readingYaw = analogRead(YAW_PIN);

    // Add trim values
    readingPitch += readingPitchTrim;
    readingPitch = constrain(readingPitch, 0, 1023);
    readingRoll += readingRollTrim;
    readingRoll = constrain(readingRoll, 0, 1023);
    readingYaw += readingYawTrim;
    readingYaw = constrain(readingYaw, 0, 1023);

    // Handle S key (Forward - Pitch Up)
    if (readingPitch > (512 + DEADZONE) && !eva_s_pressed) {
      keyboardEmulatorMessage sMsg(EVA_S_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, sMsg);
      eva_s_pressed = true;
    } else if (readingPitch <= (512 + DEADZONE) && eva_s_pressed) {
      keyboardEmulatorMessage sMsg(EVA_S_KEY, KEY_UP_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, sMsg);
      eva_s_pressed = false;
    }

    // Handle W key (Backward - Pitch Down)
    if (readingPitch < (512 - DEADZONE) && !eva_w_pressed) {
      keyboardEmulatorMessage wMsg(EVA_W_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, wMsg);
      eva_w_pressed = true;
    } else if (readingPitch >= (512 - DEADZONE) && eva_w_pressed) {
      keyboardEmulatorMessage wMsg(EVA_W_KEY, KEY_UP_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, wMsg);
      eva_w_pressed = false;
    }

    // Handle A key (Left - Roll Left)
    if (readingRoll < (512 - DEADZONE) && !eva_a_pressed) {
      keyboardEmulatorMessage aMsg(EVA_A_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, aMsg);
      eva_a_pressed = true;
    } else if (readingRoll >= (512 - DEADZONE) && eva_a_pressed) {
      keyboardEmulatorMessage aMsg(EVA_A_KEY, KEY_UP_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, aMsg);
      eva_a_pressed = false;
    }

    // Handle D key (Right - Roll Right)
    if (readingRoll > (512 + DEADZONE) && !eva_d_pressed) {
      keyboardEmulatorMessage dMsg(EVA_D_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, dMsg);
      eva_d_pressed = true;
    } else if (readingRoll <= (512 + DEADZONE) && eva_d_pressed) {
      keyboardEmulatorMessage dMsg(EVA_D_KEY, KEY_UP_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, dMsg);
      eva_d_pressed = false;
    }

    // Handle Q key (Yaw Left)
    if (readingYaw < (512 - DEADZONE) && !eva_q_pressed) {
      keyboardEmulatorMessage qMsg(EVA_Q_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, qMsg);
      eva_q_pressed = true;
    } else if (readingYaw >= (512 - DEADZONE) && eva_q_pressed) {
      keyboardEmulatorMessage qMsg(EVA_Q_KEY, KEY_UP_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, qMsg);
      eva_q_pressed = false;
    }

    // Handle E key (Yaw Right)
    if (readingYaw > (512 + DEADZONE) && !eva_e_pressed) {
      keyboardEmulatorMessage eMsg(EVA_E_KEY, KEY_DOWN_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, eMsg);
      eva_e_pressed = true;
    } else if (readingYaw <= (512 + DEADZONE) && eva_e_pressed) {
      keyboardEmulatorMessage eMsg(EVA_E_KEY, KEY_UP_MOD);
      mySimpit.send(KEYBOARD_EMULATOR, eMsg);
      eva_e_pressed = false;
    }

  } else {
    // PLANE_MODE and other modes: Use rotation commands
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
    if (ROVER_MODE || ROCKET_MODE) {
      // In rover/rocket mode, use YAW axis for roll rotation
      if (readingYaw > (512 + DEADZONE)) {
        roll = map(readingYaw, 512 + DEADZONE, 1023, 0, INT16_MAX);
      } else if (readingYaw < (512 - DEADZONE)) {
        roll = map(readingYaw, 0, 512 - DEADZONE, INT16_MIN, 0);
      }
    } else {
      // Normal mode: use ROLL axis for roll
      if (readingRoll > (512 + DEADZONE)) {
        roll = map(readingRoll, 512 + DEADZONE, 1023, 0, INT16_MAX);
      } else if (readingRoll < (512 - DEADZONE)) {
        roll = map(readingRoll, 0, 512 - DEADZONE, INT16_MIN, 0);
      }
    }

    int16_t yaw = 0;
    // In rover/rocket mode, use roll axis for yaw rotation. Otherwise use yaw axis.
    if (ROVER_MODE || ROCKET_MODE) {
      if (readingRoll > (512 + DEADZONE)) {
        yaw = map(readingRoll, 512 + DEADZONE, 1023, 0, INT16_MAX);
      } else if (readingRoll < (512 - DEADZONE)) {
        yaw = map(readingRoll, 0, 512 - DEADZONE, INT16_MIN, 0);
      }
    } else if (readingYaw > (512 + DEADZONE)) {
      yaw = map(readingYaw, 512 + DEADZONE, 1023, 0, INT16_MAX);
    } else if (readingYaw < (512 - DEADZONE)) {
      yaw = map(readingYaw, 0, 512 - DEADZONE, INT16_MIN, 0);
    }

    rotMsg.setPitch(pitch);
    rotMsg.setRoll(roll);
    rotMsg.setYaw(yaw);
    mySimpit.send(ROTATION_MESSAGE, rotMsg);
  }
}

// Send wheel commands for rover mode
void sendWheelCommands() {
  wheelMessage wheelMsg;
  int readingRoll = analogRead(ROLL_PIN);

  // Add trim value for roll
  readingRoll += readingRollTrim;
  readingRoll = constrain(readingRoll, 0, 1023);

  // Map roll input to wheel steering
  int16_t steer = 0;
  if (readingRoll > (512 + DEADZONE)) {
    steer = map(readingRoll, 512 + DEADZONE, 1023, 0, INT16_MIN);
  } else if (readingRoll < (512 - DEADZONE)) {
    steer = map(readingRoll, 0, 512 - DEADZONE, INT16_MAX, 0);
  }

  // Read throttle input for rover with gearshift
  int readingThrottle = analogRead(THROTTLE_PIN);
  // Map throttle input based on direction (forward or reverse)
  // Throttle goes from INT16_MAX/INT16_MIN (full throttle) to 0 (no throttle)
  int16_t throttle = 0;
  if (roverReverseMode) {
    // Reverse: 0 = full reverse (INT16_MIN), 1023 = no throttle
    throttle = map(readingThrottle, 0, 1023, INT16_MIN, 0);
  } else {
    // Forward: 0 = full forward (INT16_MAX), 1023 = no throttle
    throttle = map(readingThrottle, 0, 1023, INT16_MAX, 0);
  }

  // Set steering and throttle for rover
  wheelMsg.setSteer(steer);
  wheelMsg.setThrottle(throttle);
  mySimpit.send(WHEEL_MESSAGE, wheelMsg);
}

// Message handler for Kerbal Simpit
void messageHandler(byte messageType, byte msg[], byte msgSize) {
  // Any packet at all proves the link is alive (this includes ECHO_RESP_MESSAGE)
  lastInboundMessageTime = millis();

  switch (messageType) {
    case ECHO_RESP_MESSAGE:
      // KSP answered a heartbeat, so the watchdog can be trusted from here on
      echoSupported = true;
      break;

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
        if (myDeltaV.totalDeltaV > maxTotalDeltaVSeen) {
          maxTotalDeltaVSeen = myDeltaV.totalDeltaV;
        }
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

    case FLIGHT_STATUS_MESSAGE: {
        if (msgSize == sizeof(flightStatusMessage)) {
          myFlightStatus = parseMessage<flightStatusMessage>(msg);
          LEDS_ALARM_PANEL();
        }
      } break;

    case ADVANCED_ACTIONSTATUS_MESSAGE:
      if (msgSize == sizeof(advancedActionStatusMessage)) {
        myAdvancedActions = parseMessage<advancedActionStatusMessage>(msg);
        solarStatus = myAdvancedActions.getActionStatus(ADVANCED_SOLAR_ACTION);
        radiatorStatus = myAdvancedActions.getActionStatus(ADVANCED_RADIATOR_ACTION);
        scienceStatus = myAdvancedActions.getActionStatus(ADVANCED_SCIENCE_ACTION);
        scienceAvailable = (scienceStatus == 1);
      }
      break;
  }
  
}

void SAS_mode_pot() {
  // Don't update SAS mode while animation is playing
  if (sasAnimationActive) {
    return;
  }

  // Read the potentiometer value (0 to 1023)
  int POT_SAS_VALUE = analogRead(POT_SAS_PIN);

  // Act on a change of mode, not on a change of raw value. ADC noise of a few
  // counts would otherwise re-send setSASMode every single loop.
  int index = getSASModeIndexFromPot(POT_SAS_VALUE);
  if (index != lastSASModeIndex) {
    lastSASModeIndex = index;
    LastSASModePotValue = POT_SAS_VALUE;

    // Clear only the LEDs used in this function
    clearSASModeLEDs();
    setSASModeLED(POT_SAS_VALUE);
  }
}

// Helper function to set SAS mode LED based on potentiometer value
void setSASModeLED(int POT_SAS_VALUE) {
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

// Helper function to map potentiometer value to SAS mode LED index
int getSASModeIndexFromPot(int POT_SAS_VALUE) {
  if (POT_SAS_VALUE >= 0 && POT_SAS_VALUE < 10) {
    return 0;  // LED_AUTO_PILOT
  } else if (POT_SAS_VALUE >= 10 && POT_SAS_VALUE < 95) {
    return 1;  // LED_ANTI_NORMAL
  } else if (POT_SAS_VALUE >= 95 && POT_SAS_VALUE < 195) {
    return 2;  // LED_NORMAL
  } else if (POT_SAS_VALUE >= 195 && POT_SAS_VALUE < 310) {
    return 3;  // LED_RETRO_GRADE
  } else if (POT_SAS_VALUE >= 310 && POT_SAS_VALUE < 420) {
    return 4;  // LED_PRO_GRADE
  } else if (POT_SAS_VALUE >= 420 && POT_SAS_VALUE < 507) {
    return 5;  // LED_MANAUVER
  } else if (POT_SAS_VALUE >= 507 && POT_SAS_VALUE < 600) {
    return 6;  // LED_STABILITY_ASSIST
  } else if (POT_SAS_VALUE >= 600 && POT_SAS_VALUE < 690) {
    return 7;  // LED_RADIAL_OUT
  } else if (POT_SAS_VALUE >= 690 && POT_SAS_VALUE < 790) {
    return 8;  // LED_RADIAL_IN
  } else if (POT_SAS_VALUE >= 790 && POT_SAS_VALUE < 875) {
    return 9;  // LED_TARGET
  } else if (POT_SAS_VALUE >= 875 && POT_SAS_VALUE < 975) {
    return 10; // LED_ANTI_TARGET
  } else {
    return 11; // LED_NAVIGATION
  }
}

// Show current SAS mode (used after animation ends)
void showCurrentSASMode() {
  int POT_SAS_VALUE = analogRead(POT_SAS_PIN);
  clearSASModeLEDs();
  setSASModeLED(POT_SAS_VALUE);
}

// Maps the control pot to a mode index. Index 5 is the gap between 850 and 919,
// which deliberately selects no mode and clears the LEDs.
int getControlModeIndexFromPot(int POT_CONTROL_VALUE) {
  if (POT_CONTROL_VALUE < 170) return 0;
  if (POT_CONTROL_VALUE < 340) return 1;
  if (POT_CONTROL_VALUE < 510) return 2;
  if (POT_CONTROL_VALUE < 680) return 3;
  if (POT_CONTROL_VALUE < 850) return 4;
  if (POT_CONTROL_VALUE < 920) return 5;
  return 6;
}

void Control_mode_pot() {
  // Read the potentiometer value (0 to 1023)
  int POT_CONTROL_VALUE = analogRead(POT_CONTROL_PIN);

  // Act on a change of mode, not on a change of raw value
  int index = getControlModeIndexFromPot(POT_CONTROL_VALUE);
  if (index != lastControlModeIndex) {
    lastControlModeIndex = index;
    LastControlModePotValue = POT_CONTROL_VALUE;
    
    // Clear only the LEDs used in this function
    clearControlModeLEDs();

    // Define custom potentiometer ranges for each LED and turn them on accordingly
    if (POT_CONTROL_VALUE >= 0 && POT_CONTROL_VALUE < 170) {
      ROCKET_MODE = false;
      DOCKING_MODE = false;
      EVA_MODE = false;
      PRECISION = false;
      PlANE_MODE = true;
      ROVER_MODE = false;
      roverReverseMode = false;  // Reset when exiting rover mode
      setLED(LED_PlANE_MODE, true);

    } else if (POT_CONTROL_VALUE >= 170 && POT_CONTROL_VALUE < 340) {
      setLED(LED_ROVER_MODE, true);
      ROCKET_MODE = false;
      DOCKING_MODE = false;
      EVA_MODE = false;
      PRECISION = false;
      PlANE_MODE = true;
      ROVER_MODE = true;
          roverReverseMode = false;  // Reset to forward when entering rover mode
 
    } else if (POT_CONTROL_VALUE >= 340 && POT_CONTROL_VALUE < 510) {
      setLED(LED_ROCKET_MODE, true);
      ROCKET_MODE = true;
      DOCKING_MODE = false;
      EVA_MODE = false;
      PRECISION = false;
      PlANE_MODE = true;
      ROVER_MODE = false;
      roverReverseMode = false;  // Reset when exiting rover mode

    } else if (POT_CONTROL_VALUE >= 510 && POT_CONTROL_VALUE < 680) {
      
      setLED(LED_PRECISION, true);
      ROCKET_MODE = false;
      DOCKING_MODE = false;
      EVA_MODE = false;
      PRECISION = true;
      PlANE_MODE = true;
      ROVER_MODE = false;
      roverReverseMode = false;  // Reset when exiting rover mode

    } else if (POT_CONTROL_VALUE >= 680 && POT_CONTROL_VALUE < 850) {
      
      setLED(LED_EVA_MODE, true);
      ROCKET_MODE = false;
      DOCKING_MODE = false;
      EVA_MODE = true;
      PRECISION = false;
      PlANE_MODE = true;
      ROVER_MODE = false;
      roverReverseMode = false;  // Reset when exiting rover mode
      
     
    } else if (POT_CONTROL_VALUE >= 920 && POT_CONTROL_VALUE < 1024) {
      
      setLED(LED_DOCKING_MODE, true);
      ROCKET_MODE = false;
      DOCKING_MODE = true;
      EVA_MODE = false;
      PRECISION = false;
      PlANE_MODE = true;
      ROVER_MODE = false;
      roverReverseMode = false;  // Reset when exiting rover mode
      
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
// Sets a single LED bit. Does NOT touch the hardware: it only marks the
// shift registers dirty so the loop can write all 32 bits out in one go.
void setLED(int led, bool state) {
  byte *target;
  int bit;

  if (led < 8) {           // First shift register (LEDs 0-7)
    target = &ledStates1;
    bit = led;
  } else if (led < 16) {   // Second shift register (LEDs 8-15)
    target = &ledStates2;
    bit = led - 8;
  } else if (led < 24) {   // Third shift register (LEDs 16-23)
    target = &ledStates3;
    bit = led - 16;
  } else {                 // Fourth shift register (LEDs 24-31)
    target = &ledStates4;
    bit = led - 24;
  }

  byte before = *target;
  if (state) {
    *target |= (1 << bit);   // Set bit to 1 (turn on)
  } else {
    *target &= ~(1 << bit);  // Set bit to 0 (turn off)
  }

  // Only a real change is worth clocking out
  if (*target != before) {
    shiftRegisterDirty = true;
  }
}

// Write the LED states to the hardware, but only if something changed.
// Call this once per loop, after all setLED() calls.
void flushLEDs() {
  if (!shiftRegisterDirty) {
    return;
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
  shiftRegisterDirty = false;
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

// Animate alarm LEDs without blocking
void updateAlarmLEDs(unsigned long now) {
  bool tempAlarmActive = (myTemplimits.skinTempLimitPercentage > 40 || myTemplimits.tempLimitPercentage > 40);

  bool lowElectricActive = false;
  if (myElectric.total > 0) {
    float electricRatio = myElectric.available / myElectric.total;
    lowElectricActive = (electricRatio <= LOW_ELECTRICITY_THRESHOLD);
  }

  bool lowFuelActive = false;
  if (maxTotalDeltaVSeen > 0) {
    float fuelRatio = myDeltaV.totalDeltaV / maxTotalDeltaVSeen;
    lowFuelActive = (fuelRatio <= LOW_FUEL_THRESHOLD);
  }

  bool anyAlarm = tempAlarmActive || lowElectricActive || lowFuelActive;

  if (anyAlarm) {
    bool masterOn = ((now / 250) % 2) == 0;
    setLED(LED_MASTER_ALARM, masterOn);
  } else {
    setLED(LED_MASTER_ALARM, false);
  }

  if (tempAlarmActive) {
    unsigned long phase = now % 1000;
    bool tempOn = (phase < 120) || (phase >= 200 && phase < 320);
    setLED(LED_TEMPRATURE, tempOn);
  } else {
    setLED(LED_TEMPRATURE, false);
  }

  if (lowElectricActive) {
    bool elecOn = ((now / 500) % 2) == 0;
    setLED(LED_LOW_ELECTRICITY, elecOn);
  } else {
    setLED(LED_LOW_ELECTRICITY, false);
  }

  if (lowFuelActive) {
    bool fuelOn = ((now / 700) % 2) == 0;
    setLED(LED_LOW_FUEL, fuelOn);
  } else {
    setLED(LED_LOW_FUEL, false);
  }
}

// SAS Mode LEDs array (in order for wave animation)
const int SASmodeCount = 12;
const int SASmodeLEDs[SASmodeCount] = {
  LED_AUTO_PILOT,
  LED_ANTI_NORMAL,
  LED_NORMAL,
  LED_RETRO_GRADE,
  LED_PRO_GRADE,
  LED_MANAUVER,
  LED_STABILITY_ASSIST,
  LED_RADIAL_OUT,
  LED_RADIAL_IN,
  LED_TARGET,
  LED_ANTI_TARGET,
  LED_NAVIGATION
};

// Update SAS animation - wave effect when SAS is activated
void updateSASAnimation(unsigned long now) {
  if (!sasAnimationActive) {
    return;
  }

  unsigned long elapsedTime = now - sasAnimationStartTime;

  // Check if animation should continue or end
  if (elapsedTime > SAS_ANIMATION_DURATION) {
    // Animation complete, turn off animation and show selected SAS mode
    sasAnimationActive = false;
    clearSASModeLEDs();
    showCurrentSASMode();  // Show the currently selected SAS mode LED
    return;
  }

  // Update animation LEDs based on elapsed time
  if (now - lastSASAnimationUpdate >= SAS_ANIMATION_STEP) {
    lastSASAnimationUpdate = now;

    // Calculate animation progress (0.0 to 1.0)
    float progress = (float)elapsedTime / SAS_ANIMATION_DURATION;

    // Clear all SAS mode LEDs first
    clearSASModeLEDs();

    // Wave animation: smooth sweep from left to right and back
    float wavePosition;
    
    if (progress < 0.5) {
      // Left to right: progress 0.0-0.5 maps to position 0.0-1.0
      wavePosition = progress * 2.0;
    } else {
      // Right to left: progress 0.5-1.0 maps to position 1.0-0.0
      wavePosition = (1.0 - progress) * 2.0;
    }

    // Calculate which LEDs should be lit
    // wavePosition goes from 0.0 to 1.0
    int ledIndex = (int)(wavePosition * (SASmodeCount - 1));
    ledIndex = constrain(ledIndex, 0, SASmodeCount - 1);

    // Always keep the selected mode LED on during the wave
    int targetIndex = getSASModeIndexFromPot(analogRead(POT_SAS_PIN));

    // Light up LEDs up to wave position (smooth fade effect)
    for (int i = 0; i <= ledIndex; i++) {
      setLED(SASmodeLEDs[i], true);
    }

    // Ensure selected mode stays on even when the wave passes it
    setLED(SASmodeLEDs[targetIndex], true);
  }
}
