#include <KerbalSimpit.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SerLCD.h>
#include <PayloadStructs.h>

// Constants
const int THROTTLE_PIN = A0;       // the pin used for controlling throttle
const int PITCH_PIN = A1;          // the pin used for controlling pitch
const int ROLL_PIN = A2;           // the pin used for controlling roll
const int YAW_PIN = A3;            // the pin used for controlling yaw
const int TRANSLATE_X_PIN = A5;    // the pin used for controlling translation X
const int TRANSLATE_Y_PIN = A6;    // the pin used for controlling translation Y
const int TRANSLATE_Z_PIN = A4;    // the pin used for controlling translation Z
const int JOYSTICK_BUTTON_TRANSLATION = 5;
const int JOYSTICK_BUTTON_ROTATION = 4;
const int LCD_SWITCH_PIN_LEFT = 2;
const int LCD_SWITCH_PIN_RIGHT = 3;
const unsigned long DEBOUNCE_DELAY = 50; // Debounce delay in milliseconds
const unsigned int LCD_UPDATE_INTERVAL = 125;  // LCD update frequency
const unsigned int SEND_INTERVAL = 1500;
const int DEADZONE = 20; // Deadzone for joystick inputs
const int SMALL_INCREMENT = 1000; // Adjust this camera responsivenes value as needed
int16_t prevPitch = 0;
int16_t prevYaw = 0;
int16_t prevZoom = 0;
const int16_t rateLimit = 500; // Adjust this value to control the rate of change



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
unsigned long lastDebounceTimeRight = 0;
unsigned long lastDebounceTimeLeft = 0;
unsigned long lastDebounceTimeJoystickTranslation = 0;
unsigned long lastDebounceTimeJoystickRotation = 0;

int lcdScreenCase = 0;
int lcdScreenCaseBeforeAlarm = 0;
bool lcdAlarmState = false;
bool lcdAlarmStateOverride = false;

bool translationButtonPressed = false;


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
void handleButtons(unsigned long now);
void handleJoystickButtons(unsigned long now);
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
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(LED_BUILTIN, HIGH);
  
  lcd.clear();
  lcd.print("KSP CONTROLLER!");
  lcd.setCursor(0, 1);
  lcd.print("Ready to connect");

  connectToKSP();
}

// Main loop function
void loop() {
  unsigned long now = millis();
  
  handleButtons(now);
  handleTempAlarm();
  mySimpit.update();

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
    delay(250);
  }
  isConnected = true;
  digitalWrite(LED_BUILTIN, LOW);
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
}

// Function to handle all buttons
void handleButtons(unsigned long now) {
  handleLCDButtons(now);
  handleJoystickButtons(now);
}

// Function to handle joystick buttons with debouncing
void handleJoystickButtons(unsigned long now) {
  int readingJoystickButtonTranslation = digitalRead(JOYSTICK_BUTTON_TRANSLATION);
  int readingJoystickButtonRotation = digitalRead(JOYSTICK_BUTTON_ROTATION);

  // Handle the translation button (inverted logic for pull-up)
  if (readingJoystickButtonTranslation == LOW && (now - lastDebounceTimeJoystickTranslation) > DEBOUNCE_DELAY) {
    translationButtonPressed = true;
    lastDebounceTimeJoystickTranslation = now;
  } else if (readingJoystickButtonTranslation == HIGH && (now - lastDebounceTimeJoystickTranslation) > DEBOUNCE_DELAY) {
    translationButtonPressed = false;
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
// Function to send camera commands
void sendCameraCommands() {
  cameraRotationMessage camMsg;
  
  int readingPitch = analogRead(TRANSLATE_Z_PIN);
  int readingYaw = analogRead(TRANSLATE_X_PIN);
  int readingZoom = analogRead(TRANSLATE_Y_PIN);

  int16_t pitch = 0;
  int16_t yaw = 0;
  int16_t zoom = 0;

  // Scale the readings
  int16_t scaledPitch = map(readingPitch, 0, 1023, -32768, 32767);
  int16_t scaledYaw = map(readingYaw, 0, 1023, -32768, 32767);
  int16_t scaledZoom = map(readingZoom, 0, 1023, -32768, 32767);

  // Implement dead zone handling
  if (scaledPitch > -100 && scaledPitch < 100) {
    pitch = 0;
  } else {
    pitch = scaledPitch;
  }

  if (scaledYaw > -100 && scaledYaw < 100) {
    yaw = 0;
  } else {
    yaw = scaledYaw;
  }

  if (scaledZoom > -100 && scaledZoom < 100) {
    zoom = 0;
  } else {
    zoom = scaledZoom;
  }

  // Apply rate limiter
  pitch = constrain(pitch, prevPitch - rateLimit, prevPitch + rateLimit);
  yaw = constrain(yaw, prevYaw - rateLimit, prevYaw + rateLimit);
  zoom = constrain(zoom, prevZoom - rateLimit, prevZoom + rateLimit);

  // Update previous values
  prevPitch = pitch;
  prevYaw = yaw;
  prevZoom = zoom;

  camMsg.setPitch(pitch);
  camMsg.setYaw(yaw);
  camMsg.setZoom(zoom);
  mySimpit.send(CAMERA_ROTATION_MESSAGE, camMsg);
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
  }
}
