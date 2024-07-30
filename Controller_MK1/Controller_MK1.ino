#include <KerbalSimpit.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SerLCD.h>

// Declare a KerbalSimpit object that will
// communicate using the "Serial" device.
KerbalSimpit mySimpit(Serial);

// Set the pin numbers:
const int THROTTLE_PIN = A0; // the pin used for controlling throttle
const int PITCH_PIN = A1;    // the pin used for controlling pitch
const int ROLL_PIN = A2;     // the pin used for controlling roll
const int YAW_PIN = A3;      // the pin used for controlling YAW

// Translation joystick pins
const int TRANSLATE_X_PIN = A5; // the pin used for controlling translation X
const int TRANSLATE_Y_PIN = A6; // the pin used for controlling translation Y
const int TRANSLATE_Z_PIN = A4; // the pin used for controlling translation Z


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

SerLCD lcd; // Initialize the library with default I2C address 0x72
const int Joystick_button_Translation = 0;
const int Joystick_button_Rotation = 1;
const int LCD_switchpinleft = 2;
const int LCD_switchpinright = 3;

int LCD_SwitchButtonStateRight;
int LCD_SwitchButtonStateLeft;
int LCD_Last_Switch_Button_State_Right = HIGH;
int LCD_Last_Switch_Button_State_Left = HIGH;
int JoystickButtonStateTranslation = HIGH; // initial state for pull-up
int JoystickButtonStateRotation = HIGH;    // initial state for pull-up
int LCD_Screen_Case = 0;
int LCD_Screen_Case_Before_Alarm = 0;
bool LCD_alarm_state = false;
bool LCD_alarm_state_overide = false;



bool state = false;
unsigned long lastSent = 0;
const unsigned int sendInterval = 1500;

airspeedMessage myAirspeed;
deltaVMessage myDeltaV;
altitudeMessage myAltitude; // Added declaration for altitudeMessage
velocityMessage myVelocity; // Added declaration for velocityMessage
vesselPointingMessage myRotation;
tempLimitMessage myTemplimits;
atmoConditionsMessage myAtmoConditions;
resourceMessage myElectric;

unsigned long lastLCDUpdate = 0;
const unsigned int LCDUpdateInterval = 125;  // Adjust this value to change the LCD update frequency

unsigned long lastDebounceTimeRight = 0;
unsigned long lastDebounceTimeLeft = 0;
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds

bool isConnected = false;

void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud
  Wire.begin();
  
  // Initialize LCD
  lcd.begin(Wire);
  lcd.setFastBacklight(255, 255, 255);
  lcd.createChar(0, deltaChar); // Create the custom character
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
  
  // Set pin modes
  pinMode(LCD_switchpinright, INPUT_PULLUP);
  pinMode(LCD_switchpinleft, INPUT_PULLUP);
  pinMode(Joystick_button_Translation, INPUT_PULLUP);
  pinMode(Joystick_button_Rotation, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(LED_BUILTIN, HIGH);
  
  lcd.clear();
  lcd.print("KSP CONTROLLER!");
  lcd.setCursor(0, 1);
  lcd.print("Ready to connect");

  connectToKSP();
}

void loop() {
  unsigned long now = millis();
  
  handleButtons(now);
  handleTempAlarm();
  mySimpit.update();

  if (now - lastLCDUpdate >= LCDUpdateInterval) {
    updateLCD();
    lastLCDUpdate = now;  // Update the last LCD update time
  }
  
  // Reconnect if disconnected
  if (!isConnected) {
    connectToKSP();
  }
  
  // Send at each loop a message to control the throttle and the pitch/roll axis.
  sendRotationCommands();
  sendTranslationCommands();
  sendThrottleCommands();
}


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

void handleButtons(unsigned long now) {
  // Handle LCD screen buttons
  handleLCDButtons();

  // Handle joystick buttons
  handleJoystickButtons();

  // Future button handling can be added here
}


void handleJoystickButtons() {
  int reading_Joystick_button_Translation = digitalRead(Joystick_button_Translation);
  int reading_Joystick_button_Rotation = digitalRead(Joystick_button_Rotation);

  // Handle the translation button (inverted logic for pull-up)
  if (reading_Joystick_button_Translation == LOW) {
    
    mySimpit.activateAction(STAGE_ACTION);
  }


  // Handle the rotation button (inverted logic for pull-up)
  if (reading_Joystick_button_Rotation == LOW) {

   mySimpit.activateAction(STAGE_ACTION);
  }
  

}


void handleLCDButtons() {
  int reading_LCD_switchpinright = digitalRead(LCD_switchpinright);
  int reading_LCD_switchpinleft = digitalRead(LCD_switchpinleft);

  // Handle the right button
  if (reading_LCD_switchpinright == LOW && LCD_SwitchButtonStateRight == HIGH) {
    if (LCD_Screen_Case < 8) {
      LCD_Screen_Case++;
      LCD_Screen_Case_Before_Alarm = LCD_Screen_Case;
      lcd.clear(); // clear screen for new display info
      if (LCD_alarm_state) {
        LCD_alarm_state_overide = false;
        LCD_alarm_state = false;
        LCD_Screen_Case = 0;
        lcd.clear();
        lcd.setBacklight(255, 255, 255);
      }
    }
  }
  LCD_SwitchButtonStateRight = reading_LCD_switchpinright;

  // Handle the left button
  if (reading_LCD_switchpinleft == LOW && LCD_SwitchButtonStateLeft == HIGH) {
    if (LCD_Screen_Case > 0) {
      LCD_Screen_Case--;
      LCD_Screen_Case_Before_Alarm = LCD_Screen_Case;
      lcd.clear(); // clear screen for new display info
      if (LCD_alarm_state) {
        LCD_alarm_state_overide = true;
        LCD_alarm_state = false;
        LCD_Screen_Case = 0;
        lcd.clear();
        lcd.setBacklight(255, 255, 255);
      }
    }
  }
  LCD_SwitchButtonStateLeft = reading_LCD_switchpinleft;
}

void handleTempAlarm() {
  if (myTemplimits.skinTempLimitPercentage > 40 || myTemplimits.tempLimitPercentage > 40) {
    if (!LCD_alarm_state && !LCD_alarm_state_overide) {
      LCD_Screen_Case_Before_Alarm = LCD_Screen_Case; // Save the current state before alarm
      LCD_Screen_Case = 98;
      LCD_alarm_state = true;
      lcd.setBacklight(255, 0, 0);
      lcd.clear();
    }
  } else {
    if (LCD_alarm_state) {
      LCD_alarm_state = false;
      LCD_alarm_state_overide = false; 
      LCD_Screen_Case = LCD_Screen_Case_Before_Alarm;
      lcd.setBacklight(255, 255, 255);
      lcd.clear();
    }
  }
}

void updateLCD() {
  switch (LCD_Screen_Case) {
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

void sendThrottleCommands(){
  // Send at each loop a message to control the throttle and the pitch/roll axis.
  
  throttleMessage throttle_msg;
  // Read the value of the potentiometer
  int reading = analogRead(THROTTLE_PIN);
  // Convert it in KerbalSimpit Range
  throttle_msg.throttle = map(reading, 0, 1023, INT16_MAX, 0);

  // Send the message
  mySimpit.send(THROTTLE_MESSAGE, throttle_msg);
  
}

#include <PayloadStructs.h>

void sendCameraCommands() {
  cameraRotationMessage cam_msg;

  // Read the values of the potentiometers
  int reading_Pitch = analogRead(TRANSLATE_X_PIN);
  int reading_Roll = analogRead(TRANSLATE_Y_PIN);
  int reading_Zoom = analogRead(TRANSLATE_Z_PIN);

  // Deadzone size
  const int DEADZONE = 20; // Adjust this value as needed

  // Calculate deadzone for Pitch control
  int16_t pitch = 0;
  if (reading_Pitch > (512 + DEADZONE)) {
    pitch = map(reading_Pitch, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (reading_Pitch < (512 - DEADZONE)) {
    pitch = map(reading_Pitch, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  // Calculate deadzone for Roll control (inverted)
  int16_t roll = 0;
  if (reading_Roll > (512 + DEADZONE)) {
    roll = map(reading_Roll, 512 + DEADZONE, 1023, 0, INT16_MIN);
  } else if (reading_Roll < (512 - DEADZONE)) {
    roll = map(reading_Roll, 0, 512 - DEADZONE, INT16_MAX, 0);
  }

  // Calculate deadzone for Zoom control
  int16_t zoom = 0;
  if (reading_Zoom > (512 + DEADZONE)) {
    zoom = map(reading_Zoom, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (reading_Zoom < (512 - DEADZONE)) {
    zoom = map(reading_Zoom, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  // Set the values in the message
  cam_msg.setPitch(pitch);
  cam_msg.setRoll(roll);
  cam_msg.setZoom(zoom);

  // Send the message
  mySimpit.send(CAMERA_ROTATION_MESSAGE, cam_msg);
}


void sendTranslationCommands() {
  translationMessage trans_msg;

  // Read the values of the potentiometers
  int reading_X = analogRead(TRANSLATE_X_PIN);
  int reading_Y = analogRead(TRANSLATE_Y_PIN);
  int reading_Z = analogRead(TRANSLATE_Z_PIN);

  // Deadzone size
  const int DEADZONE = 20; // Adjust this value as needed

  // Calculate deadzone for X translation
  int16_t translateX = 0;
  if (reading_X > (512 + DEADZONE)) {
    translateX = map(reading_X, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (reading_X < (512 - DEADZONE)) {
    translateX = map(reading_X, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  // Calculate deadzone for Y translation
  int16_t translateY = 0;
  if (reading_Y > (512 + DEADZONE)) {
    translateY = map(reading_Y, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (reading_Y < (512 - DEADZONE)) {
    translateY = map(reading_Y, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  // Calculate deadzone for Z translation (inverted)
  int16_t translateZ = 0;
  if (reading_Z > (512 + DEADZONE)) {
    translateZ = map(reading_Z, 512 + DEADZONE, 1023, 0, INT16_MIN);
  } else if (reading_Z < (512 - DEADZONE)) {
    translateZ = map(reading_Z, 0, 512 - DEADZONE, INT16_MAX, 0);
  }

  // Put those values in the message
  trans_msg.setX(translateX);
  trans_msg.setY(translateY);
  trans_msg.setZ(translateZ);

  // Send the message
  mySimpit.send(TRANSLATION_MESSAGE, trans_msg);
}


void sendRotationCommands() {
  rotationMessage rot_msg;
  // Read the values of the potentiometers
  int reading_pitch = analogRead(PITCH_PIN);
  int reading_roll = analogRead(ROLL_PIN);
  int reading_yaw = analogRead(YAW_PIN);

  // Deadzone size
  const int DEADZONE = 20; // Adjust this value as needed

  // Calculate deadzone for pitch
  int16_t pitch = 0;
  if (reading_pitch > (512 + DEADZONE)) {
    pitch = map(reading_pitch, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (reading_pitch < (512 - DEADZONE)) {
    pitch = map(reading_pitch, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  // Calculate deadzone for roll
  int16_t roll = 0;
  if (reading_roll > (512 + DEADZONE)) {
    roll = map(reading_roll, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (reading_roll < (512 - DEADZONE)) {
    roll = map(reading_roll, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  // Calculate deadzone for yaw
  int16_t yaw = 0;
  if (reading_yaw > (512 + DEADZONE)) {
    yaw = map(reading_yaw, 512 + DEADZONE, 1023, 0, INT16_MAX);
  } else if (reading_yaw < (512 - DEADZONE)) {
    yaw = map(reading_yaw, 0, 512 - DEADZONE, INT16_MIN, 0);
  }

  // Put those values in the message
  rot_msg.setPitch(pitch);
  rot_msg.setRoll(roll);
  rot_msg.setYaw(yaw);
  // Send the message
  mySimpit.send(ROTATION_MESSAGE, rot_msg);
}


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
