#include <KerbalSimpit.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SerLCD.h>

SerLCD lcd; // Initialize the library with default I2C address 0x72
const int switchpinleft = 2;
const int switchpinright = 3;

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

int buttonStateRight;
int buttonStateLeft;
int lastButtonStateRight = HIGH;
int lastButtonStateLeft = HIGH;
int LCD_Button_case = 0;
int LCD_Button_case_before_alarm = 0;
bool LCD_alarm_state = false;

KerbalSimpit mySimpit(Serial);

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
const unsigned int LCDUpdateInterval = 750;  // Adjust this value to change the LCD update frequency

unsigned long lastDebounceTimeRight = 0;
unsigned long lastDebounceTimeLeft = 0;
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(Wire);
  lcd.setFastBacklight(255, 255, 255);
  lcd.createChar(0, deltaChar); // Create the custom character
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
  pinMode(switchpinright, INPUT_PULLUP);
  pinMode(switchpinleft, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  while (!mySimpit.init()) {
    delay(50);
  }
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

void loop() {
  unsigned long now = millis();
  
  handleButtons(now);
  handleTempAlarm();
  mySimpit.update();
  
  if (now - lastLCDUpdate >= LCDUpdateInterval) {
    updateLCD();
    lastLCDUpdate = now;  // Update the last LCD update time
  }
}

void handleButtons(unsigned long now) {
  int reading_switchpinright = digitalRead(switchpinright);
  int reading_switchpinleft = digitalRead(switchpinleft);

  // Debounce the right button
  if (reading_switchpinright != lastButtonStateRight) {
    lastDebounceTimeRight = now;
  }
  if ((now - lastDebounceTimeRight) > debounceDelay) {
    if (reading_switchpinright == LOW && buttonStateRight == HIGH) {
      if (LCD_Button_case < 8) {
        LCD_Button_case++;
        LCD_Button_case_before_alarm = LCD_Button_case;
        lcd.clear(); // clear screen for new display info
      }
    }
    buttonStateRight = reading_switchpinright;
  }
  lastButtonStateRight = reading_switchpinright;

  // Debounce the left button
  if (reading_switchpinleft != lastButtonStateLeft) {
    lastDebounceTimeLeft = now;
  }
  if ((now - lastDebounceTimeLeft) > debounceDelay) {
    if (reading_switchpinleft == LOW && buttonStateLeft == HIGH) {
      if (LCD_Button_case > 0) {
        LCD_Button_case--;
        LCD_Button_case_before_alarm = LCD_Button_case;
        lcd.clear(); // clear screen for new display info
      }
    }
    buttonStateLeft = reading_switchpinleft;
  }
  lastButtonStateLeft = reading_switchpinleft;
}

void handleTempAlarm() {
  if (myTemplimits.skinTempLimitPercentage > 30 || myTemplimits.tempLimitPercentage > 30) {
    if (!LCD_alarm_state) {
      LCD_Button_case_before_alarm = LCD_Button_case; // Save the current state before alarm
      LCD_Button_case = 98;
      LCD_alarm_state = true;
      lcd.setBacklight(255, 0, 0);
      lcd.clear();
    }
  } else {
    if (LCD_alarm_state) {
      LCD_alarm_state = false;
      LCD_Button_case = LCD_Button_case_before_alarm;
      lcd.setBacklight(255, 255, 255);
      lcd.clear();
    }
  }
}


void updateLCD() {
  switch (LCD_Button_case) {
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
