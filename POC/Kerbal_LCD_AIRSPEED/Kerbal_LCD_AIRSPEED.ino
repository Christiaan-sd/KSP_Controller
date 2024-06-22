#include "C:\GIT\KSP_Controller\libs\KerbalSimpit\src\KerbalSimpit.h"
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SerLCD.h> 

SerLCD lcd; // Initialize the library with default I2C address 0x72

int switchpin = 3;
int buttonState;
int lastButtonState = HIGH;
int LCD_Button_case = 0;
int LCD_Button_case_before_alarm = 0;

// /lcd state change status 0 = false, 1 = true
int LCD_state_change = 0;

KerbalSimpit mySimpit(Serial);

bool state = false;
unsigned long lastSent = 0;
unsigned int sendInterval = 1500;

airspeedMessage myAirspeed;
altitudeMessage myAltitude; // Added declaration for altitudeMessage
velocityMessage myVelocity; // Added declaration for velocityMessage
vesselPointingMessage myRotation;
tempLimitMessage myTemplimits;

// Add these global variables
unsigned long lastLCDUpdate = 0;
unsigned int LCDUpdateInterval = 250;  // Adjust this value to change the LCD update frequency

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(Wire);
  lcd.setFastBacklight(255, 255, 255);
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
  pinMode(switchpin, INPUT_PULLUP);
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
}

void loop() {
 
 int reading = digitalRead(switchpin);

  // If the button state has changed
  if (reading != buttonState) {
    buttonState = reading;

    // If the button is pressed
    if (buttonState == LOW) {
      LCD_Button_case = (LCD_Button_case + 1) % 5; // Cycle through the cases
      LCD_Button_case_before_alarm = LCD_Button_case;
      lcd.clear(); // clear screen for new display info
    }
  }

  lastButtonState = reading;
// HIER WAS JE! DE LOOP WORDT STEEDS GETRIGGERD WAARDOOR OOK EEN LCD CLEAR DUS WE ZIEN NIKS :(
    if (myTemplimits.skinTempLimitPercentage > 30 || myTemplimits.tempLimitPercentage > 30) {
    LCD_Button_case = 98;
    
      if (Error_lcd_reset_counter = 0) {
      Error_lcd_reset_counter + 1;
      lcd.clear();
      lcd.setBacklight(255,0,0);
      }

    } else {
      LCD_Button_case = LCD_Button_case_before_alarm;
      Error_lcd_reset_counter = 0;
    }


  unsigned long now = millis();


  mySimpit.update();
  
  // Update the LCD at the specified interval
  if (now - lastLCDUpdate >= LCDUpdateInterval) {
    // Display the corresponding information based on LCD_Button_case
    
    if (LCD_Button_case == 0) {
      
      
      lcd.setCursor(0, 0);
      lcd.print("MACH: ");
      lcd.print(myAirspeed.mach);
      lcd.setCursor(0, 1);
      lcd.print("Airspeed: ");
      lcd.print(round(myAirspeed.IAS));
    } else if (LCD_Button_case == 1) {
      
      
      lcd.setCursor(0, 0);
      lcd.print("Sealevel: ");
      lcd.print(round(myAltitude.sealevel));
      lcd.setCursor(0, 1);
      lcd.print("Surface: ");
      lcd.print(round(myAltitude.surface));
    } else if (LCD_Button_case == 2) {
      
      lcd.setCursor(0, 0);
      lcd.print("m/s: ");
      lcd.print(round(myVelocity.surface));
      lcd.setCursor(0, 1);
      lcd.print("km/h: ");
      lcd.print(round((myVelocity.surface * 3.6)));
    } else if (LCD_Button_case == 3) {
      
      lcd.setCursor(0, 0);
      lcd.print("Heading: ");
      lcd.print(myRotation.heading);
      lcd.setCursor(0, 1);
      lcd.print("Pitch: ");
      lcd.print(myRotation.pitch);
    }

    else if (LCD_Button_case == 4) {
      
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(myTemplimits.tempLimitPercentage);
      lcd.setCursor(0, 1);
      lcd.print("Temp: ");
      lcd.print(myTemplimits.skinTempLimitPercentage);
    }

    else if (LCD_Button_case == 98) {
      
      lcd.setCursor(0, 0);
      lcd.print("Temp ALARM!: ");
      lcd.print(myTemplimits.tempLimitPercentage);
      lcd.setCursor(0, 1);
      lcd.print("Temp ALARM!: ");
      lcd.print(myTemplimits.skinTempLimitPercentage);
    }

    lastLCDUpdate = now;  // Update the last LCD update time
  }
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch (messageType) {
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

    case TEMP_LIMIT_MESSAGE:
      if (msgSize == sizeof(tempLimitMessage)) {
        myTemplimits = parseMessage<tempLimitMessage>(msg);
           
      }
      break;

  }
}