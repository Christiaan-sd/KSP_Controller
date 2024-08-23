/*
  KSP SAS Control with Button, Potentiometer, and 12 LEDs

  This code will:
  1. Toggle SAS on and off with the button press.
  2. Change between 12 SAS modes using a potentiometer.
  3. Light up the corresponding LED to indicate the active SAS mode.
*/

#include "KerbalSimpit.h"

// Pin configuration
const int SAS_BUTTON_PIN = 2;  // Button for toggling SAS
const int POT_PIN = A0;        // Potentiometer for changing SAS mode

// Shift register pin configuration
const int latchPin = 8;  // Pin connected to ST_CP (Latch Pin) of 74HC595
const int clockPin = 12; // Pin connected to SH_CP (Clock Pin) of 74HC595
const int dataPin = 11;  // Pin connected to DS (Data Pin) of 74HC595

// Variables to store the button state
bool buttonState = false;
bool lastButtonState = false;

// Variables for debouncing the button
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; // Debounce delay in milliseconds

// Variable to track the SAS state
bool sasState = false;

// Variables for SAS mode
int lastPotValue = 0;
int currentSASMode = 0;

// Declare a KerbalSimpit object that will communicate using the "Serial" device.
KerbalSimpit mySimpit(Serial);

// SAS mode constants
const int NUM_SAS_MODES = 12; // Number of SAS modes available

// LED address array for shift registers (12 LEDs)
const byte ledAddresses[NUM_SAS_MODES] = {
  0x01,  // LED 0 (SR1)
  0x02,  // LED 1 (SR1)
  0x04,  // LED 2 (SR1)
  0x08,  // LED 3 (SR1)
  0x10,  // LED 4 (SR1)
  0x20,  // LED 5 (SR1)
  0x40,  // LED 6 (SR1)
  0x80,  // LED 7 (SR1)
  0x01,  // LED 8 (SR2)
  0x02,  // LED 9 (SR2)
  0x04,  // LED 10 (SR2)
  0x08   // LED 11 (SR2)
};

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set up the pins
  pinMode(SAS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);

  // Set up shift register pins
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  // Set up the built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Attempt to handshake with the plugin
  while (!mySimpit.init()) {
    delay(100);
  }

  // Indicate that the handshake is complete
  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  mySimpit.inboundHandler(messageHandler);
  mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);
}

void loop() {
  // Check for new serial messages
  mySimpit.update();

  // Read the button state
  int reading = digitalRead(SAS_BUTTON_PIN);

  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // Only change the button state if the debounce delay has passed
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // If button is pressed, toggle SAS
      if (buttonState == LOW) {
        sasState = !sasState;
        if (sasState) {
          mySimpit.activateAction(SAS_ACTION);
          mySimpit.printToKSP("SAS Activated", PRINT_TO_SCREEN);
        } else {
          mySimpit.deactivateAction(SAS_ACTION);
          mySimpit.printToKSP("SAS Deactivated", PRINT_TO_SCREEN);
        }
      }
    }
  }

  // Update the previous button state
  lastButtonState = reading;

  // Read the potentiometer value
  int potValue = analogRead(POT_PIN);
  int potIndex = map(potValue, 0, 1023, 0, NUM_SAS_MODES - 1); // Map to 12 SAS modes

  // Change SAS mode if potentiometer value changed
  if (potIndex != currentSASMode) {
    currentSASMode = potIndex;
    mySimpit.setSASMode(currentSASMode);
    mySimpit.printToKSP("SAS Mode Changed", PRINT_TO_SCREEN);
  }

  // Update the LEDs to reflect the current SAS mode
  updateShiftRegisters(currentSASMode);

  // Update the LED to reflect the SAS state
  digitalWrite(LED_BUILTIN, sasState ? HIGH : LOW);

  // Small delay to avoid rapid polling
  delay(100);
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  if (messageType == ACTIONSTATUS_MESSAGE && msgSize == 1) {
    // Update the SAS state
    sasState = msg[0] & SAS_ACTION;

    // Update the built-in LED to match the SAS state
    digitalWrite(LED_BUILTIN, sasState ? HIGH : LOW);
  }
}

// Function to update the shift registers based on the current SAS mode
void updateShiftRegisters(int modeIndex) {
  byte data1 = 0x00;  // Initialize all LEDs off for SR1
  byte data2 = 0x00;  // Initialize all LEDs off for SR2

  // Determine which LED to turn on based on the mode index
  if (modeIndex < 8) {
    // Turn on corresponding LED in SR1
    data1 |= ledAddresses[modeIndex];
  } else {
    // Turn on corresponding LED in SR2
    data2 |= ledAddresses[modeIndex];
  }

  // Update the shift registers with the new LED states
  updateShiftRegisters(data1, data2);
}

// Function to update both shift registers
void updateShiftRegisters(byte data1, byte data2) {
  // Ground latchPin and hold low for as long as you are transmitting
  digitalWrite(latchPin, LOW);

  // Shift out the bits to the second shift register (SR2) first, then SR1
  shiftOut(dataPin, clockPin, data2); // Send to second shift register
  shiftOut(dataPin, clockPin, data1); // Send to first shift register

  // Return the latch pin high to lock the data into the shift registers
  digitalWrite(latchPin, HIGH);
}

// The heart of the program
void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  // This shifts 8 bits out MSB first on the rising edge of the clock, clock idles low
  for (int i = 7; i >= 0; i--) {
    digitalWrite(myClockPin, LOW);
    
    // Set the data pin to HIGH or LOW depending on the bit value
    digitalWrite(myDataPin, (myDataOut & (1 << i)) ? HIGH : LOW);

    // Register shifts bits on the upstroke of clock pin
    digitalWrite(myClockPin, HIGH);
    digitalWrite(myDataPin, LOW);
  }

  // Stop shifting
  digitalWrite(myClockPin, LOW);
}
