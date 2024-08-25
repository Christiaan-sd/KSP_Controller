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
byte myCurrentSASMode;
int16_t mySASModeAvailability;
bool echoReceived = false;

// Variables for debouncing the button
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; // Debounce delay in milliseconds

// Variable to track the SAS state
bool sasState = false;
// Variables for SAS mode
int currentSASMode = 0;

// Variables for millis() timing
unsigned long sasModeChangeTime = 0;
unsigned long sasModeDelay = 1500; // Delay time for SAS mode change

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

  // Attempt to handshake with the plugin
  while (!mySimpit.init()) {
    delay(100);
  }

  // Indicate that the handshake is complete
  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  mySimpit.inboundHandler(messageHandler);
  mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);
  mySimpit.registerChannel(SAS_MODE_INFO_MESSAGE);
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
          // Activate SAS in the game
          mySimpit.activateAction(SAS_ACTION);
          mySimpit.printToKSP("SAS Activated", PRINT_TO_SCREEN);
          

          // Update the SAS mode based on current potentiometer value
          updateSASModeFromPot();
        } else {
          // Deactivate SAS in the game and turn off LEDs
          mySimpit.deactivateAction(SAS_ACTION);
          mySimpit.printToKSP("SAS Deactivated", PRINT_TO_SCREEN);
          
          // Update LEDs to reflect the current state
          updateSASLEDs(currentSASMode, sasState);
        }
      }
    }
  }

  // Update the previous button state
  lastButtonState = reading;

  // Only update the SAS mode if SAS is active
  if (sasState) {
    updateSASModeFromPot();
  }
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch (messageType) {
    case ECHO_RESP_MESSAGE: {
        echoReceived = true;
      } break;

    case ACTIONSTATUS_MESSAGE: {
        if (msgSize == 1) {
          bool newSasState = msg[0] & SAS_ACTION;

          if (newSasState && !sasState) {
            // SAS was turned on externally
            sasState = true;
            mySimpit.printToKSP("SAS Activated Externally", PRINT_TO_SCREEN);
            
            // Update the SAS mode based on current potentiometer value
            updateSASModeFromPot();
          } else if (!newSasState && sasState) {
            // SAS was turned off externally
            sasState = false;
            
            mySimpit.printToKSP("SAS Deactivated Externally", PRINT_TO_SCREEN);

            // Update LEDs to reflect the current state and mode
            updateSASLEDs(currentSASMode, sasState);
          }
        }
      } break;

    case SAS_MODE_INFO_MESSAGE: {
        if (msgSize == sizeof(SASInfoMessage)) {
          SASInfoMessage sasInfoMsg = parseMessage<SASInfoMessage>(msg);
          myCurrentSASMode = sasInfoMsg.currentSASMode;
          mySASModeAvailability = sasInfoMsg.SASModeAvailability;
        }
      } break;
  }
}

void updateSASModeFromPot() {
  int potValue = analogRead(POT_PIN);
  int potIndex = map(potValue, 0, 1023, 0, NUM_SAS_MODES - 1);
  String sasModeString = String(myCurrentSASMode);
  mySimpit.printToKSP(sasModeString, PRINT_TO_SCREEN);


  if (myCurrentSASMode != potIndex) {
    currentSASMode = potIndex;
   
    mySimpit.setSASMode(currentSASMode);
    mySimpit.printToKSP("SAS Mode Changed", PRINT_TO_SCREEN);
    
    // Update the LEDs to reflect the current SAS mode and state
    updateSASLEDs(currentSASMode, sasState);
  }
}

// Function to update both the SAS mode LEDs and the SAS state LED
void updateSASLEDs(int modeIndex, bool state) {
  byte data1 = 0x00;  // Initialize all LEDs off for SR1
  byte data2 = 0x00;  // Initialize all LEDs off for SR2

  if (state) {
    // Determine which LED to turn on based on the mode index
    if (modeIndex < 8) {
      data1 |= ledAddresses[modeIndex];
    } else if (modeIndex >= 8 && modeIndex < 12) {
      data2 |= ledAddresses[modeIndex];
    }

    // Turn on the SAS state LED if SAS is enabled
    data2 |= 0x10; // Assuming the LED at index 12 is the 5th LED in SR2
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
