#include "KerbalSimpit.h"

// Pin configuration
const int SAS_BUTTON_PIN = 17;  // Button for toggling SAS
const int POT_PIN = A7;        // Potentiometer for changing SAS mode

// Shift register pin configuration
const int LATCH_PIN = 15;  // Pin connected to ST_CP (Latch Pin) of 74HC595
const int CLOCK_PIN = 14; // Pin connected to SH_CP (Clock Pin) of 74HC595
const int DATA_PIN = 16;  // Pin connected to DS (Data Pin) of 74HC595

// SAS mode constants
const int NUM_SAS_MODES = 12; // Number of SAS modes available

// LED address array for shift registers (12 LEDs)
const byte LED_ADDRESSES[NUM_SAS_MODES] = {
  0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, // SR1 LEDs
  0x01, 0x02, 0x04, 0x08  // SR2 LEDs
};

// Variables
bool buttonS
  tate = false;
bo
ol lastButtonState = false;
bool sasState = false;
bool echoReceived = false;
byte myCurrentSASMode = 0;
int16_t mySASModeAvailability = 0;
int currentSASMode = 0;

// Timing variables
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // Debounce delay in milliseconds
unsigned long sasModeChangeTime = 0;
const unsigned long SAS_MODE_DELAY = 1500; // Delay time for SAS mode change

// KerbalSimpit object
KerbalSimpit mySimpit(Serial);

void setup() {
  Serial.begin(115200);

  pinMode(SAS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

  while (!mySimpit.init()) {
    delay(100);
  }

  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  mySimpit.inboundHandler(messageHandler);
  mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);
  mySimpit.registerChannel(SAS_MODE_INFO_MESSAGE);
}

void loop() {
  mySimpit.update();
  handleButton();
  
  if (sasState) {
    updateSASModeFromPot();
  }
}

void handleButton() {
  int reading = digitalRead(SAS_BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        toggleSAS();
      }
    }
  }

  lastButtonState = reading;
}

void toggleSAS() {
  sasState = !sasState;
  if (sasState) {
    mySimpit.activateAction(SAS_ACTION);
    mySimpit.printToKSP("SAS Activated", PRINT_TO_SCREEN);
    updateSASModeFromPot();
  } else {
    mySimpit.deactivateAction(SAS_ACTION);
    mySimpit.printToKSP("SAS Deactivated", PRINT_TO_SCREEN);
    updateSASLEDs(currentSASMode, sasState);
  }
}

void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch (messageType) {
    case ECHO_RESP_MESSAGE:
      echoReceived = true;
      break;

    case ACTIONSTATUS_MESSAGE:
      handleActionStatusMessage(msg, msgSize);
      break;

    case SAS_MODE_INFO_MESSAGE:
      handleSASModeInfoMessage(msg, msgSize);
      break;
  }
}

void handleActionStatusMessage(byte msg[], byte msgSize) {
  if (msgSize == 1) {
    bool newSasState = msg[0] & SAS_ACTION;
    if (newSasState != sasState) {
      sasState = newSasState;
      mySimpit.printToKSP(sasState ? "SAS Activated Externally" : "SAS Deactivated Externally", PRINT_TO_SCREEN);
      updateSASLEDs(currentSASMode, sasState);
      if (sasState) {
        updateSASModeFromPot();
      }
    }
  }
}

void handleSASModeInfoMessage(byte msg[], byte msgSize) {
  if (msgSize == sizeof(SASInfoMessage)) {
    SASInfoMessage sasInfoMsg = parseMessage<SASInfoMessage>(msg);
    myCurrentSASMode = sasInfoMsg.currentSASMode;
    mySASModeAvailability = sasInfoMsg.SASModeAvailability;
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
    updateSASLEDs(currentSASMode, sasState);
  }
}

void updateSASLEDs(int modeIndex, bool state) {
  byte data1 = 0x00;  // LEDs for SR1
  byte data2 = 0x00;  // LEDs for SR2

  if (state) {
    if (modeIndex < 8) {
      data1 |= LED_ADDRESSES[modeIndex];
    } else {
      data2 |= LED_ADDRESSES[modeIndex];
    }
    data2 |= 0x10; // Turn on the SAS state LED
  }

  updateShiftRegisters(data1, data2);
}

void updateShiftRegisters(byte data1, byte data2) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, data2);
  shiftOut(DATA_PIN, CLOCK_PIN, data1);
  digitalWrite(LATCH_PIN, HIGH);
}

void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(myClockPin, LOW);
    digitalWrite(myDataPin, (myDataOut & (1 << i)) ? HIGH : LOW);
    digitalWrite(myClockPin, HIGH);
    digitalWrite(myDataPin, LOW);
  }
  digitalWrite(myClockPin, LOW);
}
