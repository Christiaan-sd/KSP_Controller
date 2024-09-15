#include <Wire.h>
#include <SerLCD.h>

// Shift register pin configuration
const int LATCH_PIN = 15;  // Pin connected to ST_CP (Latch Pin) of 74HC595
const int CLOCK_PIN = 14;  // Pin connected to SH_CP (Clock Pin) of 74HC595
const int DATA_PIN = 16;   // Pin connected to DS (Data Pin) of 74HC595

const int POT_PIN = A7;    // Potentiometer for LED control

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

void setup() {
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
}

void loop() {
  SAS_mode_pot();
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
