#include <Wire.h>
#include <SerLCD.h>

// Shift register pin configuration
const int LATCH_PIN = 15;  // Pin connected to ST_CP (Latch Pin) of 74HC595
const int CLOCK_PIN = 14;  // Pin connected to SH_CP (Clock Pin) of 74HC595
const int DATA_PIN = 16;   // Pin connected to DS (Data Pin) of 74HC595

const int POT_PIN = A7;    // Potentiometer for LED control

// Number of LEDs (total across two shift registers)
const int NUM_LEDS = 16;

// Define each LED with its corresponding address in the shift registers
const byte LED_SAS = 0x01;
const byte LED_AUTO_PILOT = 0x02;
const byte LED_ANTI_NORMAL = 0x20;
const byte LED_NORMAL = 0x08;
const byte LED_RETRO_GRADE = 0x10;
const byte LED_PRO_GRADE = 0x40;
const byte LED_MANAUVER = 0x04;
const byte LED_STABILITY_ASSIST = 0x80;
const byte LED_RADIAL_OUT = 0x01;
const byte LED_RADIAL_IN = 0x02;
const byte LED_TARGET = 0x04; 
const byte LED_ANTI_TARGET = 0x08;
const byte LED_NAVIGATION = 0x10;
const byte LED_RCS = 0x20;
const byte LED_GEARS = 0x40; 
const byte LED_BRAKES = 0x80;

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

  // Create two byte variables to store the states of the shift registers
  byte data1 = 0x00;  // LEDs for SR1 (first 8 LEDs)
  byte data2 = 0x00;  // LEDs for SR2 (next 8 LEDs)

  // Define your own custom potentiometer ranges for each LED
  if (potValue >= 0 && potValue < 10) {
    data1 = LED_AUTO_PILOT;
  } else if (potValue >= 10 && potValue < 95) {
    data1 = LED_ANTI_NORMAL;
  } else if (potValue >= 95 && potValue < 195) {
    data1 = LED_NORMAL;
  } else if (potValue >= 195 && potValue < 310) {
    data1 = LED_RETRO_GRADE;
  } else if (potValue >= 310 && potValue < 420) {
    data1 = LED_PRO_GRADE;
  } else if (potValue >= 420 && potValue < 507) {
    data1 = LED_MANAUVER;
  } else if (potValue >= 507 && potValue < 600) {
    data1 = LED_STABILITY_ASSIST;
  } else if (potValue >= 600 && potValue < 690) {
    data2 = LED_RADIAL_OUT;
  } else if (potValue >= 690 && potValue < 790) {
    data2 = LED_RADIAL_IN;
  } else if (potValue >= 790 && potValue < 875) {
    data2 = LED_TARGET;
  } else if (potValue >= 875 && potValue < 975) {
    data2 = LED_ANTI_TARGET;
  } else if (potValue >= 975 && potValue < 1024) {
    data2 = LED_NAVIGATION;
  }

  // Update the shift registers with the LED states
  updateShiftRegisters(data1, data2);
}

// Function to update the shift registers with new LED data
void updateShiftRegisters(byte data1, byte data2) {
  digitalWrite(LATCH_PIN, LOW);     // Prepare to send data
  shiftOut(DATA_PIN, CLOCK_PIN, data2);  // Send data for SR2 (LEDs 9-16)
  shiftOut(DATA_PIN, CLOCK_PIN, data1);  // Send data for SR1 (LEDs 1-8)
  digitalWrite(LATCH_PIN, HIGH);    // Latch the data (output to LEDs)
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
