/*
  LED Control with Potentiometer and Button for 2x 74HC595 Shift Registers

  This code will:
  1. Turn on a specific LED based on the potentiometer position.
  2. Toggle LED 12 (index 12) on and off with the button press.
  3. Toggle the potentiometer-controlled LEDs on and off with the button press.
*/

// Pin connected to ST_CP (Latch Pin) of 74HC595
int latchPin = 8;
// Pin connected to SH_CP (Clock Pin) of 74HC595
int clockPin = 12;
// Pin connected to DS (Data Pin) of 74HC595
int dataPin = 11;

// Pin connected to the middle pin of the 10k potentiometer (analog input)
int potPin = A0;

// Pin connected to the button (digital input)
int buttonPin = 2;

// Variables to store the button state
bool buttonState = false;
bool lastButtonState = false;

// Variables for debouncing the button
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; // Debounce delay in milliseconds

// Variable to track the state of LED 12
bool led12State = false;

// Variable to track whether potentiometer-controlled LEDs are enabled
bool potLedsEnabled = false;

// LED address array
const byte ledAddresses[13] = {
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
  0x08,  // LED 11 (SR2)
  0x80   // LED 12 (SR2) - LED 12
};

void setup() {
  // Set pins to output or input as needed
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor

  // Initialize LED 12 as output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Start with LED 12 and potentiometer-controlled LEDs off
  updateShiftRegisters(0x00, 0x00); // Turn off all LEDs initially
}

void loop() {
  // Read the button state
  int reading = digitalRead(buttonPin);

  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // Only change the button state if the debounce delay has passed
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // If button is pressed (LOW), toggle LED 12 and the potentiometer LEDs
      if (buttonState == LOW) {
        // Toggle LED 12 state
        led12State = !led12State;
        Serial.print("Button pressed - LED 12 ");
        Serial.println(led12State ? "ON" : "OFF");

        // Toggle potentiometer-controlled LEDs
        potLedsEnabled = !potLedsEnabled;
        Serial.print("Potentiometer-controlled LEDs ");
        Serial.println(potLedsEnabled ? "ENABLED" : "DISABLED");
      }
    }
  }

  // Update the previous button state
  lastButtonState = reading;

  // Initialize LED data
  byte data1 = 0x00;  // Initialize all LEDs off for SR1
  byte data2 = 0x00;  // Initialize all LEDs off for SR2

  // Determine LED control based on potentiometer and button states
  if (potLedsEnabled) {
    // Read the potentiometer value (range: 0-1023)
    int potValue = analogRead(potPin);
    // Map the potentiometer value to a range of 0-12 (13 LEDs in total)
    int ledIndex = map(potValue, 0, 1024, 0, 12);

    // Print the potentiometer and LED index values for debugging
    Serial.print("Potentiometer Value: ");
    Serial.print(potValue);
    Serial.print(" | LED Index: ");
    Serial.println(ledIndex);

    // Perform actions based on LED index
    switch (ledIndex) {
      case 0:
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
        // Turn on corresponding LED in SR1
        data1 |= ledAddresses[ledIndex];
        break;
      case 8:
      case 9:
      case 10:
      case 11:
        // Turn on corresponding LED in SR2
        data2 |= ledAddresses[ledIndex];
        break;
      case 12:
        // Always turn on LED 12 (SR2)
        data2 |= ledAddresses[12];
        break;
      default:
        // Ensure no LEDs are on if index is out of range
        data1 = 0x00;
        data2 = 0x00;
        break;
    }
  }

  // Set the state of LED 12 in the second shift register
  if (led12State) {
    data2 |= ledAddresses[12]; // Ensure LED 12 is on if toggled
  } else {
    data2 &= ~ledAddresses[12]; // Ensure LED 12 is off if toggled
  }

  // Update the shift registers with the new LED states
  updateShiftRegisters(data1, data2);

  // Small delay to avoid too rapid updates
  delay(100); // Adjust delay for better readability
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
