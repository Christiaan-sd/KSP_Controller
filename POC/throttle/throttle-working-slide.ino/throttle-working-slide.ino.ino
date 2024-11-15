// Constants for control pins
const int THROTTLE_PIN = A0;

void setup() {
  // Initialize serial communication at 9600 bps
  Serial.begin(9600);
  
  // Give the user some time to open the Serial Monitor
  delay(1000);
}

void loop() {
  // Read the throttle input
  int reading = analogRead(THROTTLE_PIN);
  int throttlePercentage = 0;

  // Interpolate the throttle percentage based on observed ranges
  if (reading >= 900) {
    // Map between 995 (0%) and 900 (25%)
    throttlePercentage = 0 + (995 - reading) * 25 / (995 - 900);
  } else if (reading >= 500) {
    // Map between 900 (25%) and 500 (50%)
    throttlePercentage = 25 + (900 - reading) * 25 / (900 - 500);
  } else if (reading >= 118) {
    // Map between 500 (50%) and 118 (75%)
    throttlePercentage = 50 + (500 - reading) * 25 / (500 - 118);
  } else if (reading >= 8) {
    // Map between 118 (75%) and 8 (100%)
    throttlePercentage = 75 + (118 - reading) * 25 / (118 - 8);
  } else {
    // Default to 0% throttle if reading is outside expected range
    throttlePercentage = 100;
  }

  // Print out the raw and interpolated throttle values for testing
  Serial.print("Raw throttle reading: ");
  Serial.print(reading);
  Serial.print(" | Throttle percentage: ");
  Serial.println(throttlePercentage);

  // Delay a bit to avoid overwhelming the Serial Monitor
  delay(250);
}
