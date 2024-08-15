#include "KerbalSimpit.h"

// Define the pins for joystick input
const int YAW_PIN = A5;  // X-axis (Left/Right)
const int PITCH_PIN = A4; // Y-axis (Up/Down)
const int ZOOM_PIN = A6;  // Z-axis (In/Out)

// Define the deadzone to avoid small movements around the center
const int DEADZONE = 40;

// Define key codes for Yaw control in KSP
const int LEFT_KEY = 0x25;   // Left arrow key
const int RIGHT_KEY = 0x27;  // Right arrow key

// Define key codes for Pitch control in KSP
const int PITCH_UP_KEY = 0x26;   // Up arrow key (Zoom In)
const int PITCH_DOWN_KEY = 0x28; // Down arrow key (Zoom Out)

// Define key codes for Zoom control in KSP
const int ZOOM_IN_KEY = 0x21;   // Numpad Plus key (Zoom In)
const int ZOOM_OUT_KEY = 0x22;  // Numpad Minus key (Zoom Out)

// Track whether keys are currently pressed
bool left_pressed = false, right_pressed = false;
bool zoom_in_pressed = false, zoom_out_pressed = false;
bool pitch_up_pressed = false, pitch_down_pressed = false;

KerbalSimpit mySimpit(Serial);

void setup() {
  Serial.begin(115200);

  // Initialize communication with the SimPit plugin
  while (!mySimpit.init()) {
    delay(100);
  }
  mySimpit.printToKSP("Camera Control Connected", PRINT_TO_SCREEN);
}

void loop() {
  mySimpit.update();
  sendYawCommands();  // Handle yaw commands first
  sendPitchCommands(); // Handle pitch commands second
  sendZoomCommands(); // Handle zoom commands third
}

void sendYawCommands() {
  int readingYaw = analogRead(YAW_PIN);

  // Handle yaw (Left)
  if (readingYaw > (512 + DEADZONE) && !left_pressed) {
    keyboardEmulatorMessage yawMsg(LEFT_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, yawMsg);
    mySimpit.printToKSP("Left Key Pressed", PRINT_TO_SCREEN);
    left_pressed = true;
  } else if (readingYaw <= (512 + DEADZONE) && left_pressed) {
    keyboardEmulatorMessage yawMsg(LEFT_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, yawMsg);
    mySimpit.printToKSP("Left Key Released", PRINT_TO_SCREEN);
    left_pressed = false;
  }

  // Handle yaw (Right)
  if (readingYaw < (512 - DEADZONE) && !right_pressed) {
    keyboardEmulatorMessage yawMsg(RIGHT_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, yawMsg);
    mySimpit.printToKSP("Right Key Pressed", PRINT_TO_SCREEN);
    right_pressed = true;
  } else if (readingYaw >= (512 - DEADZONE) && right_pressed) {
    keyboardEmulatorMessage yawMsg(RIGHT_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, yawMsg);
    mySimpit.printToKSP("Right Key Released", PRINT_TO_SCREEN);
    right_pressed = false;
  }
}


void sendPitchCommands() {
  int readingPitch = analogRead(PITCH_PIN);

  // Handle pitch (Up)
  if (readingPitch > (512 + DEADZONE) && !pitch_up_pressed) {
    keyboardEmulatorMessage pitchMsg(PITCH_UP_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, pitchMsg);
    mySimpit.printToKSP("Pitch Up Key Pressed", PRINT_TO_SCREEN);
    pitch_up_pressed = true;
  } else if (readingPitch <= (512 + DEADZONE) && pitch_up_pressed) {
    keyboardEmulatorMessage pitchMsg(PITCH_UP_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, pitchMsg);
    mySimpit.printToKSP("Pitch Up Key Released", PRINT_TO_SCREEN);
    pitch_up_pressed = false;
  }

  // Handle pitch (Down)
  if (readingPitch < (512 - DEADZONE) && !pitch_down_pressed) {
    keyboardEmulatorMessage pitchMsg(PITCH_DOWN_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, pitchMsg);
    mySimpit.printToKSP("Pitch Down Key Pressed", PRINT_TO_SCREEN);
    pitch_down_pressed = true;
  } else if (readingPitch >= (512 - DEADZONE) && pitch_down_pressed) {
    keyboardEmulatorMessage pitchMsg(PITCH_DOWN_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, pitchMsg);
    mySimpit.printToKSP("Pitch Down Key Released", PRINT_TO_SCREEN);
    pitch_down_pressed = false;
  }
}

void sendZoomCommands() {
  int readingZoom = analogRead(ZOOM_PIN);

  // Handle zoom (In) - Moving joystick to the right
  if (readingZoom > (512 + DEADZONE) && !zoom_in_pressed) {
    keyboardEmulatorMessage zoomMsg(ZOOM_IN_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, zoomMsg);
    mySimpit.printToKSP("Zoom In Key Pressed", PRINT_TO_SCREEN);
    zoom_in_pressed = true;
  } else if (readingZoom <= (512 + DEADZONE) && zoom_in_pressed) {
    keyboardEmulatorMessage zoomMsg(ZOOM_IN_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, zoomMsg);
    mySimpit.printToKSP("Zoom In Key Released", PRINT_TO_SCREEN);
    zoom_in_pressed = false;
  }

  // Handle zoom (Out) - Moving joystick to the left
  if (readingZoom < (512 - DEADZONE) && !zoom_out_pressed) {
    keyboardEmulatorMessage zoomMsg(ZOOM_OUT_KEY, KEY_DOWN_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, zoomMsg);
    mySimpit.printToKSP("Zoom Out Key Pressed", PRINT_TO_SCREEN);
    zoom_out_pressed = true;
  } else if (readingZoom >= (512 - DEADZONE) && zoom_out_pressed) {
    keyboardEmulatorMessage zoomMsg(ZOOM_OUT_KEY, KEY_UP_MOD);
    mySimpit.send(KEYBOARD_EMULATOR, zoomMsg);
    mySimpit.printToKSP("Zoom Out Key Released", PRINT_TO_SCREEN);
    zoom_out_pressed = false;
  }
}

