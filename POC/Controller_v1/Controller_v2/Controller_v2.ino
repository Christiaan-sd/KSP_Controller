/* KerbalSimpitActionSwitch
   A demonstration of using two switches to command both SAS and RCS.
   In this example, the KSP state is monitored to update the SAS and RCS
   to match the switch state. This ensure that when using the keyboard or when
   switching vessels, the game will still match the switch positions.
   In addition, the LED_BUILTIN matches the SAS state.

*/
#include "KerbalSimpit.h"

const int THROTTLE_PIN = A0; // the pin used for controlling throttle
const int SAS_SWITCH_PIN = 3; // the pin used for controlling SAS.
const int RCS_SWITCH_PIN = 4; // the pin used for controlling RCS.
const int STAGE_BUTTON_PIN = 2;    // the number of the pushbutton pin
const int ABORT_BUTTON_PIN = 6;    // the number of the pushbutton pin
const int GEAR_SWITCH_PIN = 5; // the pin used for controlling the gears.
const int BRAKE_SWITCH_PIN = 7; // the pin used for controlling RCS.

//Store the current action status, as recevied by simpit.
byte currentActionStatus = 0;

// Declare a KerbalSimpit object that will communicate using the "Serial" device.
KerbalSimpit mySimpit(Serial);

void setup() {
  // Open the serial connection.
  Serial.begin(115200);

  // Set up the build in LED, and turn it on.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Set up the two switches with builtin pullup.
  pinMode(SAS_SWITCH_PIN, INPUT_PULLUP);
  pinMode(RCS_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STAGE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(GEAR_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BRAKE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ABORT_BUTTON_PIN, INPUT_PULLUP);

  // This loop continually attempts to handshake with the plugin.
  // It will keep retrying until it gets a successful handshake.
  while (!mySimpit.init()) {
    delay(100);
  }
  // Turn off the built-in LED to indicate handshaking is complete.
  digitalWrite(LED_BUILTIN, LOW);
  // Display a message in KSP to indicate handshaking is complete.
  mySimpit.printToKSP("Connected", PRINT_TO_SCREEN);
  // Sets our callback function. The KerbalSimpit library will
  // call this function every time a packet is received.
  mySimpit.inboundHandler(messageHandler);
  // Send a message to the plugin registering for the Action status channel.
  // The plugin will now regularly send Action status messages while the
  // flight scene is active in-game.
  mySimpit.registerChannel(ACTIONSTATUS_MESSAGE);
}

void loop() {
  // Check for new serial messages.
  mySimpit.update();

// --------------------- Switches ----------------------------//
// --------------------- SAS ----------------------------//
  // Get the SAS switch state
  bool sas_switch_state = digitalRead(SAS_SWITCH_PIN);

  // Update the SAS to match the state, only if a change is needed to avoid
  // spamming commands.
  if(sas_switch_state && !(currentActionStatus & SAS_ACTION)){
    mySimpit.printToKSP("Activate SAS!");
    mySimpit.activateAction(SAS_ACTION);
  }
  if(!sas_switch_state && (currentActionStatus & SAS_ACTION)){
    mySimpit.printToKSP("Desactivate SAS!");
    mySimpit.deactivateAction(SAS_ACTION);
  }

// --------------------- Switches ----------------------------//
// --------------------- RCS ----------------------------//
  // Get the RCS switch state
  bool rcs_switch_state = digitalRead(RCS_SWITCH_PIN);

  // Update the RCS to match the state, only if a change is needed to avoid
  // spamming commands.
  if(rcs_switch_state && !(currentActionStatus & RCS_ACTION)){
    mySimpit.printToKSP("Activate RCS!");
    mySimpit.activateAction(RCS_ACTION);
  }
  if(!rcs_switch_state && (currentActionStatus & RCS_ACTION)){
    mySimpit.printToKSP("Desactivate RCS!");
    mySimpit.deactivateAction(RCS_ACTION);
  }

// --------------------- Switches ----------------------------//
// --------------------- GEAR ----------------------------//
  // Get the RCS switch state
  bool gear_switch_state = digitalRead(GEAR_SWITCH_PIN);

  // Update the RCS to match the state, only if a change is needed to avoid
  // spamming commands.
  if(gear_switch_state && !(currentActionStatus & GEAR_ACTION)){
    mySimpit.printToKSP("Activate GEAR!");
    mySimpit.activateAction(GEAR_ACTION);
  }
  if(!gear_switch_state && (currentActionStatus & GEAR_ACTION)){
    mySimpit.printToKSP("Desactivate GEAR!");
    mySimpit.deactivateAction(GEAR_ACTION);
  }

  // --------------------- Switches ----------------------------//
// --------------------- GEAR ----------------------------//
  // Get the BRAKE switch state
  bool brake_switch_state = digitalRead(BRAKE_SWITCH_PIN);

  // Update the BRAKE to match the state, only if a change is needed to avoid
  // spamming commands.
  if(brake_switch_state && !(currentActionStatus & BRAKES_ACTION)){
    mySimpit.printToKSP("Activate BRAKE!");
    mySimpit.activateAction(BRAKES_ACTION);
  }
  if(!brake_switch_state && (currentActionStatus & BRAKES_ACTION)){
    mySimpit.printToKSP("Desactivate BRAKE!");
    mySimpit.deactivateAction(BRAKES_ACTION);
  }


// --------------------- BUTTONS ----------------------------//
// --------------------- STAGE ----------------------------// 
// Get the STAGE button state
  bool stage_switch_state = digitalRead(STAGE_BUTTON_PIN);
  // Update the RCS to match the state, only if a change is needed to avoid
  // spamming commands.
  if(stage_switch_state == HIGH){
    mySimpit.printToKSP("NO STAGE!");

  } else {
    mySimpit.printToKSP("STAGE!");
    mySimpit.activateAction(STAGE_ACTION);
    delay(500);
  }

  // --------------------- BUTTONS ----------------------------//
// --------------------- ABORT ----------------------------// 
// Get the STAGE button state
  bool abort_switch_state = digitalRead(ABORT_BUTTON_PIN);
  // Update the RCS to match the state, only if a change is needed to avoid
  // spamming commands.
  if(abort_switch_state == HIGH){
    mySimpit.printToKSP("NO ABORT!");

  } else {
    mySimpit.printToKSP("ABORT!");
    mySimpit.activateAction(ABORT_ACTION);
    delay(500);
  }


// --------------------- Potentiometers ----------------------------//
// --------------------- throttle ----------------------------//
// Send at each loop a message to control the throttle 
  throttleMessage throttle_msg;
  // Read the value of the potentiometer
  int reading = analogRead(THROTTLE_PIN);
  // Convert it in KerbalSimpit Range
  throttle_msg.throttle = map(reading, 0, 1023, 0, INT16_MAX);
  // Send the message
  mySimpit.send(THROTTLE_MESSAGE, throttle_msg);


}
// END OF LOOP!

// START FUNCTIONS
void messageHandler(byte messageType, byte msg[], byte msgSize) {
  switch(messageType) {
  case ACTIONSTATUS_MESSAGE:
    // Checking if the message is the size we expect is a very basic
    // way to confirm if the message was received properly.
    if (msgSize == 1) {
      currentActionStatus = msg[0];

// BUTTON & SIWTCHES LEDS!
      //Let the LED_BUILIN match the current SAS state
      if(currentActionStatus & SAS_ACTION){
        digitalWrite(13, HIGH);
      } else {
        digitalWrite(13, LOW);
      }
      if(currentActionStatus & RCS_ACTION){
        digitalWrite(10, HIGH);
      } else {
        digitalWrite(10, LOW);
      }
    }
    break;
  }
}
