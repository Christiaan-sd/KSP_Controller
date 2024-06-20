/*
  SparkFun Inventor’s Kit
  Circuit 2B-ButtonTrumpet

  Use 3 buttons plugged to play musical notes on a buzzer.

  This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
  This code is completely free for any use.

  View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v41
  Download drawings and code at: https://github.com/sparkfun/SIK-Guide-Code
*/

//set the pins for the button and buzzer
int SASswitchpin = 2;
int SASledPin = 13;


void setup() {
  Serial.begin(9600);
  //set the button pins as inputs
  pinMode(SASswitchpin, INPUT_PULLUP);
  pinMode(SASledPin, OUTPUT);

}

void loop() {


  if (digitalRead(SASswitchpin) == HIGH) { 
    digitalWrite(SASledPin,LOW);
      Serial.write("HIGH");
      Serial.println();


  }

  if (digitalRead(SASswitchpin) == LOW) {
    digitalWrite(SASledPin,HIGH);
      Serial.write("LOW");
      Serial.println();
  }

}


