#include <digitalWriteFast.h>
#define pinNum 9
//int pinNum = 9; //do not use variables, macro will revert to the slower digitalWrite()
//const int pinNum = 9; //this is a constant, will use port manipulation (fast)
void setup() {
  pinModeFast(pinNum, OUTPUT);
}

void loop() {
//the pin is toggled multiple time before looping is because it took too long that the pin stayed low for 600ns, while clearing or setting the pin only took 125ns. For 16MHz Arduino Uno.
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
  digitalWriteFast(pinNum, HIGH);
  digitalWriteFast(pinNum, LOW);
}