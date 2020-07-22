#include <EEPROM.h>

#include <Plan13.h>

//#include <avr/sleep.h>
//#include <avr/power.h>
#define ONEPPM 1.0e-6
#define DEBUG true
Plan13 p13;
void setup () {
 p13.setFrequency(435300000, 145920000);//AO-51  frequency
  p13.setLocation(-64.375, 45.8958, 20); // Sackville, NB
Serial.begin(38400);
}
void loop() { 
  p13.setTime(2009, 10, 1, 19, 5, 0); //Oct 1, 2009 19:05:00 UTC
  p13.setElements(2009, 232.55636497, 98.0531, 238.4104, 83652*1.0e-7, 290.6047,
  68.6188, 14.406497342, -0.00000001, 27022, 180.0); //fairly recent keps for AO-51 //readElements();
  p13.initSat();
  p13.satvec();
  p13.rangevec();
  p13.printdata();
  Serial.println();
  Serial.println("Should be:");
  Serial.println("AZ:57.07 EL: 4.05 RX 435301728 TX 145919440");
 exit(1);
}




