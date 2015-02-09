#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "hh12.h"

/*

Code adapted from here: http://www.madscientisthut.com/forum_php/viewtopic.php?f=11&t=7

Updated 2015-02-07 for 12 bit readings - Thanks Johan PA3FPQ


*/


int hh12_clock_pin = 0;
int hh12_cs_pin = 0;
int hh12_data_pin = 0;

int inputstream = 0; //one bit read from pin
long packeddata = 0; //two bytes concatenated from inputstream
long angle = 0; //holds processed angle value
float floatangle = 0;
#ifdef OPTION_HH12_10_BIT_READINGS
long anglemask = 65472; //0x1111111111000000: mask to obtain first 10 digits with position info
#else
long anglemask = 262080; // 0x111111111111000000: mask to obtain first 12 digits with position info
#endif //OPTION_HH12_10_BIT_READINGS
long statusmask = 63; //0x000000000111111; mask to obtain last 6 digits containing status info
long statusbits; //holds status/error information
int DECn; //bit holding decreasing magnet field error data
int INCn; //bit holding increasing magnet field error data
int OCF; //bit holding startup-valid bit
int COF; //bit holding cordic DSP processing error data
int LIN; //bit holding magnet field displacement error data

//-----------------------------------------------------------------------------------------------------
hh12::hh12(){


}
//-----------------------------------------------------------------------------------------------------
void hh12::initialize(int _hh12_clock_pin, int _hh12_cs_pin, int _hh12_data_pin){

  hh12_clock_pin = _hh12_clock_pin;
  hh12_cs_pin = _hh12_cs_pin;
  hh12_data_pin = _hh12_data_pin;

  pinMode(hh12_clock_pin, OUTPUT);
  pinMode(hh12_cs_pin, OUTPUT);
  pinMode(hh12_data_pin, INPUT);

}

//-----------------------------------------------------------------------------------------------------
float hh12::heading(){

  digitalWrite(hh12_cs_pin, HIGH); // CSn high
  digitalWrite(hh12_clock_pin, HIGH); // CLK high
  digitalWrite(hh12_cs_pin, LOW); // CSn low: start of transfer
  delayMicroseconds(HH12_DELAY); // delay for chip initialization
  digitalWrite(hh12_clock_pin, LOW); // CLK goes low: start clocking
  delayMicroseconds(HH12_DELAY); // hold low
#ifdef OPTION_HH12_10_BIT_READINGS
  for (int x=0; x <16; x++) // clock signal, 16 transitions, output to clock pin
#else
  for (int x=0; x <18; x++) // clock signal, 18 transitions, output to clock pin
#endif //OPTION_HH12_10_BIT_READINGS
  {
    digitalWrite(hh12_clock_pin, HIGH); //clock goes high
    delayMicroseconds(HH12_DELAY); // 
    inputstream =digitalRead(hh12_data_pin); // read one bit of data from pin
    #ifdef DEBUG_HH12
    Serial.print(inputstream, DEC);
    #endif
    packeddata = ((packeddata << 1) + inputstream);// left-shift summing variable, add pin value
    digitalWrite(hh12_clock_pin, LOW);
    delayMicroseconds(HH12_DELAY); // end of one clock cycle
  }
  // end of entire clock cycle



  #ifdef DEBUG_HH12
  Serial.print("hh12: packed:");
  Serial.println(packeddata,DEC);
  Serial.print("hh12: pack bin: ");
  Serial.println(packeddata,BIN);
  #endif

  angle = packeddata & anglemask; // mask rightmost 6 digits of packeddata to zero, into angle

  #ifdef DEBUG_HH12
  Serial.print("hh12: mask: ");
  Serial.println(anglemask, BIN);
  Serial.print("hh12: bin angle:");
  Serial.println(angle, BIN);
  Serial.print("hh12: angle: ");
  Serial.println(angle, DEC);
  #endif

  angle = (angle >> 6); // shift 16-digit angle right 6 digits to form 10-digit value

  #ifdef DEBUG_HH12
  Serial.print("hh12: angleshft:");
  Serial.println(angle, BIN);
  Serial.print("hh12: angledec: ");
  Serial.println(angle, DEC);
  #endif

#ifdef OPTION_HH12_10_BIT_READINGS
  floatangle = angle * 0.3515; // angle * (360/1024) == actual degrees
#else
  floatangle = angle * 0.08789; // angle * (360/4096) == actual degrees
#endif //OPTION_HH12_10_BIT_READINGS
  
  #ifdef DEBUG_HH12
  statusbits = packeddata & statusmask;
  DECn = statusbits & 2; // goes high if magnet moved away from IC
  INCn = statusbits & 4; // goes high if magnet moved towards IC
  LIN = statusbits & 8; // goes high for linearity alarm
  COF = statusbits & 16; // goes high for cordic overflow: data invalid
  OCF = statusbits & 32; // this is 1 when the chip startup is finished
  if (DECn && INCn) { 
    Serial.println("hh12: magnet moved out of range"); 
  } else {
    if (DECn) { 
      Serial.println("hh12: magnet moved away from chip"); 
    }
    if (INCn) { 
      Serial.println("hh12: magnet moved towards chip"); 
    }
  }
  if (LIN) { 
    Serial.println("hh12: linearity alarm: magnet misaligned? data questionable"); 
  }
  if (COF) { 
    Serial.println("hh12: cordic overflow: magnet misaligned? data invalid"); 
  }
  #endif //DEBUG_HH12


  return(floatangle);


}

