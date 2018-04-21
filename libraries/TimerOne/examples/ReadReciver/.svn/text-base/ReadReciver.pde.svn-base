/*
Copyright 2011 Lex Talionis (Lex.V.Talionis at gmail)
This program is free software: you can redistribute it 
and/or modify it under the terms of the GNU General Public 
License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any 
later version.

This uses pin change interrupts and timer 1 to mesure the 
time between the rise and fall of 3 channels of PPM 
(Though often called PWM, see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1253149521/all)
on a typical RC car reciver.  It could be extended to as
many channels as you like.

*/
#include <PinChangeInt.h>	// http://www.arduino.cc/playground/Main/PinChangeInt
#include <PinChangeIntConfig.h>
#include <TimerOne.h>		// http://www.arduino.cc/playground/Code/Timer1

#define NO_PORTB_PINCHANGES //PinChangeInt setup
#define NO_PORTC_PINCHANGES	//only port D pinchanges (see: http://www.arduino.cc/playground/Learning/Pins)
#define PIN_COUNT 3	//number of channels attached to the reciver
#define	MAX_PIN_CHANGE_PINS PIN_COUNT

#define RC_TURN 3	//arduino pins attached to the reciver
#define RC_FWD 2
#define RC_FIRE 4
byte pin[] = {RC_FWD, RC_TURN, RC_FIRE};	//for maximum efficency thise pins should be attached
unsigned int time[] = {0,0,0};				// to the reciver's channels in the order listed here

byte state=0;
byte burp=0;    // a counter to see how many times the int has executed
byte cmd=0;     // a place to put our serial data
byte i=0;       // global counter for tracking what pin we are on

void setup() {
	Serial.begin(115200);
	Serial.print("PinChangeInt ReciverReading test");
	Serial.println();			//warm up the serial port
	
	Timer1.initialize(2200);	//longest pulse in PPM is usally 2.1 milliseconds,
								//pick a period that gives you a little headroom.
	Timer1.stop();				//stop the counter
	Timer1.restart();			//set the clock to zero
	
	for (byte i=0; i<3; i++)
	{
		pinMode(pin[i], INPUT);     //set the pin to input
		digitalWrite(pin[i], HIGH); //use the internal pullup resistor
	}
	PCintPort::attachInterrupt(pin[i], rise,RISING); // attach a PinChange Interrupt to our first pin
}

void loop() {
	
	cmd=Serial.read();		//while you got some time gimme a systems report
	if (cmd=='p')
	{
		Serial.print("time:\t");
		for (byte i=0; i<PIN_COUNT;i++)
		{
			Serial.print(i,DEC);
			Serial.print(":");
			Serial.print(time[i],DEC);
			Serial.print("\t");
		}
		Serial.print(burp, DEC);
		Serial.println();
/*		Serial.print("\t");
		Serial.print(clockCyclesToMicroseconds(Timer1.pwmPeriod), DEC);
		Serial.print("\t");
		Serial.print(Timer1.clockSelectBits, BIN);
		Serial.print("\t");
		Serial.println(ICR1, DEC);*/
	}
	cmd=0;	
	
	switch (state)
	{
		case RISING: //we have just seen a rising edge
			PCintPort::detachInterrupt(pin[i]);
			PCintPort::attachInterrupt(pin[i], fall, FALLING); //attach the falling end
			state=255;
			break;
		case FALLING: //we just saw a falling edge
			PCintPort::detachInterrupt(pin[i]);
			i++;				//move to the next pin
			i = i % PIN_COUNT;  //i ranges from 0 to PIN_COUNT
			PCintPort::attachInterrupt(pin[i], rise,RISING);
			state=255;
			break;
		/*default:
			//do nothing
			break;*/
	}
}

void rise()		//on the rising edge of the currently intresting pin
{
	Timer1.restart();		//set our stopwatch to 0
	Timer1.start();			//and start it up
	state=RISING;
//	Serial.print('r');
	burp++;
}

void fall()		//on the falling edge of the signal
{
	state=FALLING;

	time[i]=Timer1.read();	// Needs Timer1-v2
	Timer1.stop();
//	Serial.print('f');
}
