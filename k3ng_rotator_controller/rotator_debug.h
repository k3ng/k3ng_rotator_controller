// debug.h  contributed by Matt VK5ZM

#ifndef _ROTATOR_DEBUG_h
#define _ROTATOR_DEBUG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "rotator.h"

#ifdef HARDWARE_EA4TX_ARS_USB
  #include "rotator_features_ea4tx_ars_usb.h"
#endif
#ifdef HARDWARE_WB6KCN
  #include "rotator_features_wb6kcn.h"
#endif
#ifdef HARDWARE_M0UPU
  #include "rotator_features_m0upu.h"
#endif
#ifdef HARDWARE_TEST
  #include "rotator_features_test.h"
#endif    
#if !defined(HARDWARE_CUSTOM)
  #include "rotator_features.h" 
#endif 

#include "rotator_hardware.h"

// Serial port class definitions for various devices

// #if defined(ARDUINO_MAPLE_MINI)
//   #define SERIAL_PORT_CLASS USBSerial
// #elif defined(ARDUINO_AVR_PROMICRO) || defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_ESPLORA) || defined(ARDUINO_AVR_LILYPAD_USB) || defined(ARDUINO_AVR_ROBOT_CONTROL) || defined(ARDUINO_AVR_ROBOT_MOTOR) || defined(ARDUINO_AVR_LEONARDO_ETH)
//   #define SERIAL_PORT_CLASS Serial_
// #elif defined(TEENSYDUINO)
//   #define SERIAL_PORT_CLASS usb_serial_class
// #else
//   #define SERIAL_PORT_CLASS HardwareSerial
// #endif

// #ifdef FEATURE_ETHERNET
//   #include <SPI.h>
//   #include <Ethernet.h>
// #endif

class DebugClass
{
 protected:


 public:
	void init();

	void print(const char *str);
	void print(const __FlashStringHelper *str);
	void print(char ch);
	void print(int i);
	void print(float f);
	void print(float f, byte places);
	void print(unsigned int i);
	void print(long unsigned int i);
	void print(long i);
	void print(double i);
	
	void println(double i);
	void println(const char *str);
	void println(const __FlashStringHelper *str);
	
	void write(const char *str);
	void write(int i);
};


extern uint8_t debug_mode;
extern SERIAL_PORT_CLASS * control_port;

// #ifdef FEATURE_ETHERNET
//   extern EthernetClient ethernetclient0;
// #endif

#endif //_ROTATOR_DEBUG_h

