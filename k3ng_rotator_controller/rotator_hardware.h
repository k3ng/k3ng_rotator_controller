/* rotator_hardware.h

   Uncomment defines below if you're specifically using any of this hardware and edit the appropriate special features, pins, and settings files

*/

// #define HARDWARE_M0UPU          // customize rotator_features_m0upu.h, rotators_pins_m0upu.h, rotator_settings_m0upu.h
// #define HARDWARE_EA4TX_ARS_USB  // customize rotator_features_e4tx_ars_usb.h, rotators_pins_e4tx_ars_usb.h, rotator_settings_e4tx_ars_usb.h
// #define HARDWARE_WB6KCN         // customize rotator_features_wb6kcn.h, rotators_pins_wb6kcn.h, rotator_settings_wb6kcn.h
// #define HARDWARE_TEST         // customize rotator_features_test.h, rotators_pins_test.h, rotator_settings_test.h


// do not modify anything below this line

#if defined(HARDWARE_M0UPU) || defined(HARDWARE_EA4TX_ARS_USB) || defined(HARDWARE_WB6KCN) || defined(HARDWARE_TEST)
  #define HARDWARE_CUSTOM
#endif


// Serial port class definitions for various devices

#if defined(ARDUINO_MAPLE_MINI)
  #define SERIAL_PORT_CLASS USBSerial
#elif defined(ARDUINO_AVR_PROMICRO) || defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_ESPLORA) || defined(ARDUINO_AVR_LILYPAD_USB) || defined(ARDUINO_AVR_ROBOT_CONTROL) || defined(ARDUINO_AVR_ROBOT_MOTOR) || defined(ARDUINO_AVR_LEONARDO_ETH)
  #define SERIAL_PORT_CLASS Serial_
#elif defined(TEENSYDUINO)
  #define SERIAL_PORT_CLASS usb_serial_class
#else
  #define SERIAL_PORT_CLASS HardwareSerial
#endif