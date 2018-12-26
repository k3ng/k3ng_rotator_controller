
#ifndef _K3NGDISPLAY_H
#define _K3NGDISPLAY_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*


                  Set the display type here !!!!!!!!!


                                                              */

#define FEATURE_4_BIT_LCD_DISPLAY
// #define FEATURE_ADAFRUIT_I2C_LCD
// #define FEATURE_YOURDUINO_I2C_LCD
// #define FEATURE_RFROBOT_I2C_DISPLAY
// #define FEATURE_YWROBOT_I2C_DISPLAY
// #define FEATURE_SAINSMART_I2C_LCD
// #define FEATURE_ADAFRUIT_BUTTONS   
// #define FEATURE_MIDAS_I2C_DISPLAY
// #define FEATURE_FABO_LCD_PCF8574_DISPLAY  
// #define FEATURE_HD44780_I2C_DISPLAY           

// #define OPTION_RFROBOT_I2C_DISPLAY_BACKLIGHT_OFF

#include "rotator_hardware.h"

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

#ifdef HARDWARE_EA4TX_ARS_USB
  #include "rotator_pins_ea4tx_ars_usb.h"
#endif
#ifdef HARDWARE_M0UPU
  #include "rotator_pins_m0upu.h"
#endif
#ifdef HARDWARE_WB6KCN
  #include "rotator_pins_wb6kcn.h"
#endif
#ifdef HARDWARE_TEST
  #include "rotator_pins_test.h"
#endif
#if !defined(HARDWARE_CUSTOM)
  #include "rotator_pins.h"
#endif

#if defined(FEATURE_ADAFRUIT_I2C_LCD)
  #include "rotator.h"
#endif

#define K3NG_DISPLAY_LIBRARY_VERSION "2018.11.21.01"
#define MAX_SCREEN_BUFFER_COLUMNS 20
#define MAX_SCREEN_BUFFER_ROWS 4

#define ATTRIBUTE_BLINK B00000001

#define TEXT_BLINK_MS 500
#define WORK_STRING_SIZE 32

#define I2C_LCD_COLOR WHITE            // default color of I2C LCD display, including Adafruit and Yourduino; some Yourduino units may want this as LED_ON                  
// #define I2C_LCD_COLOR GREEN                  
// #define I2C_LCD_COLOR LED_ON

class K3NGdisplay {

public:

    K3NGdisplay(int display_columns, int display_rows, int _update_time);
    void initialize();
    void service(uint8_t force_update_flag);  // write pending changes to the screen periodically and blink text that has the blink attribute
    void clear();   // clear the display immediately
    void clear_pending_buffer();
    void update();  // update pending changes to the screen
    void print(char * print_string);
    void print(char * print_string,int x,int y);
    void print(char * print_string,int x,int y, uint8_t text_attribute);
    void print_attribute(char * print_string, uint8_t text_attribute);
    void print_attribute(char * print_string,int x,int y, uint8_t text_attribute);
    void print_center(char * print_string,int y);
    void print_center_padded(char * print_string,int y,int padding);
    void print_center_fixed_field_size(char * print_string,int y,int field_size);
    void print_center_entire_row(char * print_string,int y,uint8_t text_attribute);
    void print_center(char * print_string,int y,uint8_t text_attribute);
    void print_center_screen(char * print_string);
    void print_center_screen(char * print_string,uint8_t text_attribute);
    void print_center_screen(char * print_string,char * print_string2);
    void print_center_screen(char * print_string,char * print_string2,char * print_string3);
    void print_center_screen(char * print_string,char * print_string2,char * print_string3,char * print_string4);
    void print_right(char * print_string,int y);
    void print_right_padded(char * print_string,int y,int padding);
    void print_right_fixed_field_size(char * print_string,int y,int field_size);
    void print_left(char * print_string,int y);
    void print_left_padded(char * print_string,int y,int padding);
    void print_left_fixed_field_size(char * print_string,int y,int field_size);
    void print_top_left(char * print_string);
    void print_top_right(char * print_string);
    void print_bottom_left(char * print_string);
    void print_bottom_right(char * print_string); 

    /* print a timed message in the center of the screen; this can be multiline */

    void print_center_timed_message(char * print_string,int ms_to_display);  
    void print_center_timed_message(char * print_string,int ms_to_display,uint8_t text_attribute);  // TODO - add multiline timed attribute prints
    void print_center_timed_message(char * print_string,char * print_string2,int ms_to_display);
    void print_center_timed_message(char * print_string,char * print_string2,char * print_string3,int ms_to_display);
    void print_center_timed_message(char * print_string,char * print_string2,char * print_string3,char * print_string4,int ms_to_display);

    void redraw();  // redraw the entire screen
    void row_scroll();
    void println(char * print_string);
    int length(char * print_string);
    void clear_row(uint8_t row_number);  // clear one entire row

    #if defined(FEATURE_ADAFRUIT_BUTTONS)
      uint8_t readButtons();
    #endif   

private:

	void save_current_screen_to_revert_screen_buffer();  // used by the timed message functionality to push the current screen to a buffer for saving
	//void push_revert_screen_buffer_to_live_buffer();  // used by the timed message functionality to pull a saved screen and push to the live display
    void push_revert_screen_buffer_to_pending_buffer();
	void revert_back_screen();                           // used by the timed message functionality to pull a saved screen and push to the live display
    void prepare_for_timed_screen(int ms_to_display);
    int Xposition(int screen_buffer_index);
    int Yposition(int screen_buffer_index);
    int buffer_index_position(int x,int y);

};

#endif //_K3NGDISPLAY_H

