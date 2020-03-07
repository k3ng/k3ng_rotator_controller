#ifndef K3NG_DISPLAY_H
#define K3NG_DISPLAY_H

// K3NG_DISPLAY_LIBRARY_VERSION "2018.11.21.01"


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "rotator_k3ngdisplay.h"




#ifdef FEATURE_4_BIT_LCD_DISPLAY
  #include <LiquidCrystal.h>
  LiquidCrystal lcd(lcd_4_bit_rs_pin, lcd_4_bit_enable_pin, lcd_4_bit_d4_pin, lcd_4_bit_d5_pin, lcd_4_bit_d6_pin, lcd_4_bit_d7_pin);
#endif // FEATURE_4_BIT_LCD_DISPLAY

#ifdef FEATURE_WIRE_SUPPORT
  #include <Wire.h>
#endif
#if defined(FEATURE_ADAFRUIT_I2C_LCD)
  #include <Adafruit_MCP23017.h>
  #include <Adafruit_RGBLCDShield.h>
  Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#endif
#if defined(FEATURE_YOURDUINO_I2C_LCD) || defined(FEATURE_RFROBOT_I2C_DISPLAY) || defined(FEATURE_SAINSMART_I2C_LCD)
  #include <LiquidCrystal_I2C.h> 
#endif
#if defined(FEATURE_YOURDUINO_I2C_LCD)
  #include <LCD.h>
#endif  
#if defined(FEATURE_MIDAS_I2C_DISPLAY)
  //#include <lcd.h>
  #include <LCD_C0220BiZ.h>
  #include <ST7036.h>
#endif
#if defined(FEATURE_FABO_LCD_PCF8574_DISPLAY)
  #include <FaBoLCD_PCF8574.h>
#endif
#if defined(FEATURE_HD44780_I2C_DISPLAY)
  #include <hd44780.h>                       // main hd44780 header
  #include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#endif  


#if defined(FEATURE_YOURDUINO_I2C_LCD)
  #define I2C_ADDR 0x20
  #define BACKLIGHT_PIN 3
  #define LED_OFF 1
  #define LED_ON 0            
  LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
#endif //FEATURE_YOURDUINO_I2C_LCD

#if defined(FEATURE_RFROBOT_I2C_DISPLAY)
  LiquidCrystal_I2C lcd(0x27,16,2); 
  // LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif //FEATURE_RFROBOT_I2C_DISPLAY


#if defined(FEATURE_YWROBOT_I2C_DISPLAY)
  #include <LiquidCrystal_I2C.h>
  //LiquidCrystal_I2C lcd(ywrobot_address, ywrobot_pin_en, ywrobot_pin_rw, ywrobot_pin_rs, ywrobot_pin_d4, ywrobot_pin_d5, ywrobot_pin_d6, ywrobot_pin_d7, ywrobot_pin_bl, ywrobot_blpol);  
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif //FEATURE_YWROBOT_I2C_DISPLAY

#if defined(FEATURE_SAINSMART_I2C_LCD)
  #include <LiquidCrystal_I2C.h>
  #define I2C_ADDR      0x27
  #define BACKLIGHT_PIN 3
  #define En_pin        2
  #define Rw_pin        1
  #define Rs_pin        0
  #define D4_pin        4
  #define D5_pin        5
  #define D6_pin        6
  #define D7_pin        7
  LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin, BACKLIGHT_PIN, POSITIVE);  
#endif //FEATURE_SAINSMART_I2C_LCD

#if defined(FEATURE_MIDAS_I2C_DISPLAY)
  ST7036 lcd = ST7036 ( 2, 16, 120 );
#endif  

#if defined(FEATURE_FABO_LCD_PCF8574_DISPLAY)
  FaBoLCD_PCF8574 lcd;
#endif

#if defined(FEATURE_HD44780_I2C_DISPLAY)
  hd44780_I2Cexp lcd;
#endif  


int display_columns = 0;
uint8_t display_rows = 0;
char screen_buffer_live[MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS];
char screen_buffer_pending[MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS];
char screen_buffer_revert[MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS];
uint8_t screen_buffer_pending_dirty = 0;
uint8_t current_print_row = 0;
uint8_t current_print_column = 0;
unsigned long revert_screen_time = 0;
uint8_t revert_screen_flag = 0;
int update_time_ms = 0;
uint8_t last_blink_state = 0;
uint8_t timed_screen_changes_pending = 0;


uint8_t screen_buffer_attributes_live[MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS];
uint8_t screen_buffer_attributes_pending[MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS];
uint8_t screen_buffer_attributes_revert[MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS];

uint8_t current_blink_state = 0;
unsigned long next_blink_state_transition_time = TEXT_BLINK_MS;



//-----------------------------------------------------------------------------------------------------

K3NGdisplay::K3NGdisplay(int _display_columns, int _display_rows, int _update_time = 1000){
  
  display_columns = _display_columns;
  display_rows = _display_rows;
  update_time_ms = _update_time;

}

//-----------------------------------------------------------------------------------------------------

void K3NGdisplay::initialize(){

  #if !defined(FEATURE_MIDAS_I2C_DISPLAY)
  lcd.begin(display_columns, display_rows);  // if you are getting an error on this line and do not have
                                             // any of the LCD display features enabled, remove
                                             // k3ngdisplay.h and k3ngdisplay.cpp from your ino directory
  #endif

  #if defined(FEATURE_MIDAS_I2C_DISPLAY)
    lcd.init();
  #endif

  #if defined(FEATURE_FABO_LCD_PCF8574_DISPLAY)
    lcd.begin(display_columns, display_rows);
  #endif  

  #ifdef FEATURE_YOURDUINO_I2C_LCD
    lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
    lcd.setBacklight(I2C_LCD_COLOR);
  #endif // FEATURE_YOURDUINO_I2C_LCD

  #ifdef FEATURE_ADAFRUIT_I2C_LCD
    lcd.setBacklight(I2C_LCD_COLOR);
  #endif // FEATURE_ADAFRUIT_I2C_LCD

  #ifdef FEATURE_RFROBOT_I2C_DISPLAY
    #ifdef OPTION_RFROBOT_I2C_DISPLAY_BACKLIGHT_OFF
      lcd.noBacklight();
    #else
      lcd.backlight();
    #endif
  #endif

  #ifdef FEATURE_4_BIT_LCD_DISPLAY
    lcd.noCursor();
  #endif  

  clear();


}

//-----------------------------------------------------------------------------------------------------

void K3NGdisplay::service(uint8_t force_update_flag = 0){


  // force_update_flag = 1 : force a screen update regardless of update_time_ms, but not if there is a timed message (i.e. revert_screen_flag = 1)
  // force_update_flag = 2 : force a screen update regardless of update_time_ms and revert_screen_flag

  static unsigned long last_screen_buffer_pending_update_time = 0;

  if (revert_screen_flag){
    if (force_update_flag>1) {
      update();
      last_screen_buffer_pending_update_time = millis();
      revert_screen_flag = 0;
      screen_buffer_pending_dirty = 0;
      return;
    } else {
      if (millis() >= revert_screen_time){
        revert_back_screen();
        revert_screen_flag = 0;
        return;
      }      
    }
  } else {
    if (((screen_buffer_pending_dirty) && ((millis() - last_screen_buffer_pending_update_time) >= update_time_ms)) || (force_update_flag>0) ) {
      update();
      last_screen_buffer_pending_update_time = millis();
      screen_buffer_pending_dirty = 0;
    }    
  }

  // push an update immediately if we have a timed screen waiting to be displayed
  if (timed_screen_changes_pending){
    update();
    timed_screen_changes_pending = 0;
    screen_buffer_pending_dirty = 0;
    return;
  }

  // do we need to blink text?
  if (millis() >= next_blink_state_transition_time){
  	if (current_blink_state){
      current_blink_state = 0;
  	} else {
  	  current_blink_state = 1;
  	}
  	next_blink_state_transition_time = millis() + TEXT_BLINK_MS;
    redraw();  // TODO - check if there are actually any blink attributes, maybe separate out blink updates?
    return;
  }


}

//-----------------------------------------------------------------------------------------------------

void K3NGdisplay::clear_pending_buffer(){  

  // do an immediate clearing of the screen

  for (int x = 0;x < MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS;x++){
    screen_buffer_pending[x] = ' ';
    screen_buffer_attributes_pending[x] = 0;
  }

  screen_buffer_pending_dirty = 1;
}

//-----------------------------------------------------------------------------------------------------

void K3NGdisplay::clear(){  

  // do an immediate clearing of the screen

  for (int x = 0;x < MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS;x++){
    screen_buffer_live[x] = ' ';
    screen_buffer_pending[x] = ' ';
    screen_buffer_revert[x] = ' ';

    screen_buffer_attributes_live[x] = 0;
    screen_buffer_attributes_pending[x] = 0;
    screen_buffer_attributes_revert[x] = 0;

  }

  lcd.clear();

  #ifdef FEATURE_4_BIT_LCD_DISPLAY
    lcd.noCursor();
  #endif  
  
  current_print_row = 0;
  current_print_column = 0;
  revert_screen_flag = 0;

}

//-----------------------------------------------------------------------------------------------------

void K3NGdisplay::clear_row(uint8_t row_number){

  int x = 0;

  while ((buffer_index_position(x,row_number) < (display_columns * display_rows)) && (x < display_columns)) {
    screen_buffer_pending[buffer_index_position(x,row_number)] = ' ';
    screen_buffer_attributes_pending[buffer_index_position(x,row_number)] = 0;
    x++;
  }

  screen_buffer_pending_dirty = 1;


}
//-----------------------------------------------------------------------------------------------------
int K3NGdisplay::Xposition(int screen_buffer_index){

  return(screen_buffer_index % display_columns);
}
//-----------------------------------------------------------------------------------------------------
int K3NGdisplay::Yposition(int screen_buffer_index){

  return(screen_buffer_index / display_columns);
}

//-----------------------------------------------------------------------------------------------------

void K3NGdisplay::update(){

  // update the screen with changes that are pending in screen_buffer_pending

  lcd.noCursor();
  lcd.setCursor(0,0);

  for (int x = 0;x < (display_columns*display_rows);x++){  	
    if (screen_buffer_live[x] != screen_buffer_pending[x]){  // do we have a new character to put on the screen ?
      lcd.setCursor(Xposition(x),Yposition(x));
      if (screen_buffer_attributes_pending[x] & ATTRIBUTE_BLINK){  // does this character have the blink attribute
        if (current_blink_state){
          lcd.print(screen_buffer_pending[x]);
        } else {
          lcd.print(' ');
        }
      } else {
        lcd.print(screen_buffer_pending[x]);
      }
      screen_buffer_live[x] = screen_buffer_pending[x];
      screen_buffer_attributes_live[x] = screen_buffer_attributes_pending[x];
    } else {  // not a new character, do we have live character on the screen to blink?
      if (last_blink_state != current_blink_state){
        if (screen_buffer_attributes_live[x] & ATTRIBUTE_BLINK){
        	lcd.setCursor(Xposition(x),Yposition(x));
        	if (current_blink_state){
              lcd.print(screen_buffer_live[x]);
      	    } else {
      	      lcd.print(' ');
      	    }
        }
      }
    }
  }

  last_blink_state = current_blink_state;

}
//-----------------------------------------------------------------------------------------------------

void K3NGdisplay::redraw(){

  // redraw the screen with the current screen_buffer_live

  lcd.noCursor();
  lcd.setCursor(0,0);

  for (int x = 0;x < (display_columns*display_rows);x++){   
    lcd.setCursor(Xposition(x),Yposition(x));
    if (screen_buffer_attributes_live[x] & ATTRIBUTE_BLINK){  // does this character have the blink attribute
      if (current_blink_state){
        lcd.print(screen_buffer_live[x]);
      } else {
        lcd.print(' ');
      }
    } else {
      lcd.print(screen_buffer_live[x]);
    }
  }


}	

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print(char * print_string, int x, int y){

  print_attribute(print_string, x, y, 0);

}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print(char * print_string, int x, int y, uint8_t attribute){

  print_attribute(print_string, x, y, attribute);

}
//-----------------------------------------------------------------------------------------------------

int K3NGdisplay::buffer_index_position(int x,int y){

  return((y * display_columns) + x);

}



//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_attribute(char * print_string, int x, int y, uint8_t attribute){

  
  for (int print_string_index = 0;print_string_index < (display_columns * display_rows);print_string_index++){
  	if (print_string[print_string_index] != 0){
  	  if (((buffer_index_position(x,y)+print_string_index)) < (display_columns * display_rows)){
        screen_buffer_pending[buffer_index_position(x,y)+print_string_index] = print_string[print_string_index];
        screen_buffer_attributes_pending[buffer_index_position(x,y) + print_string_index] = attribute;
      }
    } else {
    	print_string_index = display_columns * display_rows;
    }
  }

  screen_buffer_pending_dirty = 1;

}



//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center(char * print_string,int y){


  print(print_string,((display_columns/*-1*/)/2)-(length(print_string)/2),y);
	
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_padded(char * print_string,int y,int padding){

  char workstring[WORK_STRING_SIZE] = "";

  for (int x = 0;(x < padding) && (strlen(workstring) < (WORK_STRING_SIZE-1));x++){
    strcat(workstring," ");
  }
  strcat(workstring,print_string);
  for (int x = 0;(x < padding) && (strlen(workstring) < (WORK_STRING_SIZE-1));x++){
    strcat(workstring," ");
  }
  print_center(workstring,y);

}


//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_fixed_field_size(char * print_string,int y,int field_size){

  char workstring[WORK_STRING_SIZE] = "";
  char workstring2[WORK_STRING_SIZE] = "";

  int spaces_to_add = field_size - strlen(print_string);

  if (spaces_to_add > 0){
    for (int x = 0;(x < (spaces_to_add/2)) && (strlen(workstring) < (WORK_STRING_SIZE-1));x++){
      strcat(workstring," ");
    }
  }
  strncpy(workstring2,print_string,field_size);
  strcat(workstring,workstring2); 
  if (spaces_to_add > 0){
    for (int x = 0;(x < (spaces_to_add/2)) && (strlen(workstring) < (WORK_STRING_SIZE-1));x++){
      strcat(workstring," ");
    }
    if ((spaces_to_add % 2) != 0){  // odd number means we have another space to add on the end
      strcat(workstring," ");
    }
  }
  print_center(workstring,y);

}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center(char * print_string,int y,uint8_t text_attribute){


  print_attribute(print_string,((display_columns/*-1*/)/2)-(length(print_string)/2),y,text_attribute);
  
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_entire_row(char * print_string,int y,uint8_t text_attribute){

  clear_row(y);
  print_attribute(print_string,((display_columns/*-1*/)/2)-(length(print_string)/2),y,text_attribute);
  
}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_screen(char * print_string){


  print_center(print_string,(display_rows-1)/2);
	
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_screen(char * print_string,uint8_t text_attribute){


  print_center(print_string,(display_rows-1)/2,text_attribute);
  
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_screen(char * print_string,char * print_string2){

  if (display_rows == 2){
    print_center(print_string,0);
    print_center(print_string2,1);
  } else {
    print_center(print_string,(display_rows/2)-1);
    print_center(print_string2,(display_rows/2));
  }
	
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_screen(char * print_string,char * print_string2,char * print_string3){

  if (display_rows == 4){
    print_center(print_string,0);
    print_center(print_string2,1);
    print_center(print_string3,2);
  } else {
    print_center(print_string,(display_rows/2)-1);
    print_center(print_string2,(display_rows/2));
    print_center(print_string3,(display_rows/2)+1);
  }
	
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_screen(char * print_string,char * print_string2,char * print_string3,char * print_string4){

  if (display_rows == 4){
  	print_center(print_string,0);
    print_center(print_string2,1);
    print_center(print_string3,2);
    print_center(print_string4,3);  	
  } else {
    print_center(print_string,(display_rows/2)-1);
    print_center(print_string2,(display_rows/2));
    print_center(print_string3,(display_rows/2)+1);
    print_center(print_string4,(display_rows/2)+3);
  }

	
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_right(char * print_string,int y){

	print(print_string,display_columns-length(print_string),y);
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_left(char * print_string,int y){

	print(print_string,0,y);
}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_left_padded(char * print_string,int y,int padding){


  char workstring[WORK_STRING_SIZE] = "";

  strcpy(workstring,print_string);
  for (int x = 0;(x < padding) && (strlen(workstring) < (WORK_STRING_SIZE-1));x++){
    strcat(workstring," ");
  }
  print_left(workstring,y);
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_left_fixed_field_size(char * print_string,int y,int field_size){

  char workstring[WORK_STRING_SIZE] = "";

  strncpy(workstring,print_string,field_size);
  print_left_padded(workstring, y, field_size - strlen(print_string));
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_right_fixed_field_size(char * print_string,int y,int field_size){

  char workstring[WORK_STRING_SIZE] = "";

  strncpy(workstring,print_string,field_size);
  print_right_padded(workstring, y, field_size - strlen(print_string));
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_right_padded(char * print_string,int y,int padding){


  char workstring[WORK_STRING_SIZE] = "";

  
  for (int x = 0;(x < padding) && (strlen(workstring) < (WORK_STRING_SIZE-1));x++){
    strcat(workstring," ");
  }
  strcat(workstring,print_string);
  print_right(workstring,y);
}


//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_top_right(char * print_string){

	print(print_string,display_columns-length(print_string),0);
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_top_left(char * print_string){

	print(print_string,0,0);
}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_bottom_right(char * print_string){

	print(print_string,display_columns-length(print_string),display_rows-1);
}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_bottom_left(char * print_string){

	print(print_string,0,display_rows-1);
}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::row_scroll(){


  for (int x = 0; x < display_columns; x++){
  	for (int y = 0; y < display_rows; y++){
      if (y < (display_rows-1)){      	
        screen_buffer_pending[buffer_index_position(x,y)] = screen_buffer_pending[buffer_index_position(x,y+1)];
        screen_buffer_attributes_pending[buffer_index_position(x,y)] = screen_buffer_attributes_pending[buffer_index_position(x,y+1)];
      } else {
      	screen_buffer_pending[buffer_index_position(x,y)] = ' ';
      	screen_buffer_attributes_pending[buffer_index_position(x,y)] = 0;
      } 
  	}
  }

  screen_buffer_pending_dirty = 1;


}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print(char * print_string){

  print_attribute(print_string, 0);



}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::println(char * print_string){


  char workstring[WORK_STRING_SIZE] = "";

  strcpy(workstring,print_string);
  if (strlen(workstring) < (WORK_STRING_SIZE-2)){
    strcat(workstring,"\n");
  }
  print_attribute(workstring, 0);

}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_attribute(char * print_string, uint8_t text_attribute){

  

  for (int print_string_index = 0;print_string_index < (display_columns * display_rows);print_string_index++){
  	switch (print_string[print_string_index]){
      case 0: return; break;
      case '\n':
        current_print_column = display_columns;
        break;
      default: 
	      if (current_print_column >= display_columns){
	        current_print_column = 0;
	        current_print_row++;
	      	if (current_print_row >= display_rows){ // we're at the end of the display 
	          row_scroll();
	          current_print_row--;
	      	}
	      }
	      screen_buffer_pending[buffer_index_position(current_print_column,current_print_row)] = print_string[print_string_index];
	      screen_buffer_attributes_pending[buffer_index_position(current_print_column,current_print_row)] = text_attribute;
	      current_print_column++;
	      break;
    }

  }

  screen_buffer_pending_dirty = 1;

}

//-----------------------------------------------------------------------------------------------------

int K3NGdisplay::length(char * print_string){

  int char_count = 0;

  for (int x = 0;x < (display_columns * display_rows);x++){
	if (print_string[x] != 0){
	  char_count++;
	} else {
	  x = display_columns * display_rows;
	}
  }

  return char_count;

}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::prepare_for_timed_screen(int ms_to_display){

  save_current_screen_to_revert_screen_buffer();
  revert_screen_flag = 1;
  revert_screen_time = millis() + ms_to_display;
  timed_screen_changes_pending = 1;

}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_timed_message(char * print_string,int ms_to_display,uint8_t text_attribute){

  prepare_for_timed_screen(ms_to_display);
  print_center_screen(print_string,text_attribute);

}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_timed_message(char * print_string,int ms_to_display){

  prepare_for_timed_screen(ms_to_display);
  print_center_screen(print_string);

}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_timed_message(char * print_string,char * print_string2,int ms_to_display){


  prepare_for_timed_screen(ms_to_display);
  print_center_screen(print_string,print_string2);

}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_timed_message(char * print_string,char * print_string2,char * print_string3,int ms_to_display){

  prepare_for_timed_screen(ms_to_display);
  print_center_screen(print_string,print_string2,print_string3);

}

//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::print_center_timed_message(char * print_string,char * print_string2,char * print_string3,char * print_string4,int ms_to_display){


  prepare_for_timed_screen(ms_to_display);
  print_center_screen(print_string,print_string2,print_string3,print_string4);

}
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::save_current_screen_to_revert_screen_buffer(){

  for (int x = 0;x < MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS;x++){
    screen_buffer_revert[x] = screen_buffer_live[x];
    screen_buffer_pending[x] = ' ';
    screen_buffer_attributes_revert[x] = screen_buffer_attributes_live[x];
    screen_buffer_attributes_pending[x] = 0;
  }

}

//-----------------------------------------------------------------------------------------------------
/*
void K3NGdisplay::push_revert_screen_buffer_to_live_buffer(){

  for (int x = 0;x < MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS;x++){
    screen_buffer_live[x] = screen_buffer_revert[x];
    screen_buffer_attributes_live[x] = screen_buffer_attributes_revert[x];
  }

}
*/
//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::push_revert_screen_buffer_to_pending_buffer(){

  for (int x = 0;x < MAX_SCREEN_BUFFER_COLUMNS*MAX_SCREEN_BUFFER_ROWS;x++){
    screen_buffer_pending[x] = screen_buffer_revert[x];
    screen_buffer_attributes_pending[x] = screen_buffer_attributes_revert[x];
  }

}


//-----------------------------------------------------------------------------------------------------
void K3NGdisplay::revert_back_screen(){

  //push_revert_screen_buffer_to_live_buffer();
  push_revert_screen_buffer_to_pending_buffer();
  //redraw();
  update();

}

//-----------------------------------------------------------------------------------------------------

#if defined(FEATURE_ADAFRUIT_BUTTONS)
uint8_t K3NGdisplay::readButtons(){

  return lcd.readButtons();

}
#endif //FEATURE_ADAFRUIT_BUTTONS


#endif //K3NG_DISPLAY_H

