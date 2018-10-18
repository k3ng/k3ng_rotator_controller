// ---------------------------------------------------------------------------
// Created by Francisco Malpartida on 20/08/11.
// Copyright 2011 - Under creative commons license 3.0:
//        Attribution-ShareAlike CC BY-SA
//
// This software is furnished "as is", without technical support, and with no 
// warranty, express or implied, as to its usefulness for any purpose.
//
// Thread Safe: No
// Extendable: No
//
// @file lcd.h
// LCD API 1.0 interface declaration class.
// 
// @brief Based on the LCD API 1.0 by dale@wentztech.com
//        This class implements the LCD API abstract library class from
//        which all LCDs inherite.
//
// @author F. Malpartida - fmalpartida@gmail.com
// ---------------------------------------------------------------------------

#ifndef LCD_h
#define LCD_h
#include "Arduino.h"
#include "Print.h"


#define _LCDEXPANDED				// If defined turn on advanced functions

#define _LCD_API_VERSION      "1.0"

class LCD : public Print 
{
	
public: 
	
   /**
    Send a command to the display
    
    @param value[in] Command to be sent to the display
    
    @return None
    
    void command(uint8_t value);
    */
	virtual void command(uint8_t value) = 0;
   
   /**
    Initialise the display. Once created the object, this is the next operation
    that has to be called to initialise the display into a known state. It
    assumes that the I2C bus is not initialised and hence initialise the Wire
    interface.
    
    Clear the display
    Set contrast levels
    Set the cursor at origens (0,0)
    Turn on the entire display
    
    void init();
    */
	virtual void init() = 0;
   
	/**
    Set a different delay to that in the library. It may be needed to delay
    sending commands or characters one after the other.
    
    @param cmdDelay[in] Delay after issuing a command
    @param charDelay[in] Delay after issuing a character to the LCD
    
    @return None
    
    void setDelay(int,int);    
    */
	virtual void setDelay(int,int) = 0;
	
   /**
    This is the write method used by the Print library. It allows printing
    characters to the display and new lines: print, println. It will write the
    value to the display and increase the cursor.
    
    @param value[in] character to write to the current LCD write position
    
    @return None
    
    virtual void write(uint8_t);
    */
	virtual size_t write(uint8_t) = 0;
   
   /**
    This is the write method used by the Print library. It allows printing
    characters to the display and new lines: print, println. It will write the
    value to the display and increase the cursor.
    
    @param buffer[in] buffer to write to the current LCD write position
    @param size[in] size of the buffer
    
    @return None
    
    virtual void write(uint8_t, size_t);
    */
   virtual size_t write(const uint8_t *buffer, size_t size) = 0;
	
   /**
    Clear the display and set the cursor to 0,0
    
    void clear();
    */    
	virtual void clear() = 0;
	
   /**
    Set the cursor to 0,0 
    
    void home();
    */
	virtual void home() = 0;
	
   /**
    Switch the display on. This is the default state when the display is
    initialised. See. init() method
    
    void on();
    */
	virtual void on() = 0;
   
   /**
    Switch the display off. 
    
    void off();
    */   
	virtual void off() = 0;
   
   /**
    Turn on the cursor "_". 
    
    void cursor_on();
    */
	virtual void cursor_on() = 0;
   
   /**
    Turn off the cursor. This is the default state when the display is
    initialised.
    
    void cursor_off();
    */   
	virtual void cursor_off() = 0;
	
   /**
    Activate cursor blink.
    
    void blink_on();
    */
	virtual void blink_on() = 0;
	
   /**
    Deactivate cursor blinking. This is the default state when the display is
    initialised.
    
    void blink_off ();
    */
	virtual void blink_off() = 0;
	
   /**
    Set the cursor at the following coordinates (Line, Col). Initial value after
    initialization is (0,0).
    
    @param Line[in] Line where to put the cursor, range (0, max display lines-1)
    This display only take (0, 1)
    @param Col[in]  Colum where to put the cursor, range (0, max width+1)
    
    @return None
    
    void setCursor(uint8_t Line, uint8_t Col );
    */
	virtual void setCursor(uint8_t Line, uint8_t Col ) = 0;
	
	//
   // EXPANDED FUNCTIONALITY METHODS
   // --------------------------------------------------------------------------
	
#ifdef _LCDEXPANDED		
   
	/**
    Provides the state of the LCD. This value is updated every command is sent
    to the LCD or a character or a buffer is writen to the display.
    
    @return 0 OK, 1 if data was too big to be transmitted, 2 NACK on address
    transmission, 3 NACK on data transmission, 4 other error.
    
    uint8_t status();
    */
	virtual uint8_t status() = 0;
   
   /**
    Load a custom character on the display. After adding a new character to
    the character set, the coordinates are set to (0, 0). This method should
    be called during initialization.
    
    @param char_num[in] Character to load onto the display, this display supports
    upto 16 user defined characters.
    @param rows[in] Bitmap defining the character, the display assumes an array
    of 8 bytes per character.
    
    @return None.
    
    uint8_t load_custom_character(uint8_t char_num, uint8_t *rows);
    */
	virtual void load_custom_character(uint8_t char_num, uint8_t *rows) = 0;
   
	/**
    NOT SUPPORTED
    
    uint8_t keypad();
    */
	virtual uint8_t keypad() = 0;
	
	void printstr(const char[]);
	
   /**
    Sets the backlight level. If the backlight level is connected to a PWM  pin,
    new_val will set a light level range between 0 and 255. If it is connected
    to a normal GPIO, from 0 to 127 it will be off and from 128 to 255 the 
    backlight will be on. Backlight pin allocation on constructor.
    
    @param new_val[in] Backlight level of the display. Full range will only be
    available on pins with PWM support.
    
    @return None.
    
    uint8_t setBacklight();
    */   
	virtual void setBacklight(uint8_t new_val) = 0;
   
   /**
    Sets the LCD contrast level.
    
    @param new_val[in] The contrast range (0 to 255) has been mapped to 16 
    contrast levels on the display.
    
    @return None.
    
    uint8_t setContrast();
    */
	virtual void setContrast(uint8_t new_val) = 0;
   
   
#endif
	
private:

};

#endif

