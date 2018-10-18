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
// @file LCD_C0220BiZ.h
// NHD C0220BiZ display class definition.
// 
// @brief Based on the LCD API 1.0 by dale@wentztech.com
//        This library implements the driver to drive the Newhaven Display
//        NHD-C0220BiZ-FSW-FBW-3V3M. The display is build around the ST7036
//        i2c LCD controller. This is a 3.3V display.
//        I2C displays based on the ST7632 should also be compatible.
//
//        Other compatible displays:
//           - NHD-C0220BiZ-FSW-FBW-3V3M
//           - NHD-C0220BiZ-FS(RGB)-FBW-3VM
//        Non tested but should be compatible with no or little changes
//           - NHD-C0216CiZ-FSW-FBW-3V3
//           - NHD-C0216CiZ-FN-FBW-3V
//
// @author F. Malpartida - fmalpartida@gmail.com
// ---------------------------------------------------------------------------

#ifndef LCD_C0220BIZ_h
#define LCD_C0220BIZ_h

#define _LCDEXPANDED				// If defined turn on advanced functions

#include <Arduino.h>
#include "ST7036.h"

#define _LCD_C0220BIZ_VERSION "1.2.0"
#define _LCD_API_VERSION      "1.0"

class LCD_C0220BIZ : public ST7036
{
	
public: 
	
   /**
    Constructor for the display class
        
    @return None
    
    LCD_C0220BIZ( );
    */
   LCD_C0220BIZ( );
   
   /**
    Constructor for the display class with backlight allowcation pin.

    @param backlightPin  initiales the backlight pin.
    
    @return None
    
    LCD_C0220BIZ(int8_t backlightPin );
    */
   LCD_C0220BIZ( int8_t backlightPin );
		
};

#endif

