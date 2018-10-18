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
// @file LCD_C0220BiZ.cpp
// Display class implementation of the LCD API 1.0
// 
// @brief Based on the LCD API 1.0 by dale@wentztech.com
//        This library implements the driver to drive the Newhaven Display
//        NHD‐C0220BiZ‐FSW‐FBW‐3V3M. The display is build around the ST7036
//        i2c LCD controller. This is a 3.3V display.
//        I2C displays based on the ST7032i should also be compatible.
//
//        Other compatible displays:
//           - NHD‐C0220BiZ‐FSW‐FBW‐3V3M
//           - NHD-C0220BiZ-FS(RGB)-FBW-3VM
//        Non tested but should be compatible with no or little changes
//           - NHD-C0216CiZ-FSW-FBW-3V3
//           - NHD‐C0216CiZ‐FN‐FBW‐3V
//
//
// @author F. Malpartida - fmalpartida@gmail.com
// ---------------------------------------------------------------------------
#include "LCD_C0220BiZ.h"

// Class private constants and definition 
// ----------------------------------------------------------------------------
const int     NUM_LINES           = 2;     // Number of lines in the display
const int     NUM_COLUMNS         = 20;    // Number of columns in the display
const int     I2C_ADDRS           = 0x78;  // I2C address of the display

// Static  member variable definitions
// ----------------------------------------------------------------------------

// Static file scope variable definitions
// ----------------------------------------------------------------------------

// Private support functions
// ----------------------------------------------------------------------------

// CLASS METHODS
// ----------------------------------------------------------------------------

// Constructors:
// ---------------------------------------------------------------------------
LCD_C0220BIZ::LCD_C0220BIZ( ):ST7036 ( NUM_LINES, NUM_COLUMNS, I2C_ADDRS )
{

}

LCD_C0220BIZ::LCD_C0220BIZ(int8_t backlightPin ) : 
   ST7036 ( NUM_LINES, NUM_COLUMNS, I2C_ADDRS, backlightPin )
{

}
