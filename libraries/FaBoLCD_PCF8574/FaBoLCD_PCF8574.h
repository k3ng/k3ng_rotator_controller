/**
 @file FaBoLCD_PCF8574.h
 @brief This is a library for the FaBo LCD I2C Brick.

   http://fabo.io/212.html

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

#ifndef FABOLCD_PCF8574_H
#define FABOLCD_PCF8574_H

#include <Arduino.h>
#include <Wire.h>
#include "Print.h"

#define PCF8574_SLAVE_ADDRESS 0x20 ///< PCF8574 Default I2C Slave Address

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// pin/port bit
#define RS  B00000001 // P0 : RS bit
#define RW  B00000010 // P1 : R/W bit
#define EN  B00000100 // P2 : Enable bit
#define BL  B00001000 // P3 : BACKLIGHT bit
#define DB4 B00010000 // P4 : DB4 bit
#define DB5 B00100000 // P5 : DB5 bit
#define DB6 B01000000 // P6 : DB6 bit
#define DB7 B10000000 // P7 : DB7 bit

/**
 @class FaBoLCDmini_AQM0802A
 @brief FaBo LCD I2C Controll class
*/
class FaBoLCD_PCF8574 : public Print {
  public:
    FaBoLCD_PCF8574(uint8_t addr = PCF8574_SLAVE_ADDRESS);
    void init(void);
    void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);

    void clear();
    void home();

    void noDisplay();
    void display();
    void noBlink();
    void blink();
    void noCursor();
    void cursor();
    void scrollDisplayLeft();
    void scrollDisplayRight();
    void leftToRight();
    void rightToLeft();
    void autoscroll();
    void noAutoscroll();

    void setRowOffsets(int row1, int row2, int row3, int row4);
    void createChar(uint8_t, uint8_t[]);
    void setCursor(uint8_t, uint8_t);
    virtual size_t write(uint8_t);
    void command(uint8_t);

private:
    void send(uint8_t, uint8_t);
    void write4bits(uint8_t);
    void write8bits(uint8_t);
    void pulseEnable(uint8_t);
    void writeI2c(uint8_t);

    uint8_t _displayfunction;
    uint8_t _displaycontrol;
    uint8_t _displaymode;

    uint8_t _numlines;
    uint8_t _row_offsets[4];

    uint8_t _i2caddr;
    uint8_t _backlight;
};

#endif // FABOLCD_PCF8574_H
