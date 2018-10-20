/**
 @file FaBoLCD_PCF8574.cpp
 @brief This is a library for the FaBo LCD I2C Brick.

   http://fabo.io/212.html

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

#include "FaBoLCD_PCF8574.h"

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

/**
 @brief Constructor
*/
FaBoLCD_PCF8574::FaBoLCD_PCF8574(uint8_t addr)
{
  _i2caddr = addr;
  _backlight = BL;
  Wire.begin();
  init();
}

/**
 @brief init
*/
void FaBoLCD_PCF8574::init()
{
  _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
}

/**
 @brief brgin
*/
void FaBoLCD_PCF8574::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;

  setRowOffsets(0x00, 0x40, 0x00 + cols, 0x40 + cols);

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != LCD_5x8DOTS) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  delayMicroseconds(50000);
  // Now we pull both RS and R/W low to begin commands
  writeI2c(0x00);

  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46

  // we start in 8bit mode, try to set 4 bit mode
  write4bits(DB4|DB5);
  delayMicroseconds(4500); // wait min 4.1ms

  // second try
  write4bits(DB4|DB5);
  delayMicroseconds(4500); // wait min 4.1ms

  // third go!
  write4bits(DB4|DB5);
  delayMicroseconds(150);

  // finally, set to 4-bit interface
  write4bits(DB5);

  // finally, set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);

}

/**
 @brief setRowOffsets
*/
void FaBoLCD_PCF8574::setRowOffsets(int row0, int row1, int row2, int row3)
{
  _row_offsets[0] = row0;
  _row_offsets[1] = row1;
  _row_offsets[2] = row2;
  _row_offsets[3] = row3;
}

/********** high level commands, for the user! */

/**
 @brief clear
*/
void FaBoLCD_PCF8574::clear()
{
  command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

/**
 @brief home
*/
void FaBoLCD_PCF8574::home()
{
  command(LCD_RETURNHOME);  // set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

/**
 @brief setCursor
*/
void FaBoLCD_PCF8574::setCursor(uint8_t col, uint8_t row)
{
  const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
  if ( row >= max_lines ) {
    row = max_lines - 1;    // we count rows starting w/0
  }
  if ( row >= _numlines ) {
    row = _numlines - 1;    // we count rows starting w/0
  }

  command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}

/**
 @brief Turn the display off (quickly)
*/
void FaBoLCD_PCF8574::noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

/**
 @brief Turn the display on (quickly)
*/
void FaBoLCD_PCF8574::display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

/**
 @brief Turns the underline cursor off
*/
void FaBoLCD_PCF8574::noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

/**
 @brief Turns the underline cursor on
*/
void FaBoLCD_PCF8574::cursor() {
  _displaycontrol |= LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

/**
 @brief Turn off the blinking cursor
*/
void FaBoLCD_PCF8574::noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

/**
 @brief Turn on the blinking cursor
*/
void FaBoLCD_PCF8574::blink() {
  _displaycontrol |= LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

/**
 @brief These commands scroll the display without changing the RAM
*/
void FaBoLCD_PCF8574::scrollDisplayLeft(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

/**
 @brief These commands scroll the display without changing the RAM
*/
void FaBoLCD_PCF8574::scrollDisplayRight(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

/**
 @brief This is for text that flows Left to Right
*/
void FaBoLCD_PCF8574::leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

/**
 @brief This is for text that flows Right to Left
*/
void FaBoLCD_PCF8574::rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

/**
 @brief This will 'right justify' text from the cursor
*/
void FaBoLCD_PCF8574::autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

/**
 @brief This will 'left justify' text from the cursor
*/
void FaBoLCD_PCF8574::noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

/**
 @brief Allows us to fill the first 8 CGRAM locations with custom characters
*/
void FaBoLCD_PCF8574::createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    write(charmap[i]);
  }
}

/*********** mid level commands, for sending data/cmds */

/**
 @brief command
*/
inline void FaBoLCD_PCF8574::command(uint8_t value) {
  send(value, 0);
}

/**
 @brief write
*/
inline size_t FaBoLCD_PCF8574::write(uint8_t value) {
  send(value, RS);
  return 1; // assume sucess
}

/************ low level data pushing commands **********/

/**
 @brief write either command or data, 4-bit
*/
void FaBoLCD_PCF8574::send(uint8_t value, uint8_t mode) {
  uint8_t Hbit = value & 0xF0;
  uint8_t Lbit = (value << 4) & 0xF0;
  write4bits(Hbit|mode);
  write4bits(Lbit|mode);
}

/**
 @brief pulseEnable
*/
void FaBoLCD_PCF8574::pulseEnable(uint8_t value) {
  writeI2c(value & ~EN); // EN LOW
  delayMicroseconds(1);
  writeI2c(value|EN);    // EN HIGH
  delayMicroseconds(1);  // enable pulse must be >450ns
  writeI2c(value & ~EN); // EN LOW
  delayMicroseconds(100); // commands need > 37us to settle
}

/**
 @brief write4bits
*/
void FaBoLCD_PCF8574::write4bits(uint8_t value) {
  writeI2c(value);
  pulseEnable(value);
}

/**
 @brief writeI2c
*/
void FaBoLCD_PCF8574::writeI2c(uint8_t data) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(data|_backlight);
  Wire.endTransmission();
}
