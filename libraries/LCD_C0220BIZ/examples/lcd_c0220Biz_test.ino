#include <Arduino.h>
#include <Wire.h>
#include "ST7036.h"
#include "LCD_C0220BiZ.h"

// ???:fmalpartida:20110821 

/*!
    @defined    CHAR_WIDTH
    @abstract   Character witdth of the display, expressed in pixeles per character.
*/
#define CHAR_WIDTH  5

/*!
    @defined    LDR_PIN
    @abstract   Light Detection Resistor Analog pin.
    @discussion Define the Analog channel that will be used to read a LDR.
*/
#define LDR_PIN     7


extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;


//LCD_C0220BIZ lcd = LCD_C0220BIZ ( );
ST7036 lcd = ST7036 ( 2, 20, 0x78 );


/*!
    @const      charBitmap 
    @abstract   Define Character bitmap for the bargraph.
    @discussion Defines a character bitmap to represent a bargraph on a text
    display. The bitmap goes from a blank character to full black.
*/
const uint8_t charBitmap[][8] = {
   { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },
   { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0 },
   { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x0 },
   { 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x0 },
   { 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x0 },
   { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0 }
};

/*!
    @function
    @abstract   Return available RAM memory
    @discussion This routine returns the ammount of RAM memory available after
                initialising the C runtime.
    @param      
    @result     Free RAM available.
*/

static int freeMemory() 
{
  int free_memory;

  if((int)__brkval == 0)
     free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}


/*!
    @function
    @abstract   Braws a bargraph onto the display representing the value passed.
    @discussion Draws a bargraph on the specified row using barLength characters. 
    @param      value[in] Value to represent in the bargraph
    @param      row[in] Row of the LCD where to display the bargraph. Range (0, 1)
                for this display.
    @param      barlength[in] Length of the bar, expressed in display characters.
    @param      start[in]     Start bar character
    @param      end [in]      End bar character

    @result     None
*/
static void drawBars ( int value, uint8_t row, uint8_t barLength, char start, 
                       char end )
{
   int numBars;

   // Set initial titles on the display
   lcd.setCursor (row, 0);
   lcd.print (start);

   // Calculate the size of the bar
   value = map ( value, 0, 1024, 0, ( barLength - 1) * CHAR_WIDTH );
   numBars = value / CHAR_WIDTH;

   lcd.setCursor ( row,2 );
   
   // Draw the bars
   while ( numBars-- )
   {
      lcd.print ( char( (sizeof(charBitmap ) / sizeof(charBitmap[0])) - 1 ) );
   }
   
   // Draw the fractions
   numBars = value % CHAR_WIDTH;
   lcd.print ( char(numBars) );
   lcd.setCursor (row, barLength + 1);
   lcd.print ( " " );
   lcd.print (end);

}

void setup ()
{
   int i; 
   int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));
   
   Serial.begin ( 57600 );
   analogReference ( DEFAULT );
   pinMode ( LDR_PIN, INPUT );
   lcd.init ();
   lcd.setContrast(10);
  
   // Load custom character set into CGRAM
   for ( i = 0; i < charBitmapSize; i++ )
   {
      lcd.load_custom_character ( i, (uint8_t *)charBitmap[i] );
   }
   Serial.println ( freeMemory () );
}


void loop ()
{
  int lightLevel;
  
  lcd.clear ();
  lcd.print ("Light Level:");
  lightLevel = analogRead (LDR_PIN);
  lcd.setCursor ( 1, 16 );
  lcd.print ( lightLevel );  
  lcd.setCursor (1,0);
  drawBars ( lightLevel, 1, 12, '-', '+' );
  delay (100);
}
