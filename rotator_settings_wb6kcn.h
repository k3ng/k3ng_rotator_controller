
/* -------------------------- rotation settings ---------------------------------------*/

#define AZIMUTH_STARTING_POINT_DEFAULT 180      // the starting point in degrees of the azimuthal rotator
                                                
#define AZIMUTH_ROTATION_CAPABILITY_DEFAULT 360 // the default rotation capability of the rotator in degrees
                                                
#define ELEVATION_MAXIMUM_DEGREES 180           // change this to set the maximum elevation in degrees

/* --------------------------- Settings ------------------------------------------------

You can tweak these, but read the online documentation!

*/

// analog voltage calibration - these are default values; you can either tweak these or set via the Yaesu O and F commands (and O2 and F2)....
#define ANALOG_AZ_FULL_CCW 4
#define ANALOG_AZ_FULL_CW 1009
#define ANALOG_EL_0_DEGREES 2
#define ANALOG_EL_MAX_ELEVATION 1018  // maximum elevation is normally 180 degrees unless change below for ELEVATION_MAXIMUM_DEGREES

#define ANALOG_AZ_OVERLAP_DEGREES 540         // if overlap_led above is enabled, turn on overlap led line if azimuth is greater than this setting
                                              // you must use raw azimuth (if the azimuth on the rotator crosses over to 0 degrees, add 360
                                              // for example, on a Yaesu 450 degree rotator with a starting point of 180 degrees, and an overlap LED
                                              // turning on when going CW and crossing 180, ANALOG_AZ_OVERLAP_DEGREES should be set for 540 (180 + 360)
                                              

// PWM speed voltage settings
#define PWM_SPEED_VOLTAGE_X1  74         // 0 to 255
#define PWM_SPEED_VOLTAGE_X2  128        // 0 to 255
#define PWM_SPEED_VOLTAGE_X3  191        // 0 to 255
#define PWM_SPEED_VOLTAGE_X4  253        // 0 to 255

//AZ
#define AZ_SLOWSTART_DEFAULT 1            // 0 = off ; 1 = on
#define AZ_SLOWDOWN_DEFAULT 1             // 0 = off ; 1 = on
#define AZ_SLOW_START_UP_TIME 550        // if slow start is enabled, the unit will ramp up speed for this many milliseconds
#define AZ_SLOW_START_STARTING_PWM 2      // PWM starting value for slow start (must be < 256)
#define AZ_SLOW_START_STEPS 20            // must be < 256


#define SLOW_DOWN_BEFORE_TARGET_AZ 5.0  // if slow down is enabled, slowdown will be activated within this many degrees of target azimuth
#define AZ_SLOW_DOWN_PWM_START 150         // starting PWM value for slow down (must be < 256)
#define	AZ_SLOW_DOWN_PWM_STOP 10          // ending PWM value for slow down (must be < 256)
#define AZ_SLOW_DOWN_STEPS 200 //20       // must be < 256
#define AZ_INITIALLY_IN_SLOW_DOWN_PWM 10  // PWM value to start at if we're starting in the slow down zone (1 - 255)

//EL
#define EL_SLOWSTART_DEFAULT 1            // 0 = off ; 1 = on
#define EL_SLOWDOWN_DEFAULT 1            // 0 = off ; 1 = on
#define EL_SLOW_START_UP_TIME 200        // if slow start is enabled, the unit will ramp up speed for this many milliseconds
#define EL_SLOW_START_STARTING_PWM 3      // PWM starting value for slow start  (must be < 256 and > 0)
#define EL_SLOW_START_STEPS 20            // must be < 256

#define SLOW_DOWN_BEFORE_TARGET_EL 20.0 //5.0  // if slow down is enabled, slowdown will be activated within this many degrees of target elevtion
#define EL_SLOW_DOWN_PWM_START 50 //150         // starting PWM value for slow down (must be < 256 and > 0)
#define	EL_SLOW_DOWN_PWM_STOP 1          // ending PWM value for slow down (must be < 256 and > 0)
#define EL_SLOW_DOWN_STEPS 500 //200
#define EL_INITIALLY_IN_SLOW_DOWN_PWM 10  // PWM value to start at if we're starting in the slow down zone (1 - 255)

#define TIMED_SLOW_DOWN_TIME 500

//Variable frequency output settings - LOWEST FREQUENCY IS 31 HERTZ DUE TO ARDUINO tone() FUNCTION LIMITATIONS!
// (Except when used with FEATURE_STEPPER_MOTOR and FEATURE_STEPPER_MOTOR_EXPERIMENTAL_CODE)
#define AZ_VARIABLE_FREQ_OUTPUT_LOW   5 //32     // Frequency in hertz of minimum speed
#define AZ_VARIABLE_FREQ_OUTPUT_HIGH 1000 //100    // Frequency in hertz of maximum speed
#define EL_VARIABLE_FREQ_OUTPUT_LOW   5 //32     // Frequency in hertz of minimum speed
#define EL_VARIABLE_FREQ_OUTPUT_HIGH 500    // Frequency in hertz of maximum speed

// Settings for OPTION_AZ_MANUAL_ROTATE_LIMITS
#define AZ_MANUAL_ROTATE_CCW_LIMIT 179   // if using a rotator that starts at 180 degrees, set this to something like 185
#define AZ_MANUAL_ROTATE_CW_LIMIT 539  // add 360 to this if you go past 0 degrees (i.e. 180 CW after 0 degrees = 540)

// Settings for OPTION_EL_MANUAL_ROTATE_LIMITS
#define EL_MANUAL_ROTATE_DOWN_LIMIT 10
#define EL_MANUAL_ROTATE_UP_LIMIT 170

// Speed pot settings
#define SPEED_POT_LOW 0
#define SPEED_POT_HIGH 1023
#define SPEED_POT_LOW_MAP 1
#define SPEED_POT_HIGH_MAP 255

// Azimuth preset pot settings
#define AZ_PRESET_POT_FULL_CW 0
#define AZ_PRESET_POT_FULL_CCW 1023
#define AZ_PRESET_POT_FULL_CW_MAP 180         // azimuth pot fully counter-clockwise degrees
#define AZ_PRESET_POT_FULL_CCW_MAP 630        // azimuth pot fully clockwise degrees

#define ENCODER_PRESET_TIMEOUT 5000

// various code settings
#define AZIMUTH_TOLERANCE 0.10            // rotator will stop within X degrees when doing autorotation
#define ELEVATION_TOLERANCE 0.1 //1.0
#define OPERATION_TIMEOUT 120000        // timeout for any rotation operation in mS ; 120 seconds is usually enough unless you have the speed turned down
#define TIMED_INTERVAL_ARRAY_SIZE 20

#define CONTROL_PORT_BAUD_RATE 9600
#define REMOTE_UNIT_PORT_BAUD_RATE 9600
#define GPS_PORT_BAUD_RATE 9600
#define GPS_MIRROR_PORT_BAUD_RATE 9600
#define CONTROL_PORT_MAPPED_TO &Serial  // change this line to map the control port to a different serial port (Serial1, Serial2, etc.)
#define REMOTE_PORT_MAPPED_TO &Serial1  // change this line to map the remote_unit port to a different serial port
#define GPS_PORT_MAPPED_TO &Serial2  // change this line to map the GPS port to a different serial port
//#define GPS_MIRROR_PORT &Serial3 // use this to mirror output from a GPS unit into the Arduino out another port (uncomment to enable)

#define LCD_COLUMNS 20 //16
#define LCD_ROWS 4 //2
#define LCD_UPDATE_TIME 1000           // LCD update time in milliseconds
#define I2C_LCD_COLOR GREEN            // default color of I2C LCD display, including Adafruit and Yourduino; some Yourduino may want this as LED_ON
#define LCD_HHMM_CLOCK_POSITION LEFT          //LEFT or RIGHT
#define LCD_HHMMSS_CLOCK_POSITION LEFT          //LEFT or RIGHT
#define LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_POSITION LEFT
#define LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_ROW 1
#define LCD_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD_POSITION LEFT
#define LCD_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD_ROW 1
#define LCD_BIG_CLOCK_ROW 4
#define LCD_GPS_INDICATOR_POSITION RIGHT //LEFT or RIGHT
#define LCD_GPS_INDICATOR_ROW 1
#define LCD_MOON_TRACKING_ROW 3                                   // LCD display row for OPTION_DISPLAY_MOON_TRACKING_CONTINUOUSLY
#define LCD_MOON_TRACKING_UPDATE_INTERVAL 5000
#define LCD_SUN_TRACKING_ROW 4                                    // LCD display row for OPTION_DISPLAY_SUN_TRACKING_CONTINUOUSLY
#define LCD_SUN_TRACKING_UPDATE_INTERVAL 5000
#define LCD_MOON_OR_SUN_TRACKING_CONDITIONAL_ROW 3                // LCD display row for OPTION_DISPLAY_MOON_OR_SUN_TRACKING_CONDITIONAL
#define SPLASH_SCREEN_TIME 3000

#define AZ_BRAKE_DELAY 3000            // in milliseconds
#define EL_BRAKE_DELAY 3000            // in milliseconds

#define BRAKE_ACTIVE_STATE HIGH
#define BRAKE_INACTIVE_STATE LOW

#define EEPROM_MAGIC_NUMBER 103
#define EEPROM_WRITE_DIRTY_CONFIG_TIME  30  //time in seconds


#if defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
#define HEADING_MULTIPLIER 100L
#define LCD_HEADING_MULTIPLIER 100.0
#define LCD_DECIMAL_PLACES 2
#endif //FEATURE_TWO_DECIMAL_PLACE_HEADINGS

#if defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS)
#define HEADING_MULTIPLIER 10
#define LCD_HEADING_MULTIPLIER 10.0
#define LCD_DECIMAL_PLACES 1
#endif //FEATURE_ONE_DECIMAL_PLACE_HEADINGS

#if !defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) && !defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
#define HEADING_MULTIPLIER 1
#define LCD_HEADING_MULTIPLIER 1.0
#define LCD_DECIMAL_PLACES 0
#endif //!defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) && !defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)

#define AZ_POSITION_ROTARY_ENCODER_DEG_PER_PULSE 0.5
#define EL_POSITION_ROTARY_ENCODER_DEG_PER_PULSE 0.5

#define AZ_POSITION_PULSE_DEG_PER_PULSE 0.5
#define EL_POSITION_PULSE_DEG_PER_PULSE 0.5

#define PARK_AZIMUTH 360.0 * HEADING_MULTIPLIER      // replace the 0.0 with your park azimuth; azimuth is in raw degrees (i.e. on a 180 degree starting point rotator, 0 degrees = 360)
#define PARK_ELEVATION 0.0 * HEADING_MULTIPLIER    // replace the 0.0 with your park elevation

#define COMMAND_BUFFER_SIZE 50

#define REMOTE_BUFFER_TIMEOUT_MS 250
#define REMOTE_UNIT_COMMAND_TIMEOUT_MS 2000
#define AZ_REMOTE_UNIT_QUERY_TIME_MS 150         // how often we query the remote remote for azimuth
#define EL_REMOTE_UNIT_QUERY_TIME_MS 150         // how often we query the remote remote for elevation

#define ROTATE_PIN_INACTIVE_VALUE LOW
#define ROTATE_PIN_ACTIVE_VALUE HIGH

#define AZIMUTH_SMOOTHING_FACTOR 0      // value = 0 to 99.9
#define ELEVATION_SMOOTHING_FACTOR 0    // value = 0 to 99.9

#define AZIMUTH_MEASUREMENT_FREQUENCY_MS 100        // this does not apply if using FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT
#define ELEVATION_MEASUREMENT_FREQUENCY_MS 100      // this does not apply if using FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT

#define JOYSTICK_WAIT_TIME_MS 100

#define ROTATION_INDICATOR_PIN_ACTIVE_STATE HIGH
#define ROTATION_INDICATOR_PIN_INACTIVE_STATE LOW
#define ROTATION_INDICATOR_PIN_TIME_DELAY_SECONDS 0
#define ROTATION_INDICATOR_PIN_TIME_DELAY_MINUTES 0

#define AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV 2000.0
#define EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV 2000.0
#define AZ_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION 0  // can be 0 to 4 x AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV
#define EL_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION 0  // can be 0 to 4 x EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV

#define SERIAL_LED_TIME_MS 250

#define DEFAULT_LATITUDE 40.889958
#define DEFAULT_LONGITUDE -75.585972

#define MOON_TRACKING_CHECK_INTERVAL 5000
#define MOON_AOS_AZIMUTH_MIN 0
#define MOON_AOS_AZIMUTH_MAX 360
#define MOON_AOS_ELEVATION_MIN 0
#define MOON_AOS_ELEVATION_MAX 180

 
#define SUN_TRACKING_CHECK_INTERVAL 5000
#define SUN_AOS_AZIMUTH_MIN 0
#define SUN_AOS_AZIMUTH_MAX 360
#define SUN_AOS_ELEVATION_MIN 0
#define SUN_AOS_ELEVATION_MAX 180

#ifdef LANGUAGE_ENGLISH               // English language support (copy leading and trailing spaces when making your own language section)
#define MOON_STRING "moon "
#define SUN_STRING "sun "
#define AZ_TARGET_STRING "Az Target "
#define EL_TARGET_STRING "El Target "
#define TARGET_STRING "Target "
#define PARKED_STRING "Parked"
#define ROTATING_CW_STRING "Rotating CW"
#define ROTATING_CCW_STRING "Rotating CCW"
#define ROTATING_TO_STRING "Rotating to "
#define ELEVATING_TO_STRING "Elevating to "
#define ELEVATING_UP_STRING "Elevating Up"
#define ELEVATING_DOWN_STRING "Elevating Down"
#define ROTATING_STRING "Rotating "
#define CW_STRING "CW"
#define CCW_STRING "CCW"
#define UP_STRING "UP"
#define DOWN_STRING "DOWN"
#define AZIMUTH_STRING "Azimuth "
#define AZ_STRING "Az"
#define AZ_SPACE_STRING "Az "
#define SPACE_EL_STRING " El"
#define SPACE_EL_SPACE_STRING " El "
#define GPS_STRING "GPS"
#define N_STRING "N"
#define W_STRING "W"
#define S_STRING "S"
#define E_STRING "E"
#define NW_STRING "NW"
#define SW_STRING "SW"
#define SE_STRING "SE"
#define NE_STRING "NE"
#define NNW_STRING "NNW"
#define WNW_STRING "WNW"
#define WSW_STRING "WSW"
#define SSW_STRING "SSW"
#define SSE_STRING "SSE"
#define ESE_STRING "ESE"
#define ENE_STRING "ENE"
#define NNE_STRING "NNE"
#endif //LANGUAGE_ENGLISH

#ifdef LANGUAGE_SPANISH               // Courtesy of Maximo, EA1DDO
#define MOON_STRING "Luna "
#define SUN_STRING "Sol "
#define AZ_TARGET_STRING "Az Objetivo "
#define EL_TARGET_STRING "El Objetivo "
#define TARGET_STRING "Objetivo "
#define PARKED_STRING "Aparcado"
#define ROTATING_CW_STRING "Girando Dcha"
#define ROTATING_CCW_STRING "Girando Izq"
#define ROTATING_TO_STRING "Girando a "
#define ELEVATING_TO_STRING "Elevando a "
#define ELEVATING_UP_STRING "Subiendo"
#define ELEVATING_DOWN_STRING "Bajando"
#define ROTATING_STRING "Girando "
#define CW_STRING "Dcha"
#define CCW_STRING "Izq"
#define UP_STRING "Arriba"
#define DOWN_STRING "Abajo"
#define AZIMUTH_STRING "Azimuth "
#define AZ_STRING "Az"
#define AZ_SPACE_STRING "Az "
#define SPACE_EL_STRING " El"
#define SPACE_EL_SPACE_STRING " El "
#define GPS_STRING "GPS"
#define N_STRING "N"
#define W_STRING "O"
#define S_STRING "S"
#define E_STRING "E"
#define NW_STRING "NO"
#define SW_STRING "SO"
#define SE_STRING "SE"
#define NE_STRING "NE"
#define NNW_STRING "NNO"
#define WNW_STRING "ONO"
#define WSW_STRING "OSO"
#define SSW_STRING "SSO"
#define SSE_STRING "SSE"
#define ESE_STRING "ESE"
#define ENE_STRING "ENE"
#define NNE_STRING "NNE"
#endif //LANGUAGE_SPANISH

#ifdef LANGUAGE_CHECK            // courtesy of Jan, OK2ZAW
#define MOON_STRING "mesic "
#define SUN_STRING "slunce "
#define AZ_TARGET_STRING "Az cíl "
#define EL_TARGET_STRING "El cíl "
#define TARGET_STRING "Cil "
#define PARKED_STRING "Parkovat"
#define ROTATING_CW_STRING "Otacim CW"
#define ROTATING_CCW_STRING "Otacim CCW"
#define ROTATING_TO_STRING "Otacim na "
#define ELEVATING_TO_STRING "Elevovat na "
#define ELEVATING_UP_STRING "Elevovat nahoru"
#define ELEVATING_DOWN_STRING "Elevovat dolu"
#define ROTATING_STRING "Otacet "
#define CW_STRING "CW"
#define CCW_STRING "CCW"
#define UP_STRING "Nahoru"
#define DOWN_STRING "Dolu"
#define AZIMUTH_STRING "Azimut "
#define AZ_STRING "Az"
#define AZ_SPACE_STRING "Az "
#define SPACE_EL_STRING " El"
#define SPACE_EL_SPACE_STRING " El "
#define GPS_STRING "GPS"
#define N_STRING "smer   ^   KL"
#define W_STRING "smer   <   HK"
#define S_STRING "smer   v   ZS"
#define E_STRING "smer   >   VK"
#define NW_STRING "smer   <    W"
#define SW_STRING "smer   v  VP8"
#define SE_STRING "smer   >   HZ"
#define NE_STRING "smer   ^   JA"
#define NNW_STRING "smer   ^   VE"
#define WNW_STRING "smer   <   CO"
#define WSW_STRING "smer   <   PY"
#define SSW_STRING "smer   v  ZD9"
#define SSE_STRING "smer   v   5R"
#define ESE_STRING "smer   >   8Q"
#define ENE_STRING "smer   >   ZL"
#define NNE_STRING "smer   ^  UA0"
#endif //LANGUAGE_CZECH

#ifdef LANGUAGE_ITALIAN    // courtesy of Paolo, IT9IPQ           
#define MOON_STRING "luna"
#define SUN_STRING "sole "
#define AZ_TARGET_STRING "Punta Az  "
#define EL_TARGET_STRING "Punta El  "
#define TARGET_STRING "Punta  "
#define PARKED_STRING "Posa  "
#define ROTATING_CW_STRING "Ruota DX > "
#define ROTATING_CCW_STRING "Ruota SX <  "
#define ROTATING_TO_STRING "Ruota verso "
#define ELEVATING_TO_STRING "Alza verso   "
#define ELEVATING_UP_STRING "Alzo  Su    "
#define ELEVATING_DOWN_STRING "Alzo  Giu'    "
#define ROTATING_STRING "Ruota    "
#define CW_STRING "DX"
#define CCW_STRING "SX "
#define UP_STRING "SU"
#define DOWN_STRING "GIU'"
#define AZIMUTH_STRING "Azimuth "
#define AZ_STRING "Az"
#define AZ_SPACE_STRING "Az "
#define SPACE_EL_STRING " El"
#define SPACE_EL_SPACE_STRING " El "
#define GPS_STRING "GPS"
#define N_STRING "N"
#define W_STRING "W"
#define S_STRING "S"
#define E_STRING "E"
#define NW_STRING "NW"
#define SW_STRING "SW"
#define SE_STRING "SE"
#define NE_STRING "NE"
#define NNW_STRING "NNW"
#define WNW_STRING "WNW"
#define WSW_STRING "WSW"
#define SSW_STRING "SSW"
#define SSE_STRING "SSE"
#define ESE_STRING "ESE"
#define ENE_STRING "ENE"
#define NNE_STRING "NNE"
#endif //LANGUAGE_ITALIAN

#define TRACKING_ACTIVE_CHAR "*"
#define TRACKING_INACTIVE_CHAR "-"

#define INTERNAL_CLOCK_CORRECTION 0.00145

#define SYNC_TIME_WITH_GPS 1
#define SYNC_COORDINATES_WITH_GPS 1
#define GPS_SYNC_PERIOD_SECONDS 10  // how long to consider internal clock syncronized after a GPS reading
#define GPS_VALID_FIX_AGE_MS 10000  // consider a GPS reading valid if the fix age is less than this
#define GPS_UPDATE_LATENCY_COMPENSATION_MS 200

#define SYNC_WITH_RTC_SECONDS 59    // syncronize internal clock with realtime clock every x seconds
#define SYNC_RTC_TO_GPS_SECONDS 12  // synchronize realtime clock to GPS every x seconds

#define SYNC_MASTER_CLOCK_TO_SLAVE_CLOCK_SECS 10 // for OPTION_SYNC_MASTER_CLOCK_TO_SLAVE - use when GPS unit is connected to slave unit and you want to synchronize the master unit clock to the slave unit clock

#define ETHERNET_MAC_ADDRESS 0xDE,0xAD,0xBE,0xEF,0xFE,0xEE  //<-DON'T FORGET TO USE DIFFERENT MAC ADDRESSES FOR MASTER AND SLAVE!!!
#define ETHERNET_IP_ADDRESS 192,168,1,172  //<-DON'T FORGET TO USE DIFFERENT IP ADDRESSES FOR MASTER AND SLAVE!!!
#define ETHERNET_IP_GATEWAY 192,168,1,254
#define ETHERNET_IP_SUBNET_MASK 255,255,255,0
#define ETHERNET_TCP_PORT_0 23
#define ETHERNET_TCP_PORT_1 24
#define ETHERNET_MESSAGE_TIMEOUT_MS 5000
#define ETHERNET_PREAMBLE "K3NG"    // used only with Ethernet master/slave link

#define ETHERNET_SLAVE_IP_ADDRESS 192,168,1,173
#define ETHERNET_SLAVE_TCP_PORT 23
#define ETHERNET_SLAVE_RECONNECT_TIME_MS 250

#define POWER_SWITCH_IDLE_TIMEOUT 15  // use with FEATURE_POWER_SWITCH; units are minutes

#ifdef HARDWARE_EA4TX_ARS_USB
#define BUTTON_ACTIVE_STATE HIGH
#define BUTTON_INACTIVE_STATE LOW
#else
#define BUTTON_ACTIVE_STATE LOW
#define BUTTON_INACTIVE_STATE HIGH
#endif

/*
 *
 * Azimuth and Elevation calibraton tables - use with FEATURE_AZIMUTH_CORRECTION and/or FEATURE_ELEVATION_CORRECTION
 *
 * You must have the same number of entries in the _FROM_ and _TO_ arrays!
 *
 */

#define AZIMUTH_CALIBRATION_FROM_ARRAY {180,630}            /* these are in "raw" degrees, i.e. when going east past 360 degrees, add 360 degrees*/
#define AZIMUTH_CALIBRATION_TO_ARRAY {180,630}

// example: reverse rotation sensing
//   #define AZIMUTH_CALIBRATION_FROM_ARRAY {0,359}
//   #define AZIMUTH_CALIBRATION_TO_ARRAY {359,0}


//#define ELEVATION_CALIBRATION_FROM_ARRAY {-180,0,180}
//#define ELEVATION_CALIBRATION_TO_ARRAY {-180,0,180}

#define ELEVATION_CALIBRATION_FROM_ARRAY {0,269.9,270.0,359.9}
#define ELEVATION_CALIBRATION_TO_ARRAY {0,269.9,-90.0,-0.1}

#define ANALOG_OUTPUT_MAX_EL_DEGREES 180

#define EL_POSITION_PULSE_DEBOUNCE 500  // in ms 


/* Pololu LSM303 Calibration tables
 *
 *
 * For use with FEATURE_AZ_POSITION_POLOLU_LSM303 and/or FEATURE_EL_POSITION_POLOLU_LSM303
 *


  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.

  min: {   +59,    +19,   -731}    max: {  +909,   +491,    +14}
  min: {32767, 32767, 32767}       max: {-32768, -32768, -32768}

 */

#define POLOLU_LSM_303_MIN_ARRAY {+59, +19, -731}
#define POLOLU_LSM_303_MAX_ARRAY {+909, +491, +14}

#define AUTOCORRECT_TIME_MS_AZ 1000
#define AUTOCORRECT_TIME_MS_EL 1000   



/* ---------------------------- object declarations ----------------------------------------------


 Object declarations are required for several devices, including LCD displays, compass devices, and accelerometers
   

*/


#ifdef FEATURE_4_BIT_LCD_DISPLAY 
LiquidCrystal lcd(lcd_4_bit_rs_pin, lcd_4_bit_enable_pin, lcd_4_bit_d4_pin, lcd_4_bit_d5_pin, lcd_4_bit_d6_pin, lcd_4_bit_d7_pin); 
#endif //FEATURE_4_BIT_LCD_DISPLAY


#ifdef FEATURE_ADAFRUIT_I2C_LCD
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#endif //FEATURE_ADAFRUIT_I2C_LCD

#ifdef FEATURE_YOURDUINO_I2C_LCD
#define I2C_ADDR 0x20
#define BACKLIGHT_PIN 3
#define LED_OFF 1
#define LED_ON 0            
LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
#endif //FEATURE_YOURDUINO_I2C_LCD

#ifdef FEATURE_RFROBOT_I2C_DISPLAY
LiquidCrystal_I2C lcd(0x27,16,2); 
#endif //FEATURE_RFROBOT_I2C_DISPLAY

#ifdef FEATURE_AZ_POSITION_HMC5883L
HMC5883L compass;
#endif //FEATURE_AZ_POSITION_HMC5883L

#ifdef FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB
ADXL345 accel;
#endif //FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB

#ifdef FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
#endif //FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB

#if defined(FEATURE_EL_POSITION_ADAFRUIT_LSM303) || defined(FEATURE_AZ_POSITION_ADAFRUIT_LSM303)
Adafruit_LSM303 lsm;
#endif

#if defined(FEATURE_AZ_POSITION_POLOLU_LSM303) || defined(FEATURE_EL_POSITION_POLOLU_LSM303)
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
char report[80];
#endif //FEATURE_AZ_POSITION_POLOLU_LSM303

#ifdef FEATURE_AZ_POSITION_HH12_AS5045_SSI
#include "hh12.h"
hh12 azimuth_hh12;
#endif //FEATURE_AZ_POSITION_HH12_AS5045_SSI

#ifdef FEATURE_EL_POSITION_HH12_AS5045_SSI
#include "hh12.h"
hh12 elevation_hh12;
#endif //FEATURE_EL_POSITION_HH12_AS5045_SSI

#ifdef FEATURE_GPS
TinyGPS gps;
#endif //FEATURE_GPS

#ifdef FEATURE_RTC_DS1307
RTC_DS1307 rtc;
#endif //FEATURE_RTC_DS1307

#ifdef FEATURE_RTC_PCF8583
PCF8583 rtc(0xA0);
#endif //FEATURE_RTC_PCF8583

#ifdef HARDWARE_EA4TX_ARS_USB
#undef LCD_COLUMNS
#undef LCD_ROWS
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#endif //HARDWARE_EA4TX_ARS_USB



