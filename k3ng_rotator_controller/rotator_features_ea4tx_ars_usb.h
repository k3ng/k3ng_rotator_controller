/* ---------------------- EA4TX ARS USB Features and Options - you must configure this if using HARDWARE_EA4TX_ARS_USB !! ------------------------------------------------*/

/* main features */
//#define FEATURE_ELEVATION_CONTROL       // uncomment this for AZ/EL rotators
#define FEATURE_YAESU_EMULATION           // uncomment this for Yaesu GS-232 emulation on control port
//#define FEATURE_EASYCOM_EMULATION         // Easycom protocol emulation on control port (undefine FEATURE_YAESU_EMULATION above)

#define LANGUAGE_ENGLISH         // all languages customized in rotator_language.h
//#define LANGUAGE_SPANISH
//#define LANGUAGE_CZECH
//#define LANGUAGE_ITALIAN
//#define LANGUAGE_PORTUGUESE_BRASIL
//#define LANGUAGE_GERMAN  
//#define LANGUAGE_FRENCH
//#define LANGUAGE_DUTCH

//#define FEATURE_TEST_DISPLAY_AT_STARTUP

#define FEATURE_AZ_POSITION_POTENTIOMETER   //this is used for both a voltage from a rotator control or a homebrew rotator with a potentiometer

#define FEATURE_EL_POSITION_POTENTIOMETER

#define FEATURE_4_BIT_LCD_DISPLAY //Uncomment for classic 4 bit LCD display (most common)

// #define FEATURE_AUDIBLE_ALERT


/* less often used features and options */
#define OPTION_GS_232B_EMULATION          // comment this out to default to Yaesu GS-232A emulation when using FEATURE_YAESU_EMULATION above
//#define FEATURE_ROTATION_INDICATOR_PIN     // activate rotation_indication_pin to indicate rotation
//#define FEATURE_LIMIT_SENSE
//#define FEATURE_TIMED_BUFFER           // Support for Yaesu timed buffer commands
//#define OPTION_SERIAL_HELP_TEXT        // Yaesu help command prints help
//#define FEATURE_PARK
//#define FEATURE_AUTOPARK               // Requires FEATURE_PARK
//#define OPTION_AZ_MANUAL_ROTATE_LIMITS    // this option will automatically stop the L and R commands when hitting a CCW or CW limit (settings below - AZ_MANUAL_ROTATE_*_LIMIT) 
//#define OPTION_EL_MANUAL_ROTATE_LIMITS
#define OPTION_EASYCOM_AZ_QUERY_COMMAND // Adds non-standard Easycom command: AZ with no parm returns current azimuth
#define OPTION_EASYCOM_EL_QUERY_COMMAND // Adds non-standard Easycom command: EL with no parm returns current elevation
//#define OPTION_C_COMMAND_SENDS_AZ_AND_EL  // uncomment this when using Yaesu emulation with Ham Radio Deluxe
//#define OPTION_DELAY_C_CMD_OUTPUT         // uncomment this when using Yaesu emulation with Ham Radio Deluxe
//#define FEATURE_ONE_DECIMAL_PLACE_HEADINGS
//#define FEATURE_AZIMUTH_CORRECTION        // correct the azimuth using a calibration table below
//#define FEATURE_ELEVATION_CORRECTION      // correct the elevation using a calibration table below
//#define FEATURE_ANCILLARY_PIN_CONTROL     // control I/O pins with serial commands \F, \N, \P
//#define OPTION_EL_SPEED_FOLLOWS_AZ_SPEED    // changing the azimith speed with Yaesu X commands or an azimuth speed pot will also change elevation speed
//#define OPTION_BUTTON_RELEASE_NO_SLOWDOWN  // disables slowdown when CW or CCW button is released, or stop button is depressed
//#define FEATURE_POWER_SWITCH
//#define OPTION_EXTERNAL_ANALOG_REFERENCE  //Activate external analog voltage reference (needed for RemoteQTH.com unit)
#define OPTION_DISPLAY_DIRECTION_STATUS
#define OPTION_SAVE_MEMORY_EXCLUDE_EXTENDED_COMMANDS
//#define OPTION_DONT_READ_GPS_PORT_AS_OFTEN
// #define OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING  // change OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING_STRING in settings file



/* ---------------------- debug stuff - don't touch unless you know what you are doing --------------------------- */



#define DEFAULT_DEBUG_STATE 0// this should be set to zero unless you're debugging something at startup

#define DEBUG_DUMP
// #define DEBUG_BUTTONS
// #define DEBUG_SERIAL
// #define DEBUG_SERVICE_REQUEST_QUEUE
// #define DEBUG_EEPROM
// #define DEBUG_AZ_SPEED_POT
// #define DEBUG_AZ_PRESET_POT
// #define DEBUG_PRESET_ENCODERS
// #define DEBUG_AZ_MANUAL_ROTATE_LIMITS
// #define DEBUG_EL_MANUAL_ROTATE_LIMITS
// #define DEBUG_BRAKE
// #define DEBUG_OVERLAP
// #define DEBUG_DISPLAY
// #define DEBUG_AZ_CHECK_OPERATION_TIMEOUT
// #define DEBUG_TIMED_BUFFER
// #define DEBUG_EL_CHECK_OPERATION_TIMEOUT
// #define DEBUG_VARIABLE_OUTPUTS
// #define DEBUG_ROTATOR
// #define DEBUG_SUBMIT_REQUEST
// #define DEBUG_SERVICE_ROTATION
// #define DEBUG_POSITION_ROTARY_ENCODER
// #define DEBUG_PROFILE_LOOP_TIME
// #define DEBUG_POSITION_PULSE_INPUT
// #define DEBUG_ACCEL
// #define DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
// #define DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER_BAD_DATA
// #define DEBUG_HEADING_READING_TIME
// #define DEBUG_JOYSTICK
// #define DEBUG_ROTATION_INDICATION_PIN
// #define DEBUG_HH12
// #define DEBUG_PARK
// #define DEBUG_LIMIT_SENSE
// #define DEBUG_AZ_POSITION_INCREMENTAL_ENCODER
// #define DEBUG_EL_POSITION_INCREMENTAL_ENCODER
// #define DEBUG_MOON_TRACKING
// #define DEBUG_SUN_TRACKING
// #define DEBUG_GPS
// #define DEBUG_GPS_SERIAL
// #define DEBUG_OFFSET
// #define DEBUG_RTC
// #define DEBUG_PROCESS_YAESU
// #define DEBUG_ETHERNET
// #define DEBUG_PROCESS_SLAVE
// #define DEBUG_MEMSIC_2125
// #define DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE
// #define DEBUG_QMC5883






