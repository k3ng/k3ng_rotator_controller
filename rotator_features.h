/* ---------------------- Features and Options - you must configure this !! ------------------------------------------------*/

/* main features */
//#define FEATURE_ELEVATION_CONTROL       // uncomment this for AZ/EL rotators
#define FEATURE_YAESU_EMULATION           // uncomment this for Yaesu GS-232A emulation
#define OPTION_GS_232B_EMULATION          // uncomment this for GS-232B emulation (also uncomment FEATURE_YAESU_EMULATION above)
//#define FEATURE_EASYCOM_EMULATION         // Easycom protocol emulation (undefine FEATURE_YAESU_EMULATION above)
//#define FEATURE_ROTATION_INDICATOR_PIN     // activate pin (defined below) to indicate rotation

/* host and remote unit functionality */
//#define FEATURE_REMOTE_UNIT_SLAVE //uncomment this to make this unit a remote unit controlled by a host unit


/* position sensors - pick one for azimuth and one for elevation if using an az/el rotator */
#define FEATURE_AZ_POSITION_POTENTIOMETER   //this is used for both a voltage from a rotator control or a homebrew rotator with a potentiometer
//#define FEATURE_AZ_POSITION_ROTARY_ENCODER
//#define FEATURE_AZ_POSITION_PULSE_INPUT
//#define FEATURE_AZ_POSITION_HMC5883L            // HMC5883L digital compass support (also uncomment object declaration below)
//#define FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT  // requires an Arduino with Serial1 suppport (i.e. Arduino Mega); makes this unit a master
//#define FEATURE_AZ_POSITION_LSM303                            // Uncomment for elevation using LSM303 magnetometer and Adafruit library (https://github.com/adafruit/Adafruit_LSM303) (also uncomment object declaration below)

//#define FEATURE_EL_POSITION_POTENTIOMETER
//#define FEATURE_EL_POSITION_ROTARY_ENCODER
//#define FEATURE_EL_POSITION_PULSE_INPUT
//#define FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB // Uncomment for elevation ADXL345 accelerometer support using ADXL345 library (also uncomment object declaration below)
//#define FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB      // Uncomment for elevation ADXL345 accelerometer support using Adafruit library (also uncomment object declaration below)
//#define FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT            // requires an Arduino with Serial1 suppport (i.e. Arduino Mega); makes this unit a master
//#define FEATURE_EL_POSITION_LSM303                            // Uncomment for elevation using LSM303 accelerometer and Adafruit library (https://github.com/adafruit/Adafruit_LSM303) (also uncomment object declaration below)

/* preset rotary encoder features and options */
//#define FEATURE_AZ_PRESET_ENCODER            // Uncomment for Rotary Encoder Azimuth Preset support
//#define FEATURE_EL_PRESET_ENCODER            // Uncomment for Rotary Encoder Elevation Preset support (requires FEATURE_AZ_PRESET_ENCODER above)
#define OPTION_ENCODER_HALF_STEP_MODE
#define OPTION_ENCODER_ENABLE_PULLUPS          // define to enable weak pullups on rotary encoder pins
//#define OPTION_PRESET_ENCODER_RELATIVE_CHANGE   // this makes the encoder(s) change the az or el in a relative fashion rather then store an absolute setting

/* position sensor options */
#define OPTION_AZ_POSITION_ROTARY_ENCODER_HARD_LIMIT // stop azimuth at lower and upper limit rather than rolling over
#define OPTION_EL_POSITION_ROTARY_ENCODER_HARD_LIMIT // stop elevation at lower and upper limits rather than rolling over
#define OPTION_AZ_POSITION_PULSE_HARD_LIMIT  // stop azimuth at lower and upper limit rather than rolling over
#define OPTION_EL_POSITION_PULSE_HARD_LIMIT  // stop elevation at lower and upper limits rather than rolling over
#define OPTION_POSITION_PULSE_INPUT_PULLUPS  // define to enable weak pullups on position pulse inputs

/* less often used features and options */
//#define FEATURE_TIMED_BUFFER           // Support for Yaesu timed buffer commands
//#define OPTION_SERIAL_HELP_TEXT        // Yaesu help command prints help
//#define FEATURE_PARK
//#define OPTION_AZ_MANUAL_ROTATE_LIMITS    // this option will automatically stop the L and R commands when hitting a CCW or CW limit (settings below - AZ_MANUAL_ROTATE_*_LIMIT) 
//#define OPTION_EL_MANUAL_ROTATE_LIMITS
//#define OPTION_EASYCOM_AZ_QUERY_COMMAND // Adds non-standard Easycom command: AZ with no parm returns current azimuth
//#define OPTION_EASYCOM_EL_QUERY_COMMAND // Adds non-standard Easycom command: EL with no parm returns current elevation
//#define OPTION_C_COMMAND_SENDS_AZ_AND_EL  // uncomment this when using Yaesu emulation with Ham Radio Deluxe
//#define OPTION_DELAY_C_CMD_OUTPUT         // uncomment this when using Yaesu emulation with Ham Radio Deluxe
#define FEATURE_ONE_DECIMAL_PLACE_HEADINGS
//#define FEATURE_AZIMUTH_CORRECTION        // correct the azimuth using a calibration table below
//#define FEATURE_ELEVATION_CORRECTION      // correct the elevation using a calibration table below
//#define FEATURE_ANCILLARY_PIN_CONTROL     // control I/O pins with serial commands \F, \N, \P
//#define FEATURE_JOYSTICK_CONTROL          // analog joystick support
//#define OPTION_JOYSTICK_REVERSE_X_AXIS
//#define OPTION_JOYSTICK_REVERSE_Y_AXIS
#define OPTION_EL_SPEED_FOLLOWS_AZ_SPEED    // changing the azimith speed with Yaesu X commands or an azimuth speed pot will also change elevation speed

  /*
  
  Note:
  
  Ham Radio Deluxe expects AZ and EL in output for Yaesu C command in AZ/EL mode.  I'm not sure if this is default behavior for
  the Yaesu interface since the C2 command is supposed to be for AZ and EL.  If you have problems with other software with this code in AZ/EL mode,
  uncomment #define OPTION_C_COMMAND_SENDS_AZ_AND_EL.
  
  */

/* ---------------------- debug stuff - don't touch unless you know what you are doing --------------------------- */



#define DEFAULT_DEBUG_STATE 0  // this should be set to zero unless you're debugging something at startup

//#define DEBUG_MEMORY
//#define DEBUG_BUTTONS
//#define DEBUG_SERIAL
//#define DEBUG_SERVICE_REQUEST_QUEUE
//#define DEBUG_EEPROM
//#define DEBUG_AZ_SPEED_POT
//#define DEBUG_AZ_PRESET_POT
//#define DEBUG_PRESET_ENCODERS
//#define DEBUG_AZ_MANUAL_ROTATE_LIMITS
//#define DEBUG_BRAKE
//#define DEBUG_OVERLAP
//#define DEBUG_DISPLAY
//#define DEBUG_AZ_CHECK_OPERATION_TIMEOUT
//#define DEBUG_TIMED_BUFFER
//#define DEBUG_EL_CHECK_OPERATION_TIMEOUT
//#define DEBUG_VARIABLE_OUTPUTS
//#define DEBUG_ROTATOR
//#define DEBUG_SUBMIT_REQUEST
//#define DEBUG_SERVICE_ROTATION
//#define DEBUG_POSITION_ROTARY_ENCODER
//#define DEBUG_PROFILE_LOOP_TIME
//#define DEBUG_POSITION_PULSE_INPUT
//#define DEBUG_ACCEL
//#define DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
//#define DEBUG_HEADING_READING_TIME
//#define DEBUG_JOYSTICK
//#define DEBUG_ROTATION_INDICATION_PIN

