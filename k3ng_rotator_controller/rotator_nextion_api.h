/* Nextion display settings and macros 


API Based Solution - In Development


  IMPORTANT !


    Be sure to edit NexConfig.h in your Nextion library directory:

      Comment out line 27 as follows:
      //#define DEBUG_SERIAL_ENABLE
      
      Comment out line 32 as follows:
      //#define dbSerial Serial

      Edit line 37:
      #define nexSerial Serial    //<-- Change "Serial to whatever Arduino Serial port you're connecting the Nextion display to (probably Serial1 or Serial2)


    Also, change the following lines in NexHardware.cpp from:

      dbSerialBegin(9600);
      nexSerial.begin(9600);

    To:

      dbSerialBegin(115200);
      nexSerial.begin(115200);  

*/

#define NEXTION_API_PAGE_ID 0            // This is the page ID where all the API global variables are defined
#define NEXTION_API_VERSION 2020041201   // This API version 

//-----------------------------------------------------------------------------------------------------------------------------

/*


    API Version Declaration


*/


// DESCRIPTION: The version of the API implementation in the display unit
// DATATYPE:    Numeric
// DATA FLOW:   Rotator Controller <- Nextion
// PURPOSE:     Used to determine if their is an API version mismatch between the rotator controller and the Nextion unit
#define NEXTION_API_NEXTION_API_VERSION_OBJECT_ID 1                         
#define NEXTION_API_NEXTION_API_VERSION_OBJECT_NAME "v001NxAPIv"

//-----------------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: The version of the API implementation in the rotator controller
// DATATYPE:    Numeric
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Used to determine if their is an API version mismatch between the rotator controller and the Nextion unit
#define NEXTION_API_ROTATOR_CONTROLLER_API_VERSION_OBJECT_ID 2              
#define NEXTION_API_ROTATOR_CONTROLLER_API_VERSION_OBJECT_NAME "v002RCAPIv"

//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------


/*


    Rotator Controller and Nextion Unit System States


*/


// DESCRIPTION: The current state of the Nextion display
// DATATYPE:    Numeric
// DATA FLOW:   Rotator Controller <- Nextion
// PURPOSE:     Used to determine the current state of the Nextion display
#define NEXTION_API_NEXTION_STATE_OBJECT_ID 3                        
#define NEXTION_API_NEXTION_STATE_OBJECT_NAME "v003NxSt"
  // STATES
  #define NEXTION_API_NEXTION_STATE_UNDEFINED 0
  #define NEXTION_API_NEXTION_STATE_INITIALIZING 1
  #define NEXTION_API_NEXTION_STATE_RUNNING 2



//-----------------------------------------------------------------------------------------------------------------------------


// DESCRIPTION: The current state of the Rotator Controller
// DATATYPE:    Numeric
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Used to determine the current state of the Rotator Controller
#define NEXTION_API_RC_STATE_OBJECT_ID 4                       
#define NEXTION_API_RC_STATE_OBJECT_NAME "v004RCSt"
  // STATES
  #define NEXTION_API_RC_STATE_UNDEFINED 0
  #define NEXTION_API_RC_STATE_INITIALIZING 1
  #define NEXTION_API_RC_STATE_RUNNING 2  



//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

/*


    Rotator Controller to Nextion Command 


*/


// DESCRIPTION: Nextion Unit Command
// DATATYPE:    Numeric
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Send a command to the Nextion
#define NEXTION_API_NEXTION_COMMAND_OBJECT_ID 5                      
#define NEXTION_API_NEXTION_COMMAND_OBJECT_NAME "v005NxCmd"

    #define NEXTION_API_NEXTION_COMMAND_NO_CMD 0
    #define NEXTION_API_NEXTION_COMMAND_RESET 1

            // DESCRIPTION: Nextion Unit Command Numeric Argument 1
            // DATATYPE:    Numeric
            // DATA FLOW:   Rotator Controller -> Nextion
            // PURPOSE:     Send a numeric argument with a command to the Nextion
            #define NEXTION_API_NEXTION_COMMANDN_NUM_ARG1_OBJECT_ID 6                      
            #define NEXTION_API_NEXTION_COMMAND_NUM_ARG1_OBJECT_NAME "v006NxCmdNum1"

            // DESCRIPTION: Nextion Unit Command Numeric Argument 2
            // DATATYPE:    Numeric
            // DATA FLOW:   Rotator Controller -> Nextion
            // PURPOSE:     Send a numeric argument with a command to the Nextion
            #define NEXTION_API_NEXTION_COMMAND_NUM_ARG2_OBJECT_ID 7                      
            #define NEXTION_API_NEXTION_COMMAND_NUM_ARG2_OBJECT_NAME "v007NxCmdNum2"


            // DESCRIPTION: Nextion Unit Command Numeric Argument 3
            // DATATYPE:    Numeric
            // DATA FLOW:   Rotator Controller -> Nextion
            // PURPOSE:     Send a numeric argument with a command to the Nextion
            #define NEXTION_API_NEXTION_COMMAND_NUM_ARG3_OBJECT_ID 8                     
            #define NEXTION_API_NEXTION_COMMAND_NUM_ARG3_OBJECT_NAME "v008NxCmdNum3"


            // DESCRIPTION: Nextion Unit Command String Argument 1
            // DATATYPE:    String [32]
            // DATA FLOW:   Rotator Controller -> Nextion
            // PURPOSE:     Send a text string argument with a command to the Nextion
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG1_OBJECT_ID 9                      
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG1_OBJECT_NAME "v009NxCmdStr1"


            // DESCRIPTION: Nextion Unit Command String Argument 2
            // DATATYPE:    String [32]
            // DATA FLOW:   Rotator Controller -> Nextion
            // PURPOSE:     Send a text string argument with a command to the Nextion
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG2_OBJECT_ID 10                   
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG2_OBJECT_NAME "v010NxCmdStr2"


            // DESCRIPTION: Nextion Unit Command String Argument 3
            // DATATYPE:    String [32]
            // DATA FLOW:   Rotator Controller -> Nextion
            // PURPOSE:     Send a text string argument with a command to the Nextion
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG3_OBJECT_ID 11                      
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG3_OBJECT_NAME "v011NxCmdStr3"


//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

/*


    Nextion to Rotator Controller Command 


*/

// DESCRIPTION: Rotator Controller Command
// DATATYPE:    Numeric
// DATA FLOW:   Rotator Controller <- Nextion
// PURPOSE:     Send a command to the Rotator Controller
#define NEXTION_API_RC_COMMAND_OBJECT_ID 12                      
#define NEXTION_API_RC_COMMAND_OBJECT_NAME "v012RCCmd"

    #define NEXTION_API_RC_COMMAND_NO_CMD 0
    #define NEXTION_API_RC_COMMAND_STOP_ALL 1
    #define NEXTION_API_RC_COMMAND_STOP_AZ 2
    #define NEXTION_API_RC_COMMAND_STOP_EL 3
    #define NEXTION_API_RC_COMMAND_KILL_ALL 4
    #define NEXTION_API_RC_COMMAND_KILL_AZ 5
    #define NEXTION_API_RC_COMMAND_KILL_EL 6
    #define NEXTION_API_RC_COMMAND_ROTATE_CCW 7
    #define NEXTION_API_RC_COMMAND_ROTATE_CW 8
    #define NEXTION_API_RC_COMMAND_UP 9
    #define NEXTION_API_RC_COMMAND_DOWN 10

    #define NEXTION_API_RC_COMMAND_ROTATE_AZ_REAL_TO 20
    #define NEXTION_API_RC_COMMAND_ROTATE_AZ_RAW_TO 21
    #define NEXTION_API_RC_COMMAND_ROTATE_EL_TO 22
    #define NEXTION_API_RC_COMMAND_PARK 23
    #define NEXTION_API_RC_COMMAND_LONG_PATH 24
    #define NEXTION_API_RC_COMMAND_MOON_TRACK 25
    #define NEXTION_API_RC_COMMAND_SUN_TRACK 26

    #define NEXTION_API_RC_COMMAND_DISENGAGE_BRAKE_AZ 50
    #define NEXTION_API_RC_COMMAND_ENGAGE_BRAKE_AZ 51
    #define NEXTION_API_RC_COMMAND_DISENGAGE_BRAKE_EL 52
    #define NEXTION_API_RC_COMMAND_ENGAGE_BRAKE_EL 53

    #define NEXTION_API_RC_COMMAND_AUTOPARK_ENABLE 60
    #define NEXTION_API_RC_COMMAND_AUTOPARK_DISABLE 61
    #define NEXTION_API_RC_COMMAND_AUTOPARK_SET_TIMER 62

    #define NEXTION_API_RC_COMMAND_CALIB_AZ_OFFSET 100
    #define NEXTION_API_RC_COMMAND_CALIB_AZ_FULLSCALE 101
    #define NEXTION_API_RC_COMMAND_CALIB_EL_0 102
    #define NEXTION_API_RC_COMMAND_CALIB_EL_180 103
    #define NEXTION_API_RC_COMMAND_CALIB_AZ_EL_MOON 104
    #define NEXTION_API_RC_COMMAND_CALIB_AZ_EL_SUN 105

    #define NEXTION_API_RC_COMMAND_MAN_SET_AZ 110
    #define NEXTION_API_RC_COMMAND_MAN_SET_EL 111 

    #define NEXTION_API_RC_COMMAND_AZ_SET_START 120
    #define NEXTION_API_RC_COMMAND_AZ_SET_CAPABILITY 121

    #define NEXTION_API_RC_COMMAND_SET_PIN_HIGH 200
    #define NEXTION_API_RC_COMMAND_SET_PIN_LOW 201
    #define NEXTION_API_RC_COMMAND_SET_PIN_PWM 202
    #define NEXTION_API_RC_COMMAND_SET_PIN_TONE 203
    #define NEXTION_API_RC_COMMAND_SET_PIN_NOTONE 204

    #define NEXTION_API_RC_COMMAND_SET_CLOCK 220
    #define NEXTION_API_RC_COMMAND_SET_CLOCK_OFFSET 221

    #define NEXTION_API_RC_COMMAND_REBOOT 255

            // DESCRIPTION: Rotator Controller Command Numeric Argument 1
            // DATATYPE:    Numeric
            // DATA FLOW:   Rotator Controller <- Nextion
            // PURPOSE:     Send a numeric argument with a command to the Rotator Controller
            #define NEXTION_API_RC_COMMAND_NUM_ARG1_OBJECT_ID 13                    
            #define NEXTION_API_RC_COMMAND_NUM_ARG1_OBJECT_NAME "v013RCCmdNum1"

            // DESCRIPTION: Rotator Controller Command Numeric Argument 2
            // DATATYPE:    Numeric
            // DATA FLOW:   Rotator Controller <- Nextion
            // PURPOSE:     Send a numeric argument with a command to the Rotator Controller
            #define NEXTION_API_RC_COMMAND_NUM_ARG2_OBJECT_ID 14                     
            #define NEXTION_API_RC_COMMAND_NUM_ARG2_OBJECT_NAME "v014RCCmdNum2"

            // DESCRIPTION: Rotator Controller Command Numeric Argument 3
            // DATATYPE:    Numeric
            // DATA FLOW:   Rotator Controller <- Nextion
            // PURPOSE:     Send a numeric argument with a command to the Rotator Controller
            #define NEXTION_API_RC_COMMAND_NUM_ARG3_OBJECT_ID 15                     
            #define NEXTION_API_RC_COMMAND_NUM_ARG3_OBJECT_NAME "v015RCCmdNum3"

            // DESCRIPTION: Rotator Controller Command String Argument 1
            // DATATYPE:    String [32]
            // DATA FLOW:   Rotator Controller <- Nextion
            // PURPOSE:     Send a text string argument with a command to the Rotator Controller
            #define NEXTION_API_RC_COMMAND_STR_ARG1_OBJECT_ID 16                      
            #define NEXTION_API_RC_COMMAND_STR_ARG1_OBJECT_NAME "v016RCCmdStr1"

            // DESCRIPTION: Rotator Controller Command String Argument 2
            // DATATYPE:    String [32]
            // DATA FLOW:   Rotator Controller <- Nextion
            // PURPOSE:     Send a text string argument with a command to the Rotator Controller
            #define NEXTION_API_RC_COMMAND_STR_ARG2_OBJECT_ID 17                   
            #define NEXTION_API_RC_COMMAND_STR_ARG2_OBJECT_NAME "v017RCCmdStr2"

            // DESCRIPTION: Rotator Controller Command String Argument 3
            // DATATYPE:    String [32]
            // DATA FLOW:   Rotator Controller <- Nextion
            // PURPOSE:     Send a text string argument with a command to the Rotator Controller
            #define NEXTION_API_RC_COMMAND_STR_ARG3_OBJECT_ID 18                      
            #define NEXTION_API_RC_COMMAND_STR_ARG3_OBJECT_NAME "v018RCCmdStr3"
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

/* 


  Static / Configuration Items 


  These are typically set at runtime initialization


*/
    

// DESCRIPTION: System Capabilities
// DATATYPE:    Numeric
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Set the supported capabilities
// DATA FORMAT: Bitmapped (see bit definitions below)
#define NEXTION_API_SYSTEM_CAPABILITIES_OBJECT_ID 19                     
#define NEXTION_API_SYSTEM_CAPABILITIES_OBJECT_NAME "v019SysCap"
   // bits
   #define NEXTION_API_SYSTEM_CAPABILITIES_GS_232A 1
   #define NEXTION_API_SYSTEM_CAPABILITIES_GS_232B 2
   #define NEXTION_API_SYSTEM_CAPABILITIES_EASYCOM 4
   #define NEXTION_API_SYSTEM_CAPABILITIES_DCU_1 8
   #define NEXTION_API_SYSTEM_CAPABILITIES_ELEVATION 16
   #define NEXTION_API_SYSTEM_CAPABILITIES_CLOCK 32
   #define NEXTION_API_SYSTEM_CAPABILITIES_GPS 64
   #define NEXTION_API_SYSTEM_CAPABILITIES_MOON 128
   #define NEXTION_API_SYSTEM_CAPABILITIES_SUN 256
   #define NEXTION_API_SYSTEM_CAPABILITIES_RTC 512


// DESCRIPTION: Rotator Controller Code Version
// DATATYPE:    String [13]
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Provide rotator controller code version to Nextion unit
// EXAMPLE:     2020.04.06.01
#define NEXTION_API_SYSTEM_RC_CODE_VERSION_OBJECT_ID 20                  
#define NEXTION_API_SYSTEM_RC_CODE_OBJECT_NAME "v020RCVersion"

// DESCRIPTION: Azimuth Starting Point
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the azimuth starting point in degrees.  This is a stright integer with no multiplier.
// TYPICAL VAL: 0 or 180
#define NEXTION_API_AZ_SP_OBJECT_ID 21               
#define NEXTION_API_AZ_SP_OBJECT_NAME "v021AzSP"

// DESCRIPTION: Azimuth Rotation Capability
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the azimuth rotation capability in degrees.  This is a stright integer with no multiplier.
// TYPICAL VAL: 360 or 450
#define NEXTION_API_AZ_RC_OBJECT_ID 22            
#define NEXTION_API_AZ_RC_OBJECT_NAME "v022AzRP"




//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

// Dynamically / Frequently Updated Variables

// DESCRIPTION: Real Azimuth
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current real azimuth (0 - 360 degrees).  Since the Nextion only handles integers,
//              this is the real azimuth * 1000
#define NEXTION_API_AZ_OBJECT_ID 23                     
#define NEXTION_API_AZ_OBJECT_NAME "v023Az"

// DESCRIPTION: Raw Azimuth
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current raw azimuth (in overlap this is equal to real azimuth + 360).  Since the Nextion only handles integers,
//              this is the raw azimuth * 1000
#define NEXTION_API_RAW_AZ_OBJECT_ID 24                   
#define NEXTION_API_RAW_AZ_OBJECT_NAME "v024AzRaw"

// DESCRIPTION: Elevation
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current elevation.  Since the Nextion only handles integers,
//              this is the elevation * 1000
#define NEXTION_API_EL_OBJECT_ID 25                  
#define NEXTION_API_EL_OBJECT_NAME "v025El"

// DESCRIPTION: Azimuth Detailed State
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current state of azimuthal rotation
#define NEXTION_API_AZ_STATE_DETAILED_OBJECT_ID 26                    
#define NEXTION_API_AZ_STATE_DETAILED_OBJECT_NAME "v026AzStDet"

    // az_state - uses the same defines from rotator.h
    // #define IDLE 0
    // #define SLOW_START_CW 1
    // #define SLOW_START_CCW 2
    // #define NORMAL_CW 3
    // #define NORMAL_CCW 4
    // #define SLOW_DOWN_CW 5
    // #define SLOW_DOWN_CCW 6
    // #define INITIALIZE_SLOW_START_CW 7
    // #define INITIALIZE_SLOW_START_CCW 8
    // #define INITIALIZE_TIMED_SLOW_DOWN_CW 9
    // #define INITIALIZE_TIMED_SLOW_DOWN_CCW 10
    // #define TIMED_SLOW_DOWN_CW 11
    // #define TIMED_SLOW_DOWN_CCW 12
    // #define INITIALIZE_DIR_CHANGE_TO_CW 13
    // #define INITIALIZE_DIR_CHANGE_TO_CCW 14
    // #define INITIALIZE_NORMAL_CW 15
    // #define INITIALIZE_NORMAL_CCW 16



// DESCRIPTION: Elevation Detailed State
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current state of elvation rotation
#define NEXTION_API_EL_STATE_DETAILED_OBJECT_ID 27                    
#define NEXTION_API_EL_STATE_DETAILED_OBJECT_NAME "v027ElStDet"

    // el_state - uses the same defines from rotator.h
    // #define IDLE 0
    // #define SLOW_START_UP 1
    // #define SLOW_START_DOWN 2
    // #define NORMAL_UP 3
    // #define NORMAL_DOWN 4
    // #define SLOW_DOWN_DOWN 5
    // #define SLOW_DOWN_UP 6
    // #define INITIALIZE_SLOW_START_UP 7
    // #define INITIALIZE_SLOW_START_DOWN 8
    // #define INITIALIZE_TIMED_SLOW_DOWN_UP 9
    // #define INITIALIZE_TIMED_SLOW_DOWN_DOWN 10
    // #define TIMED_SLOW_DOWN_UP 11
    // #define TIMED_SLOW_DOWN_DOWN 12
    // #define INITIALIZE_DIR_CHANGE_TO_UP 13
    // #define INITIALIZE_DIR_CHANGE_TO_DOWN 14
    // #define INITIALIZE_NORMAL_UP 15
    // #define INITIALIZE_NORMAL_DOWN 16


// DESCRIPTION: Azimuth Overall State
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current state of azimuthal rotation
#define NEXTION_API_AZ_STATE_OBJECT_ID 28                    
#define NEXTION_API_AZ_STATE_OBJECT_NAME "v028AzState"

    // states - same ones defined in rotator.h
    // #define NOT_DOING_ANYTHING 0
    // #define ROTATING_CW 1
    // #define ROTATING_CCW 2


// DESCRIPTION: Elevation Overall State
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current state of elevation rotation
#define NEXTION_API_EL_STATE_OBJECT_ID 29                    
#define NEXTION_API_EL_STATE_OBJECT_NAME "v029ElState"

    // states - same ones defined in rotator.h
    // #define NOT_DOING_ANYTHING 0
    // #define ROTATING_UP 3
    // #define ROTATING_DOWN 4    


// DESCRIPTION: Clock Hours
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current Arduino clock hours
#define NEXTION_API_CLK_H_OBJECT_ID 30                    
#define NEXTION_API_CLK_H_OBJECT_NAME "v030ClkH"

// DESCRIPTION: Clock Minutes
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current Arduino clock minutes
#define NEXTION_API_CLK_M_OBJECT_ID 31                    
#define NEXTION_API_CLK_M_OBJECT_NAME "v031ClkM"

// DESCRIPTION: Clock Seconds
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current Arduino clock seconds
#define NEXTION_API_CLK_S_OBJECT_ID 32                    
#define NEXTION_API_CLK_S_OBJECT_NAME "v032ClkS"

// DESCRIPTION: Clock Time Zone Offset
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current Arduino clock time zone offse
// DATA FORMAT: Offset hours multiplied by 100 (i.e. 1 hour offset = 100, -5 hour offset = -500, -7.5 hour offset = -7500)
#define NEXTION_API_CLK_TIMEZONE_OBJECT_ID 33                    
#define NEXTION_API_CLK_TIMEZONE_OBJECT_NAME "v033ClkTZ"

// DESCRIPTION: Grid Locator
// DATATYPE:    String [6]
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current grid locator
// EXAMPLE:     FN20dw
#define NEXTION_API_GRID_OBJECT_ID 34                    
#define NEXTION_API_GRID_OBJECT_NAME "v034Grid"

// DESCRIPTION: GPS Coordinates Latitude
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current GPS Latitude
// DATA FORMAT: Decimal degrees multiplied by 100000 (i.e. 45.1234 degrees = 4512340, -105.12345 degrees = -10512345)
#define NEXTION_API_GPS_LAT_OBJECT_ID 35                    
#define NEXTION_API_GPS_LAT_OBJECT_NAME "v035GPSLat"

// DESCRIPTION: GPS Coordinates Longitude
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current GPS Longitude
// DATA FORMAT: Decimal degrees multiplied by 100000 (i.e. 45.1234 degrees = 4512340, -105.12345 degrees = -10512345)
#define NEXTION_API_GPS_LONG_OBJECT_ID 36                    
#define NEXTION_API_GPS_LONG_OBJECT_NAME "v036GPSLong"

// DESCRIPTION: GPS Elevation
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     The current GPS Longitude in meters
// DATA FORMAT: Decimal degrees multiplied by 100 (i.e. 450 meters = 45000)
#define NEXTION_API_GPS_ELE_OBJECT_ID 37                    
#define NEXTION_API_GPS_ELE_OBJECT_NAME "v037GPSEl"

// DESCRIPTION: Tracking, Braking, Parking, Indicator State
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Moon and Sun Tracking
// DATA FORMAT: Bitmapped
#define NEXTION_API_TRACKING_STATE_OBJECT_ID 38                    
#define NEXTION_API_TRACKING_STATE_OBJECT_NAME "v038Track"

  // bits
  #define NEXTION_STATE_TRACKING_MOON_TRACKING_ON 1
  #define NEXTION_STATE_TRACKING_SUN_TRACKING_ON 4
  #define NEXTION_STATE_TRACKING_PARK_INITIATED 8
  #define NEXTION_STATE_TRACKING_PARKED 16
  #define NEXTION_STATE_TRACKING_BRAKE_AZ_ENGAGED 32
  #define NEXTION_STATE_TRACKING_BRAKE_EL_ENGAGED 64
  #define NEXTION_STATE_TRACKING_OVERLAP 128


// DESCRIPTION: API Variable Change Flags
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     For the Rotator Controller to flag variables that have changed and give refresh hints to the Nextion display
// DATA FORMAT: Bitmapped;  The Nextion unit sets this to 0 after it has done a refresh cycle
// NOTE:        Use of this by the Nextion display and resetting it to 0 is optional.  
#define NEXTION_API_CHANGE_FLAG_OBJECT_ID 39                   
#define NEXTION_API_CHANGE_FLAG_OBJECT_NAME "v039ChFlag"

    #define NEXTION_API_CHANGE_FLAG_AZ 1
    #define NEXTION_API_CHANGE_FLAG_EL 2
    #define NEXTION_API_CHANGE_FLAG_GPS_SATS 4
    #define NEXTION_API_CHANGE_FLAG_CLOCK 8
    #define NEXTION_API_CHANGE_FLAG_COORDINATES_ELEVATION 16
    #define NEXTION_API_CHANGE_FLAG_MOON_HEADING 32
    #define NEXTION_API_CHANGE_FLAG_SUN_HEADING 64

// DESCRIPTION: Moon Azimuth
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current real azimuth (0 - 360 degrees) of the moon.  Since the Nextion only handles integers,
//              this is the azimuth * 1000
#define NEXTION_AZ_MOON_OBJECT_ID 40                    
#define NEXTION_API_AZ_MOON_OBJECT_NAME "v040AzMoon"

// DESCRIPTION: Moon Elevation
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current elevation (0 - 180 degrees) of the moon.  Since the Nextion only handles integers,
//              this is the elevation * 1000
#define NEXTION_EL_MOON_OBJECT_ID 41                   
#define NEXTION_API_EL_MOON_OBJECT_NAME "v041ElMoon"

// DESCRIPTION: Sun Azimuth
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current real azimuth (0 - 360 degrees) of the sun.  Since the Nextion only handles integers,
//              this is the azimuth * 1000
#define NEXTION_AZ_SUN_OBJECT_ID 42                    
#define NEXTION_API_AZ_SUN_OBJECT_NAME "v042AzSun"

// DESCRIPTION: Sun Elevation
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current elevation (0 - 180 degrees) of the sun.  Since the Nextion only handles integers,
//              this is the elevation * 1000
#define NEXTION_EL_SUN_OBJECT_ID 43                
#define NEXTION_API_EL_SUN_OBJECT_NAME "v043ElSun"

NexVariable vNextionAPIVersion = NexVariable(NEXTION_API_PAGE_ID, NEXTION_API_NEXTION_API_VERSION_OBJECT_ID, NEXTION_API_NEXTION_API_VERSION_OBJECT_NAME);
NexVariable vNextionRCAPIVersion = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_ROTATOR_CONTROLLER_API_VERSION_OBJECT_ID,NEXTION_API_ROTATOR_CONTROLLER_API_VERSION_OBJECT_NAME);
NexVariable vNextionNxState = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_NEXTION_STATE_OBJECT_ID,NEXTION_API_NEXTION_STATE_OBJECT_NAME);
NexVariable vNextionRCState = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_RC_STATE_OBJECT_ID,NEXTION_API_RC_STATE_OBJECT_NAME);
NexVariable vNextionNxCmd = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_NEXTION_COMMAND_OBJECT_ID,NEXTION_API_NEXTION_COMMAND_OBJECT_NAME);
NexVariable vNextionNxCmdArgNum1 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_NEXTION_COMMANDN_NUM_ARG1_OBJECT_ID,NEXTION_API_NEXTION_COMMAND_NUM_ARG1_OBJECT_NAME);
NexVariable vNextionNxCmdArgNum2 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_NEXTION_COMMAND_NUM_ARG2_OBJECT_ID,NEXTION_API_NEXTION_COMMAND_NUM_ARG2_OBJECT_NAME);
NexVariable vNextionNxCmdArgNum3 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_NEXTION_COMMAND_NUM_ARG3_OBJECT_ID,NEXTION_API_NEXTION_COMMAND_NUM_ARG3_OBJECT_NAME);
NexVariable vNextionNxCmdArgStr1 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_NEXTION_COMMAND_STR_ARG1_OBJECT_ID,NEXTION_API_NEXTION_COMMAND_STR_ARG1_OBJECT_NAME);
NexVariable vNextionNxCmdArgStr2 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_NEXTION_COMMAND_STR_ARG2_OBJECT_ID,NEXTION_API_NEXTION_COMMAND_STR_ARG2_OBJECT_NAME);
NexVariable vNextionNxCmdArgStr3 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_NEXTION_COMMAND_STR_ARG3_OBJECT_ID,NEXTION_API_NEXTION_COMMAND_STR_ARG3_OBJECT_NAME);
NexVariable vNextionRCCmd = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_RC_COMMAND_OBJECT_ID,NEXTION_API_RC_COMMAND_OBJECT_NAME);
NexVariable vNextionRCCmdArgNum1 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_RC_COMMAND_NUM_ARG1_OBJECT_ID,NEXTION_API_RC_COMMAND_NUM_ARG1_OBJECT_NAME);
NexVariable vNextionRCCmdArgNum2 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_RC_COMMAND_NUM_ARG2_OBJECT_ID,NEXTION_API_RC_COMMAND_NUM_ARG2_OBJECT_NAME);
NexVariable vNextionRCCmdArgNum3 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_RC_COMMAND_NUM_ARG3_OBJECT_ID,NEXTION_API_RC_COMMAND_NUM_ARG3_OBJECT_NAME);
NexVariable vNextionRCCmdStrNum1 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_RC_COMMAND_STR_ARG1_OBJECT_ID,NEXTION_API_RC_COMMAND_STR_ARG1_OBJECT_NAME);
NexVariable vNextionRCCmdStrNum2 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_RC_COMMAND_STR_ARG2_OBJECT_ID,NEXTION_API_RC_COMMAND_STR_ARG2_OBJECT_NAME);
NexVariable vNextionRCCmdStrNum3 = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_RC_COMMAND_STR_ARG3_OBJECT_ID,NEXTION_API_RC_COMMAND_STR_ARG3_OBJECT_NAME);
NexVariable vNextionSystemCapabilities = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_SYSTEM_CAPABILITIES_OBJECT_ID,NEXTION_API_SYSTEM_CAPABILITIES_OBJECT_NAME);
NexVariable vNextionRCCodeVersion = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_SYSTEM_RC_CODE_VERSION_OBJECT_ID,NEXTION_API_SYSTEM_RC_CODE_OBJECT_NAME);
NexVariable vNextionAzStartingPoint = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_AZ_SP_OBJECT_ID,NEXTION_API_AZ_SP_OBJECT_NAME);
NexVariable vNextionAzRotationCapability = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_AZ_RC_OBJECT_ID,NEXTION_API_AZ_RC_OBJECT_NAME);
NexVariable vNextionAz = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_AZ_OBJECT_ID,NEXTION_API_AZ_OBJECT_NAME);
NexVariable vNextionAzRaw = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_RAW_AZ_OBJECT_ID,NEXTION_API_RAW_AZ_OBJECT_NAME);
NexVariable vNextionEl = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_EL_OBJECT_ID,NEXTION_API_EL_OBJECT_NAME);
NexVariable vNextionAzStateDetailed = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_AZ_STATE_DETAILED_OBJECT_ID,NEXTION_API_AZ_STATE_DETAILED_OBJECT_NAME);
NexVariable vNextionElStateDetailed = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_EL_STATE_DETAILED_OBJECT_ID,NEXTION_API_EL_STATE_DETAILED_OBJECT_NAME);
NexVariable vNextionAzState = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_AZ_STATE_OBJECT_ID,NEXTION_API_AZ_STATE_OBJECT_NAME);
NexVariable vNextionElState = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_EL_STATE_OBJECT_ID,NEXTION_API_EL_STATE_OBJECT_NAME);
NexVariable vNextionClockHours = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_CLK_H_OBJECT_ID,NEXTION_API_CLK_H_OBJECT_NAME);
NexVariable vNextionClockMinutes = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_CLK_M_OBJECT_ID,NEXTION_API_CLK_M_OBJECT_NAME);
NexVariable vNextionClockSeconds = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_CLK_S_OBJECT_ID,NEXTION_API_CLK_S_OBJECT_NAME);
NexVariable vNextionClockTimeZoneOffset = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_CLK_TIMEZONE_OBJECT_ID,NEXTION_API_CLK_TIMEZONE_OBJECT_NAME);
NexVariable vNextionGrid = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_GRID_OBJECT_ID,NEXTION_API_GRID_OBJECT_NAME);
NexVariable vNextionGPSLatitude = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_GPS_LAT_OBJECT_ID,NEXTION_API_GPS_LAT_OBJECT_NAME);
NexVariable vNextionGPSLongitude = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_GPS_LONG_OBJECT_ID,NEXTION_API_GPS_LONG_OBJECT_NAME);
NexVariable vNextionGPSElevation = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_GPS_ELE_OBJECT_ID,NEXTION_API_GPS_ELE_OBJECT_NAME);
NexVariable vNextionTrackingAndIndicators = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_TRACKING_STATE_OBJECT_ID,NEXTION_API_TRACKING_STATE_OBJECT_NAME);
NexVariable vNextionChangeFlags = NexVariable(NEXTION_API_PAGE_ID,NEXTION_API_CHANGE_FLAG_OBJECT_ID,NEXTION_API_CHANGE_FLAG_OBJECT_NAME);
NexVariable vNextionMoonAz = NexVariable(NEXTION_API_PAGE_ID,NEXTION_AZ_MOON_OBJECT_ID,NEXTION_API_AZ_MOON_OBJECT_NAME);
NexVariable vNextionMoonEl = NexVariable(NEXTION_API_PAGE_ID,NEXTION_EL_MOON_OBJECT_ID,NEXTION_API_EL_MOON_OBJECT_NAME);
NexVariable vNextionSunAz = NexVariable(NEXTION_API_PAGE_ID,NEXTION_AZ_SUN_OBJECT_ID,NEXTION_API_AZ_SUN_OBJECT_NAME);
NexVariable vNextionSunEl = NexVariable(NEXTION_API_PAGE_ID,NEXTION_EL_SUN_OBJECT_ID,NEXTION_API_EL_SUN_OBJECT_NAME);

  

