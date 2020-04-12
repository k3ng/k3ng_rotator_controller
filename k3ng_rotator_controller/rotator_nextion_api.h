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

#define NEXTION_API_PAGE_ID 0

//-----------------------------------------------------------------------------------------------------------------------------


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
            // DATATYPE:    String
            // DATA FLOW:   Rotator Controller -> Nextion
            // PURPOSE:     Send a text string argument with a command to the Nextion
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG1_OBJECT_ID 9                      
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG1_OBJECT_NAME "v009NxCmdStr1"

            // DESCRIPTION: Nextion Unit Command String Argument 2
            // DATATYPE:    String
            // DATA FLOW:   Rotator Controller -> Nextion
            // PURPOSE:     Send a text string argument with a command to the Nextion
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG2_OBJECT_ID 10                   
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG2_OBJECT_NAME "v010NxCmdStr2"

            // DESCRIPTION: Nextion Unit Command String Argument 3
            // DATATYPE:    String
            // DATA FLOW:   Rotator Controller -> Nextion
            // PURPOSE:     Send a text string argument with a command to the Nextion
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG3_OBJECT_ID 11                      
            #define NEXTION_API_NEXTION_COMMAND_STR_ARG3_OBJECT_NAME "v011NxCmdStr3"

//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------



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
    #define NEXTION_API_RC_COMMAND_ROTATE_CCW 4
    #define NEXTION_API_RC_COMMAND_ROTATE_CW 5
    #define NEXTION_API_RC_COMMAND_UP 6
    #define NEXTION_API_RC_COMMAND_DOWN 7
    #define NEXTION_API_RC_COMMAND_ROTATE_AZ_TO 8
    #define NEXTION_API_RC_COMMAND_ROTATE_EL_TO 9
    #define NEXTION_API_RC_COMMAND_SET_CLOCK 10


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
            // DATATYPE:    String
            // DATA FLOW:   Rotator Controller <- Nextion
            // PURPOSE:     Send a text string argument with a command to the Rotator Controller
            #define NEXTION_API_RC_COMMAND_STR_ARG1_OBJECT_ID 16                      
            #define NEXTION_API_RC_COMMAND_STR_ARG1_OBJECT_NAME "v016RCCmdStr1"

            // DESCRIPTION: Rotator Controller Command String Argument 2
            // DATATYPE:    String
            // DATA FLOW:   Rotator Controller <- Nextion
            // PURPOSE:     Send a text string argument with a command to the Rotator Controller
            #define NEXTION_API_RC_COMMAND_STR_ARG2_OBJECT_ID 17                   
            #define NEXTION_API_RC_COMMAND_STR_ARG2_OBJECT_NAME "v017RCCmdStr2"

            // DESCRIPTION: Rotator Controller Command String Argument 3
            // DATATYPE:    String
            // DATA FLOW:   Rotator Controller <- Nextion
            // PURPOSE:     Send a text string argument with a command to the Rotator Controller
            #define NEXTION_API_RC_COMMAND_STR_ARG3_OBJECT_ID 18                      
            #define NEXTION_API_RC_COMMAND_STR_ARG3_OBJECT_NAME "v018RCCmdStr3"

//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------


// DESCRIPTION: System Capabilities
// DATATYPE:    Numeric
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Set the supported capabilities
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

//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

// Dynamically / Frequently Updated Variables

// DESCRIPTION: Real Azimuth
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current real azimuth (0 - 360 degrees).  Since the Nextion only handles integers,
//              this is the real azimuth * 1000
#define NEXTION_API_AZ_OBJECT_ID 20                     
#define NEXTION_API_AZ_OBJECT_NAME "v020Az"


// DESCRIPTION: Raw Azimuth
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current raw azimuth (in overlap this is equal to real azimuth + 360).  Since the Nextion only handles integers,
//              this is the raw azimuth * 1000
#define NEXTION_API_RAW_AZ_OBJECT_ID 21                     
#define NEXTION_API_RAW_AZ_OBJECT_NAME "v021AzRaw"

// DESCRIPTION: Elevation
// DATATYPE:    Number
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     Stores the current elevation.  Since the Nextion only handles integers,
//              this is the elevation * 1000
#define NEXTION_API_EL_OBJECT_ID 22                     
#define NEXTION_API_EL_OBJECT_NAME "v022El"





   

// DESCRIPTION:
// DATATYPE:    Numeric
// DATA FLOW:   Rotator Controller -> Nextion
// PURPOSE:     


NexVariable vNextionAPIVersion = NexVariable(NEXTION_API_PAGE_ID, NEXTION_API_NEXTION_API_VERSION_OBJECT_ID, NEXTION_API_NEXTION_API_VERSION_OBJECT_NAME);