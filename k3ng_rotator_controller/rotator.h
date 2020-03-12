/*---------------------- macros - don't touch these unless you know what you are doing ---------------------*/
#define AZ 1
#define EL 2
 
#define DIR_CCW 0x10                      // CW Encoder Code (Do not change)
#define DIR_CW 0x20                       // CCW Encoder Code (Do not change)

#define BRAKE_RELEASE_OFF 0
#define BRAKE_RELEASE_ON 1

//az_state
#define IDLE 0
#define SLOW_START_CW 1
#define SLOW_START_CCW 2
#define NORMAL_CW 3
#define NORMAL_CCW 4
#define SLOW_DOWN_CW 5
#define SLOW_DOWN_CCW 6
#define INITIALIZE_SLOW_START_CW 7
#define INITIALIZE_SLOW_START_CCW 8
#define INITIALIZE_TIMED_SLOW_DOWN_CW 9
#define INITIALIZE_TIMED_SLOW_DOWN_CCW 10
#define TIMED_SLOW_DOWN_CW 11
#define TIMED_SLOW_DOWN_CCW 12
#define INITIALIZE_DIR_CHANGE_TO_CW 13
#define INITIALIZE_DIR_CHANGE_TO_CCW 14
#define INITIALIZE_NORMAL_CW 15
#define INITIALIZE_NORMAL_CCW 16

//el_state
#define SLOW_START_UP 1
#define SLOW_START_DOWN 2
#define NORMAL_UP 3
#define NORMAL_DOWN 4
#define SLOW_DOWN_DOWN 5
#define SLOW_DOWN_UP 6
#define INITIALIZE_SLOW_START_UP 7
#define INITIALIZE_SLOW_START_DOWN 8
#define INITIALIZE_TIMED_SLOW_DOWN_UP 9
#define INITIALIZE_TIMED_SLOW_DOWN_DOWN 10
#define TIMED_SLOW_DOWN_UP 11
#define TIMED_SLOW_DOWN_DOWN 12
#define INITIALIZE_DIR_CHANGE_TO_UP 13
#define INITIALIZE_DIR_CHANGE_TO_DOWN 14
#define INITIALIZE_NORMAL_UP 15
#define INITIALIZE_NORMAL_DOWN 16

//az_request & el_request
#define REQUEST_STOP 0
#define REQUEST_AZIMUTH 1
#define REQUEST_AZIMUTH_RAW 2
#define REQUEST_CW 3
#define REQUEST_CCW 4
#define REQUEST_UP 5
#define REQUEST_DOWN 6
#define REQUEST_ELEVATION 7
#define REQUEST_KILL 8

#define DEACTIVATE 0
#define ACTIVATE 1

#define CW 1
#define CCW 2
#define STOP_AZ 3
#define STOP_EL 4
#define UP 5
#define DOWN 6
#define STOP 7

//az_request_queue_state & el_request_queue_state
#define NONE 0
#define IN_QUEUE 1
#define IN_PROGRESS_TIMED 2
#define IN_PROGRESS_TO_TARGET 3

#define EMPTY 0
#define LOADED_AZIMUTHS 1
#define RUNNING_AZIMUTHS 2
#define LOADED_AZIMUTHS_ELEVATIONS 3
#define RUNNING_AZIMUTHS_ELEVATIONS 4

#define RED           0x1
#define YELLOW        0x3
#define GREEN         0x2
#define TEAL          0x6
#define BLUE          0x4
#define VIOLET        0x5
#define WHITE         0x7

#define LCD_UNDEF 0  
#define LCD_HEADING 1 
#define LCD_IDLE_STATUS 2
#define LCD_TARGET_AZ 3
#define LCD_TARGET_EL 4
#define LCD_TARGET_AZ_EL 5
#define LCD_ROTATING_CW 6
#define LCD_ROTATING_CCW 7
#define LCD_ROTATING_TO 8
#define LCD_ELEVATING_TO 9
#define LCD_ELEVATING_UP 10
#define LCD_ELEVATING_DOWN 11
#define LCD_ROTATING_AZ_EL 12
#define LCD_PARKED 13

#define ENCODER_IDLE          0
#define ENCODER_AZ_PENDING    1
#define ENCODER_EL_PENDING    2
#define ENCODER_AZ_EL_PENDING 3

#define NOT_DOING_ANYTHING 0
#define ROTATING_CW 1
#define ROTATING_CCW 2
#define ROTATING_UP 3
#define ROTATING_DOWN 4

#define REMOTE_UNIT_NO_COMMAND 0
#define REMOTE_UNIT_AZ_COMMAND 1
#define REMOTE_UNIT_EL_COMMAND 2
#define REMOTE_UNIT_OTHER_COMMAND 3
#define REMOTE_UNIT_AW_COMMAND 4
#define REMOTE_UNIT_DHL_COMMAND 5
#define REMOTE_UNIT_DOI_COMMAND 6
#define REMOTE_UNIT_CL_COMMAND 7
#define REMOTE_UNIT_RC_COMMAND 8
#define REMOTE_UNIT_GS_COMMAND 9

#define NOT_PARKED 0
#define PARK_INITIATED 1
#define PARKED 2

#define COORDINATES 1
#define MAIDENHEAD 2

#define FREE_RUNNING 0 
#define GPS_SYNC 1
#define RTC_SYNC 2
#define SLAVE_SYNC 3
#define SLAVE_SYNC_GPS 4

#define CONTROL_PORT0 1
#define ETHERNET_PORT0 2
#define ETHERNET_PORT1 4

#define CLIENT_INACTIVE 0
#define CLIENT_ACTIVE 1

#define LEFT 1
#define RIGHT 2
#define CENTER 3

#define STEPPER_UNDEF 0
#define STEPPER_CW 1
#define STEPPER_CCW 2
#define STEPPER_UP 3
#define STEPPER_DOWN 4

#define ETHERNET_SLAVE_DISCONNECTED 0
#define ETHERNET_SLAVE_CONNECTED 1

#define AUTOCORRECT_INACTIVE 0
#define AUTOCORRECT_WAITING_AZ 1
#define AUTOCORRECT_WAITING_EL 2
#define AUTOCORRECT_WATCHING_AZ 3
#define AUTOCORRECT_WATCHING_EL 4

#define AZ_DISPLAY_MODE_NORMAL 0
#define AZ_DISPLAY_MODE_RAW 1
#define AZ_DISPLAY_MODE_OVERLAP_PLUS 2

#define AUDIBLE_ALERT_SERVICE 0
#define AUDIBLE_ALERT_ACTIVATE 1

#define DBG_CHECK_BUTTONS_SUN 251
#define DBG_CHECK_BUTTONS_MOON 252
#define DBG_SERVICE_SUN_TRACKING 253
#define DBG_SERVICE_MOON_TRACKING 254

/* ------end of macros ------- */

