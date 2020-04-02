/* ---------------------- debug stuff - don't touch unless you know what you are doing --------------------------- */



#define DEFAULT_DEBUG_STATE 0 // 1 = activate debug mode at startup; this should be set to zero unless you're debugging something at startup

#define DEBUG_DUMP  // normally compile with this activated unless you're really trying to save memory
// #define DEBUG_LOOP
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
// #define DEBUG_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY
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
// #define DEBUG_SYNC_MASTER_COORDINATES_TO_SLAVE
// #define DEBUG_HMC5883L
// #define DEBUG_POLOLU_LSM303_CALIBRATION
// #define DEBUG_STEPPER
// #define DEBUG_AUTOCORRECT
// #define DEBUG_A2_ENCODER
// #define DEBUG_A2_ENCODER_LOOPBACK_TEST
// #define DEBUG_QMC5883
// #define DEBUG_ROTATION_STALL_DETECTION