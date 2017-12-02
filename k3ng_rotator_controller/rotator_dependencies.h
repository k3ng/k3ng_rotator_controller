
/* ---------------------- dependency checking - don't touch this unless you know what you are doing ---------------------*/

#if defined(FEATURE_YAESU_EMULATION) && defined(FEATURE_EASYCOM_EMULATION)
#error "You can't activate both FEATURE_YAESU_EMULATION and FEATURE_EASYCOM_EMULATION!"
#endif

#if (defined(FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT) || defined(FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT)) && (!defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && !defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE))
#error "You must activate FEATURE_MASTER_WITH_SERIAL_SLAVE or FEATURE_MASTER_WITH_ETHERNET_SLAVE when using FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT or FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT"
#endif

#if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
#error "You cannot active both FEATURE_MASTER_WITH_SERIAL_SLAVE and FEATURE_MASTER_WITH_ETHERNET_SLAVE"
#endif

#if defined(FEATURE_EL_POSITION_PULSE_INPUT) && !defined(FEATURE_ELEVATION_CONTROL)
#undef FEATURE_EL_POSITION_PULSE_INPUT
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && (defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE))
#error "You cannot make this unit be both a master and a slave"
#endif

#if defined(FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT) && (defined(FEATURE_AZ_POSITION_POTENTIOMETER)||defined(FEATURE_AZ_POSITION_ROTARY_ENCODER)||defined(FEATURE_AZ_POSITION_PULSE_INPUT)||defined(FEATURE_AZ_POSITION_HMC5883L)||defined(FEATURE_AZ_POSITION_INCREMENTAL_ENCODER))
#error "You cannot get AZ position from remote unit and have a local azimuth sensor defined"
#endif

#if defined(FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT) && (defined(FEATURE_EL_POSITION_POTENTIOMETER)||defined(FEATURE_EL_POSITION_ROTARY_ENCODER)||defined(FEATURE_EL_POSITION_PULSE_INPUT)||defined(FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB)||defined(FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB)||defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER))
#error "You cannot get EL position from remote unit and have a local elevation sensor defined"
#endif

#if (defined(FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB) && defined(FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB))
#error "You must select only one one library for the ADXL345"
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && defined(FEATURE_YAESU_EMULATION)
#error "You must turn off FEATURE_YAESU_EMULATION if using FEATURE_REMOTE_UNIT_SLAVE"
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && defined(FEATURE_EASYCOM_EMULATION)
#error "You must turn off FEATURE_EASYCOM_EMULATION if using FEATURE_REMOTE_UNIT_SLAVE"
#endif


#if !defined(FEATURE_ELEVATION_CONTROL) && defined(FEATURE_EL_PRESET_ENCODER)
#undef FEATURE_EL_PRESET_ENCODER 
#endif

#if !defined(FEATURE_AZ_POSITION_POTENTIOMETER) && !defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) && !defined(FEATURE_AZ_POSITION_PULSE_INPUT) && !defined(FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT) && !defined(FEATURE_AZ_POSITION_HMC5883L)  && !defined(FEATURE_AZ_POSITION_ADAFRUIT_LSM303) && !defined(FEATURE_AZ_POSITION_HH12_AS5045_SSI) &&!defined(FEATURE_AZ_POSITION_INCREMENTAL_ENCODER) &&!defined(FEATURE_AZ_POSITION_POLOLU_LSM303) &&!defined(FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER) &&!defined(FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
#error "You must specify one AZ position sensor feature"
#endif

#if defined(FEATURE_ELEVATION_CONTROL) && !defined(FEATURE_EL_POSITION_POTENTIOMETER) && !defined(FEATURE_EL_POSITION_ROTARY_ENCODER) && !defined(FEATURE_EL_POSITION_PULSE_INPUT) && !defined(FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB) && !defined(FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB) && !defined(FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT) && !defined(FEATURE_EL_POSITION_ADAFRUIT_LSM303) && !defined(FEATURE_EL_POSITION_HH12_AS5045_SSI) && !defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER) && !defined(FEATURE_EL_POSITION_MEMSIC_2125) &&!defined(FEATURE_EL_POSITION_POLOLU_LSM303) && !defined(FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER) && !defined(FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
#error "You must specify one EL position sensor feature"
#endif


#if (defined(FEATURE_AZ_PRESET_ENCODER) || defined(FEATURE_EL_PRESET_ENCODER) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER)) && !defined(FEATURE_ROTARY_ENCODER_SUPPORT)
#define FEATURE_ROTARY_ENCODER_SUPPORT
#endif


#if defined(FEATURE_REMOTE_UNIT_SLAVE) && !defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS)
#define FEATURE_ONE_DECIMAL_PLACE_HEADINGS
#endif

#if defined(FEATURE_4_BIT_LCD_DISPLAY) || defined(FEATURE_I2C_LCD) || defined(FEATURE_ADAFRUIT_I2C_LCD) || defined(FEATURE_YOURDUINO_I2C_LCD) || defined(FEATURE_RFROBOT_I2C_DISPLAY) || defined(FEATURE_YWROBOT_I2C_DISPLAY) || defined(FEATURE_SAINSMART_I2C_LCD) || defined(FEATURE_GLCD_DISPLAY)
#define FEATURE_LCD_DISPLAY
#endif

#if defined(FEATURE_ADAFRUIT_I2C_LCD) || defined(FEATURE_YOURDUINO_I2C_LCD) || defined(FEATURE_RFROBOT_I2C_DISPLAY) || defined(FEATURE_YWROBOT_I2C_DISPLAY) || defined(FEATURE_SAINSMART_I2C_LCD)
#define FEATURE_I2C_LCD
#endif

#if defined(FEATURE_MOON_TRACKING) && !defined(FEATURE_ELEVATION_CONTROL)
#error "FEATURE_MOON_TRACKING requires FEATURE_ELEVATION_CONTROL"
#endif

#if (defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)) && !defined(FEATURE_CLOCK)
#error "FEATURE_MOON_TRACKING and FEATURE_SUN_TRACKING requires a clock feature to be activated"
#endif

#if defined(FEATURE_GPS) && !defined(FEATURE_CLOCK)
#define FEATURE_CLOCK
#endif

#if defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) && defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
#error "You need to pick either FEATURE_ONE_DECIMAL_PLACE_HEADINGS or FEATURE_TWO_DECIMAL_PLACE_HEADINGS (or turn both off)"
#endif

#if defined(FEATURE_RTC_DS1307)|| defined(FEATURE_RTC_PCF8583) 
#define FEATURE_RTC
#endif

#if defined(FEATURE_RTC_DS1307) || defined(FEATURE_RTC_PCF8583) || defined(FEATURE_I2C_LCD) || defined(FEATURE_AZ_POSITION_HMC5883L) || defined(FEATURE_EL_POSITION_ADAFRUIT_LSM303) || defined(FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB) || defined(FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB)
#define FEATURE_WIRE_SUPPORT
#endif

#if defined(FEATURE_RTC_DS1307) && defined(FEATURE_RTC_PCF8583)
#error "You can't have two RTC features enabled!"
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)
#error "You can't define both FEATURE_REMOTE_UNIT_SLAVE and OPTION_SYNC_MASTER_CLOCK_TO_SLAVE - use OPTION_SYNC_MASTER_CLOCK_TO_SLAVE on the master unit"
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && defined(OPTION_SYNC_MASTER_COORDINATES_TO_SLAVE)
#error "You can't define both FEATURE_REMOTE_UNIT_SLAVE and OPTION_SYNC_MASTER_COORDINATES_TO_SLAVE - use OPTION_SYNC_MASTER_COORDINATES_TO_SLAVE on the master unit"
#endif

#if defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE) && !defined(FEATURE_ETHERNET)
#error "FEATURE_MASTER_WITH_ETHERNET_SLAVE requires FEATURE_ETHERNET"
#endif

#if defined(HARDWARE_EA4TX_ARS_USB) && !defined(FEATURE_4_BIT_LCD_DISPLAY)
#define FEATURE_4_BIT_LCD_DISPLAY
#endif

#if defined(HARDWARE_EA4TX_ARS_USB) && defined(FEATURE_ELEVATION_CONTROL)
#define HACK_REDUCED_DEBUG
#endif

#if defined(FEATURE_AUTOPARK) && !defined(FEATURE_PARK)
#define FEATURE_PARK
#endif




