

/*        AZ Test setup   */


/* -------------------------------------   Pin Definitions ------------------------------------------ 

  You need to look at these and set them appropriately !

  Most pins can be disabled by setting them to 0 (zero).  If you're not using a pin or function, set it to 0.

  Pins > 99 = remote unit pin, for example: pin 109 = pin 9 on remote unit, pin A0+100 = pin A0 on remote unit

*/

/* azimuth pins --------------------- (use just the azimuth pins for an azimuth-only rotator) */

#define rotate_cw 31 //6              // goes high to activate rotator R (CW) rotation - pin 1 on Yaesu connector
#define rotate_ccw 7             // goes high to activate rotator L (CCW) rotation - pin 2 on Yaesu connector
#define rotate_cw_ccw  35         // goes high for both CW and CCW rotation
#define rotate_cw_pwm 0          // optional - PWM CW output - set to 0 to disable (must be PWM capable pin)
#define rotate_ccw_pwm 0         // optional - PWM CCW output - set to 0 to disable (must be PWM capable pin)
#define rotate_cw_ccw_pwm 0      // optional - PWM on CW and CCW output - set to 0 to disable (must be PWM capable pin)
#define rotate_cw_freq 0         // optional - CW variable frequency output
#define rotate_ccw_freq 0        // optional - CCW variable frequency output
#define button_cw 49 //43              // normally open button to ground for manual CW rotation (schematic pin: A1)
#define button_ccw 45 //39             // normally open button to ground for manual CCW rotation (schematic pin: A2)
#define serial_led 0             // LED blinks when command is received on serial port (set to 0 to disable)
#define rotator_analog_az A0     // reads analog azimuth voltage from rotator - pin 4 on Yaesu connector
#define azimuth_speed_voltage 0  // optional - PWM output for speed control voltage feed into rotator (on continually unlike rotate_cw_pwm and rotate_ccw_pwm)
#define overlap_led 0            // line goes high when azimuth rotator is in overlap (> 360 rotators)
#define brake_az 0               // goes high to disengage azimuth brake (set to 0 to disable)
#define az_speed_pot 0           // connect to wiper of 1K to 10K potentiometer for speed control (set to 0 to disable)
#define az_preset_pot 0          // connect to wiper of 1K to 10K potentiometer for preset control (set to 0 to disable)
#define preset_start_button 0    // connect to momentary switch (ground on button press) for preset start (set to 0 to disable or for preset automatic start)
#define button_stop 41            // connect to momentary switch (ground on button press) for preset stop (set to 0 to disable or for preset automatic start)
#define rotation_indication_pin 47
#define blink_led 0
#define az_stepper_motor_pulse 29 //33 //0
#define az_stepper_motor_direction 0//31 //37


/*----------- elevation pins --------------*/
#ifdef FEATURE_ELEVATION_CONTROL
#define rotate_up 0               // goes high to activate rotator elevation up
#define rotate_down 0             // goes high to activate rotator elevation down
#define rotate_up_or_down 27       // goes high when elevation up or down is activated
#define rotate_up_pwm 0           // optional - PWM UP output - set to 0 to disable (must be PWM capable pin)
#define rotate_down_pwm 0         // optional - PWM DOWN output - set to 0 to disable (must be PWM capable pin)
#define rotate_up_down_pwm 0      // optional - PWM on both UP and DOWN (must be PWM capable pin)
#define rotate_up_freq 0          // optional - UP variable frequency output
#define rotate_down_freq 0        // optional - UP variable frequency output
#define rotator_analog_el A1      // reads analog elevation voltage from rotator
#define button_up 0 //49               // normally open button to ground for manual up elevation
#define button_down 0 //45             // normally open button to ground for manual down rotation
#define brake_el 0                // goes high to disengage elevation brake (set to 0 to disable)
#define elevation_speed_voltage  0 // optional - PWM output for speed control voltage feed into rotator (on continually unlike rotate_up_pwm and rotate_down_pwm)
#define el_stepper_motor_pulse 33 //29 //0 //29
#define el_stepper_motor_direction 37 //31 //0 //31
#endif //FEATURE_ELEVATION_CONTROL

// rotary encoder pins and options
#ifdef FEATURE_AZ_PRESET_ENCODER 
#define az_rotary_preset_pin1 0                     // CW Encoder Pin
#define az_rotary_preset_pin2 0                     // CCW Encoder Pin
#endif //FEATURE_AZ_PRESET_ENCODER

#ifdef FEATURE_EL_PRESET_ENCODER 
#define el_rotary_preset_pin1 0                     // UP Encoder Pin
#define el_rotary_preset_pin2 0                     // DOWN Encoder Pin
#endif //FEATURE_EL_PRESET_ENCODER

#ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER
#define az_rotary_position_pin1 0                     // CW Encoder Pin
#define az_rotary_position_pin2 0                     // CCW Encoder Pin
#endif //FEATURE_AZ_POSITION_ROTARY_ENCODER

#ifdef FEATURE_EL_POSITION_ROTARY_ENCODER
#define el_rotary_position_pin1 0                     // CW Encoder Pin
#define el_rotary_position_pin2 0                     // CCW Encoder Pin
#endif //FEATURE_EL_POSITION_ROTARY_ENCODER

#ifdef FEATURE_AZ_POSITION_PULSE_INPUT
#define az_position_pulse_pin 0                       // must be an interrupt capable pin!
#define AZ_POSITION_PULSE_PIN_INTERRUPT 0             // Uno: pin 2 = interrupt 0, pin 3 = interrupt 1 ; Mega: pin 2 = interrupt 0, pin 3 = interrupt 1, pin 21 = interrupt 2, pin 20 = interrupt 3, pin 19 = interrupt 4, pin 18 = interrupt 5
#endif                                                // read http://arduino.cc/en/Reference/AttachInterrupt for details on hardware and interrupts

#ifdef FEATURE_EL_POSITION_PULSE_INPUT
#define el_position_pulse_pin 1                       // must be an interrupt capable pin!
#define EL_POSITION_PULSE_PIN_INTERRUPT 1             // Uno: pin 2 = interrupt 0, pin 3 = interrupt 1 ; Mega: pin 2 = interrupt 0, pin 3 = interrupt 1, pin 21 = interrupt 2, pin 20 = interrupt 3, pin 19 = interrupt 4, pin 18 = interrupt 5
#endif                                                // read http://arduino.cc/en/Reference/AttachInterrupt for details on hardware and interrupts

#ifdef FEATURE_PARK
#define button_park 0
#endif

//classic 4 bit LCD pins
#define lcd_4_bit_rs_pin 12
#define lcd_4_bit_enable_pin 11
#define lcd_4_bit_d4_pin 5
#define lcd_4_bit_d5_pin 4
#define lcd_4_bit_d6_pin 3
#define lcd_4_bit_d7_pin 2


#ifdef FEATURE_JOYSTICK_CONTROL
#define pin_joystick_x A0
#define pin_joystick_y A1
#endif //FEATURE_JOYSTICK_CONTROL

#ifdef FEATURE_AZ_POSITION_HH12_AS5045_SSI
#define az_hh12_clock_pin 11
#define az_hh12_cs_pin 12
#define az_hh12_data_pin 13
#endif //FEATURE_AZ_POSITION_HH_12

#ifdef FEATURE_EL_POSITION_HH12_AS5045_SSI
#define el_hh12_clock_pin 11
#define el_hh12_cs_pin 12
#define el_hh12_data_pin 13
#endif //FEATURE_EL_POSITION_HH_12

#ifdef FEATURE_PARK
#define park_in_progress_pin 0  // goes high when a park has been initiated and rotation is in progress
#define parked_pin 0            // goes high when in a parked position
#endif //FEATURE_PARK

#define heading_reading_inhibit_pin 0 // input - a high will cause the controller to suspend taking azimuth (and elevation) readings; use when RF interferes with sensors

#ifdef FEATURE_LIMIT_SENSE
#define az_limit_sense_pin 0  // input - low stops azimuthal rotation
#define el_limit_sense_pin 0  // input - low stops elevation rotation
#endif //FEATURE_LIMIT_SENSE

#ifdef FEATURE_AZ_POSITION_INCREMENTAL_ENCODER
#define az_incremental_encoder_pin_phase_a 2 //18 //3 must be an interrupt capable pin
#define az_incremental_encoder_pin_phase_b 3 //19 //3 // must be an interrupt capable pin
#define az_incremental_encoder_pin_phase_z 5 //15 //4
#define AZ_POSITION_INCREMENTAL_ENCODER_A_PIN_INTERRUPT 0 //5 //0             // Uno: pin 2 = interrupt 0, pin 3 = interrupt 1 ; Mega: pin 2 = interrupt 0, pin 3 = interrupt 1, pin 21 = interrupt 2, pin 20 = interrupt 3, pin 19 = interrupt 4, pin 18 = interrupt 5
#define AZ_POSITION_INCREMENTAL_ENCODER_B_PIN_INTERRUPT 1 //4 //1             // Uno: pin 2 = interrupt 0, pin 3 = interrupt 1 ; Mega: pin 2 = interrupt 0, pin 3 = interrupt 1, pin 21 = interrupt 2, pin 20 = interrupt 3, pin 19 = interrupt 4, pin 18 = interrupt 5
                                                                              // read http://arduino.cc/en/Reference/AttachInterrupt for details on hardware and interrupts
#endif //FEATURE_AZ_POSITION_INCREMENTAL_ENCODER

#ifdef FEATURE_EL_POSITION_INCREMENTAL_ENCODER
#define el_incremental_encoder_pin_phase_a 18 //2 //2 // must be an interrupt capable pin
#define el_incremental_encoder_pin_phase_b 19 //3 //3 // must be an interrupt capable pin
#define el_incremental_encoder_pin_phase_z 15 //5 //4
#define EL_POSITION_INCREMENTAL_ENCODER_A_PIN_INTERRUPT 5 //0 //0             // Uno: pin 2 = interrupt 0, pin 3 = interrupt 1 ; Mega: pin 2 = interrupt 0, pin 3 = interrupt 1, pin 21 = interrupt 2, pin 20 = interrupt 3, pin 19 = interrupt 4, pin 18 = interrupt 5
#define EL_POSITION_INCREMENTAL_ENCODER_B_PIN_INTERRUPT 4 //1 //1             // Uno: pin 2 = interrupt 0, pin 3 = interrupt 1 ; Mega: pin 2 = interrupt 0, pin 3 = interrupt 1, pin 21 = interrupt 2, pin 20 = interrupt 3, pin 19 = interrupt 4, pin 18 = interrupt 5
                                                                              // read http://arduino.cc/en/Reference/AttachInterrupt for details on hardware and interrupts
#endif //FEATURE_EL_POSITION_INCREMENTAL_ENCODER

#ifdef FEATURE_YOURDUINO_I2C_LCD
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
#endif //FEATURE_YOURDUINO_I2C_LCD

#ifdef FEATURE_MOON_TRACKING
#define moon_tracking_active_pin 0    // goes high when moon tracking is active
#define moon_tracking_activate_line 0 // ground this pin to activate moon tracking (not for use with a button)
#define moon_tracking_button 0        // use with a normally open momentary switch to ground
#endif //FEATURE_MOON_TRACKING

#ifdef FEATURE_SUN_TRACKING
#define sun_tracking_active_pin 0    // goes high when sun tracking is active
#define sun_tracking_activate_line 0 // ground this pin to activate sun tracking (not for use with a button)
#define sun_tracking_button 0        // use with a normally open momentary switch to ground
#endif //FEATURE_SUN_TRACKING

#ifdef FEATURE_GPS
#define gps_sync 0
#endif //FEATURE_GPS

#ifdef FEATURE_POWER_SWITCH
#define power_switch 0             // use with FEATURE_POWER_SWITCH
#endif //FEATURE_POWER_SWITCH

#ifdef FEATURE_EL_POSITION_MEMSIC_2125
#define pin_memsic_2125_x 0
#define pin_memsic_2125_y 0
#endif //FEATURE_EL_POSITION_MEMSIC_2125

#ifdef FEATURE_ANALOG_OUTPUT_PINS
#define pin_analog_az_out 0
#define pin_analog_el_out 0
#endif //FEATURE_ANALOG_OUTPUT_PINS


// #define pin_led_cw 0
// #define pin_led_ccw 0
// #define pin_led_up 0
// #define pin_led_down 0

#ifdef FEATURE_AUTOPARK
  #define pin_autopark_disable 0       // Pull low to disable autopark 
  #define pin_autopark_timer_reset 0   // Pull low to reset the autopark timer (tie in with rig PTT) 
#endif   

#ifdef FEATURE_AUDIBLE_ALERT
  #define pin_audible_alert 0
#endif    
