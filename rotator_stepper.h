//------------------------------------------------------

#if defined(FEATURE_STEPPER_MOTOR)
void service_stepper_motor_pulse_pins(){

  service_stepper_motor_pulse_pins_count++;

  static unsigned int az_stepper_pin_transition_counter = 0;
  static byte az_stepper_pin_last_state = LOW;

  if (az_stepper_freq_count > 0){
    az_stepper_pin_transition_counter++;
    if (az_stepper_pin_transition_counter >= az_stepper_freq_count){
      if (az_stepper_pin_last_state == LOW){
        digitalWrite(az_stepper_motor_pulse,HIGH);
        az_stepper_pin_last_state = HIGH;
      } else {
        digitalWrite(az_stepper_motor_pulse,LOW);
        az_stepper_pin_last_state = LOW;
      }
      az_stepper_pin_transition_counter = 0;
    }
  } else {
    az_stepper_pin_transition_counter = 0;
  }

  #ifdef FEATURE_ELEVATION_CONTROL
  static unsigned int el_stepper_pin_transition_counter = 0;
  static byte el_stepper_pin_last_state = LOW;

  if (el_stepper_freq_count > 0){
    el_stepper_pin_transition_counter++;
    if (el_stepper_pin_transition_counter >= el_stepper_freq_count){
      if (el_stepper_pin_last_state == LOW){
        digitalWrite(el_stepper_motor_pulse,HIGH);
        el_stepper_pin_last_state = HIGH;
      } else {
        digitalWrite(el_stepper_motor_pulse,LOW);
        el_stepper_pin_last_state = LOW;
      }
      el_stepper_pin_transition_counter = 0;
    }
  } else {
    el_stepper_pin_transition_counter = 0;
  }

  #endif //FEATURE_ELEVATION_CONTROL

}
#endif //defined(FEATURE_STEPPER_MOTOR)

//------------------------------------------------------
#ifdef FEATURE_STEPPER_MOTOR
void set_az_stepper_freq(unsigned int frequency){

  if (frequency > 0) {
    az_stepper_freq_count = 2000 / frequency;
  } else {
    az_stepper_freq_count = 0;
  }

  #ifdef DEBUG_STEPPER
  debug_print("set_az_stepper_freq: ");
  debug_print_int(frequency);
  debug_print(" az_stepper_freq_count:");
  debug_print_int(az_stepper_freq_count);
  debug_println("");
  #endif //DEBUG_STEPPER

}

#endif //FEATURE_STEPPER_MOTOR
//------------------------------------------------------
#if defined(FEATURE_ELEVATION_CONTROL) && defined(FEATURE_STEPPER_MOTOR)
void set_el_stepper_freq(unsigned int frequency){


  if (frequency > 0) {
    el_stepper_freq_count = 2000 / frequency;
  } else {
    el_stepper_freq_count = 0;
  }

  #ifdef DEBUG_STEPPER
  debug_print("set_el_stepper_freq: ");
  debug_print_int(frequency);
  debug_print(" el_stepper_freq_count:");
  debug_print_int(el_stepper_freq_count);
  debug_println("");
  #endif //DEBUG_STEPPER

}

#endif //defined(FEATURE_ELEVATION_CONTROL) && defined(FEATURE_STEPPER_MOTOR)
//------------------------------------------------------