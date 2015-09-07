
#ifdef FEATURE_MOON_TRACKING
void service_moon_tracking(){

  static unsigned long last_check = 0;
  static byte moon_tracking_activated_by_activate_line = 0;

  static byte moon_tracking_pin_state = 0;

  if (moon_tracking_active_pin) {
    if ((moon_tracking_active) && (!moon_tracking_pin_state)) {
      digitalWriteEnhanced(moon_tracking_active_pin, HIGH);
      moon_tracking_pin_state = 1;
    }
    if ((!moon_tracking_active) && (moon_tracking_pin_state)) {
      digitalWriteEnhanced(moon_tracking_active_pin, LOW);
      moon_tracking_pin_state = 0;
    }
  }

  if (moon_tracking_activate_line) {
    if ((!moon_tracking_active) && (!digitalReadEnhanced(moon_tracking_activate_line))) {
      moon_tracking_active = 1;
      moon_tracking_activated_by_activate_line = 1;
    }
    if ((moon_tracking_active) && (digitalReadEnhanced(moon_tracking_activate_line)) && (moon_tracking_activated_by_activate_line)) {
      moon_tracking_active = 0;
      moon_tracking_activated_by_activate_line = 0;
    }
  }

  if ((moon_tracking_active) && ((millis() - last_check) > MOON_TRACKING_CHECK_INTERVAL)) {

    update_time();
    update_moon_position();

    #ifdef DEBUG_MOON_TRACKING
    debug.print(F("service_moon_tracking: AZ: "));
    debug.print(moon_azimuth);
    debug.print(" EL: ");
    debug.print(moon_elevation);
    debug.print(" lat: ");
    debug.print(latitude);
    debug.print(" long: ");
    debug.println(longitude);
    #endif // DEBUG_MOON_TRACKING

    if ((moon_azimuth >= MOON_AOS_AZIMUTH_MIN) && (moon_azimuth <= MOON_AOS_AZIMUTH_MAX) && (moon_elevation >= MOON_AOS_ELEVATION_MIN) && (moon_elevation <= MOON_AOS_ELEVATION_MAX)) {
      submit_request(AZ, REQUEST_AZIMUTH, moon_azimuth * HEADING_MULTIPLIER, 11);
      submit_request(EL, REQUEST_ELEVATION, moon_elevation * HEADING_MULTIPLIER, 12);
      if (!moon_visible) {
        moon_visible = 1;
        #ifdef DEBUG_MOON_TRACKING
        debug.println("service_moon_tracking: moon AOS");
        #endif // DEBUG_MOON_TRACKING
      }
    } else {
      if (moon_visible) {
        moon_visible = 0;
          #ifdef DEBUG_MOON_TRACKING
        debug.println("service_moon_tracking: moon loss of AOS");
          #endif // DEBUG_MOON_TRACKING
      } else {
            #ifdef DEBUG_MOON_TRACKING
        debug.println("service_moon_tracking: moon out of AOS limits");
            #endif // DEBUG_MOON_TRACKING
      }
    }

    last_check = millis();
  }



} /* service_moon_tracking */
    #endif // FEATURE_MOON_TRACKING

// --------------------------------------------------------------

#ifdef FEATURE_SUN_TRACKING
void service_sun_tracking(){

  static unsigned long last_check = 0;
  static byte sun_tracking_pin_state = 0;
  static byte sun_tracking_activated_by_activate_line = 0;

  if (sun_tracking_active_pin) {
    if ((sun_tracking_active) && (!sun_tracking_pin_state)) {
      digitalWriteEnhanced(sun_tracking_active_pin, HIGH);
      sun_tracking_pin_state = 1;
    }
    if ((!sun_tracking_active) && (sun_tracking_pin_state)) {
      digitalWriteEnhanced(sun_tracking_active_pin, LOW);
      sun_tracking_pin_state = 0;
    }
  }

  if (sun_tracking_activate_line) {
    if ((!sun_tracking_active) && (!digitalReadEnhanced(sun_tracking_activate_line))) {
      sun_tracking_active = 1;
      sun_tracking_activated_by_activate_line = 1;
    }
    if ((sun_tracking_active) && (digitalReadEnhanced(sun_tracking_activate_line)) && (sun_tracking_activated_by_activate_line)) {
      sun_tracking_active = 0;
      sun_tracking_activated_by_activate_line = 0;
    }
  }

  if ((sun_tracking_active) && ((millis() - last_check) > SUN_TRACKING_CHECK_INTERVAL)) {

    update_time();
    update_sun_position();


    #ifdef DEBUG_SUN_TRACKING
    debug.print(F("service_sun_tracking: AZ: "));
    debug.print(sun_azimuth);
    debug.print(" EL: ");
    debug.print(sun_elevation);
    debug.print(" lat: ");
    debug.print(latitude);
    debug.print(" long: ");
    debug.println(longitude);
    #endif // DEBUG_SUN_TRACKING

    if ((sun_azimuth >= SUN_AOS_AZIMUTH_MIN) && (sun_azimuth <= SUN_AOS_AZIMUTH_MAX) && (sun_elevation >= SUN_AOS_ELEVATION_MIN) && (sun_elevation <= SUN_AOS_ELEVATION_MAX)) {
      submit_request(AZ, REQUEST_AZIMUTH, sun_azimuth * HEADING_MULTIPLIER, 13);
      submit_request(EL, REQUEST_ELEVATION, sun_elevation * HEADING_MULTIPLIER, 14);
      if (!sun_visible) {
        sun_visible = 1;
        #ifdef DEBUG_SUN_TRACKING
        debug.println("service_sun_tracking: sun AOS");
        #endif // DEBUG_SUN_TRACKING
      }
    } else {
      if (sun_visible) {
        sun_visible = 0;
          #ifdef DEBUG_SUN_TRACKING
        debug.println("service_sun_tracking: sun loss of AOS");
          #endif // DEBUG_SUN_TRACKING
      } else {
            #ifdef DEBUG_SUN_TRACKING
        debug.println("service_sun_tracking: sun out of AOS limits");
            #endif // DEBUG_SUN_TRACKING
      }
    }

    last_check = millis();
  }




} /* service_sun_tracking */
#endif // FEATURE_SUN_TRACKING

// --------------------------------------------------------------

#if defined(FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_MOON_TRACKING)
void check_moon_pushbutton_calibration(){

  static unsigned long last_update_time = 0;

  if ((digitalReadEnhanced(pin_moon_pushbutton_calibration) == LOW) && ((millis() - last_update_time) > 500)){
    update_moon_position();
    if (calibrate_az_el(moon_azimuth, moon_elevation)) {
    } else {
    }
    last_update_time = millis();
  }

}
#endif //defined(FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_MOON_TRACKING)       

//-------------------------------------------------------

#if defined(FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_SUN_TRACKING)
void check_sun_pushbutton_calibration(){

  static unsigned long last_update_time = 0;

  if ((digitalReadEnhanced(pin_sun_pushbutton_calibration) == LOW) && ((millis() - last_update_time) > 500)){
    update_sun_position();
    if (calibrate_az_el(sun_azimuth, sun_elevation)) {
    } else {
    }
    last_update_time = millis();
  }

}
#endif //defined(FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_SUN_TRACKING)       

//-------------------------------------------------------

#if defined(FEATURE_SUN_TRACKING) || defined(FEATURE_MOON_TRACKING)
char * coordinate_string(){

  char returnstring[32] = "";
  char tempstring[12] = "";

  dtostrf(latitude,0,4,returnstring);
  strcat(returnstring," ");
  dtostrf(longitude,0,4,tempstring);
  strcat(returnstring,tempstring);
  return returnstring;

}
#endif //defined(FEATURE_SUN_TRACKING) || defined(FEATURE_MOON_TRACKING)

// --------------------------------------------------------------

#ifdef FEATURE_MOON_TRACKING
char * moon_status_string(){

    char returnstring[128] = "";
    char tempstring[16] = "";

    strcpy(returnstring,"\tmoon: AZ: ");
    dtostrf(moon_azimuth,0,2,tempstring);
    strcat(returnstring,tempstring);
    strcat(returnstring," EL: ");
    dtostrf(moon_elevation,0,2,tempstring);
    strcat(returnstring,tempstring);
    strcat(returnstring,"\tTRACKING_");
    if (!moon_tracking_active) {
      strcat(returnstring,"IN");
    }
    strcat(returnstring,"ACTIVE ");
    if (moon_tracking_active) {
      if (!moon_visible) {
        strcat(returnstring,"NOT_");
      }
      strcat(returnstring,"VISIBLE");
    }
    return returnstring;
}
#endif // FEATURE_MOON_TRACKING
// --------------------------------------------------------------
#ifdef FEATURE_SUN_TRACKING
char * sun_status_string(){

    char returnstring[128] = "";
    char tempstring[16] = "";

    strcpy(returnstring,"\tsun: AZ: ");
    dtostrf(sun_azimuth,0,2,tempstring);
    strcat(returnstring,tempstring);
    strcat(returnstring," EL: ");
    dtostrf(sun_elevation,0,2,tempstring);
    strcat(returnstring,tempstring);
    strcat(returnstring,"\tTRACKING_");
    if (!sun_tracking_active) {
      strcat(returnstring,"IN");
    }
    strcat(returnstring,"ACTIVE ");
    if (sun_tracking_active) {
      if (!sun_visible) {
        strcat(returnstring,"NOT_");
      }
      strcat(returnstring,"VISIBLE");
    }
    return returnstring;
}
#endif // FEATURE_SUN_TRACKING
// --------------------------------------------------------------






