#ifdef FEATURE_CLOCK
void update_time(){
  unsigned long runtime = millis() - millis_at_last_calibration;

  unsigned long time = (3600L * clock_hour_set) + (60L * clock_min_set) + clock_sec_set + ((runtime + (runtime * INTERNAL_CLOCK_CORRECTION)) / 1000.0);

  clock_years = clock_year_set;
  clock_months = clock_month_set;
  clock_days = time / 86400L;
  time -= clock_days * 86400L;
  clock_days += clock_day_set;
  clock_hours = time / 3600L;

  switch (clock_months) {

    case 1:
    case 3:
    case 5:
    case 7:
    case 8:
    case 10:
    case 12:
      if (clock_days > 31) {
        clock_days = 1; clock_months++;
      }
      break;

    case 2:
      if ((float(clock_years) / 4.0) == 0.0) {  // do we have a leap year?
        if (clock_days > 29) {
          clock_days = 1; clock_months++;
        }
      } else {
        if (clock_days > 28) {
          clock_days = 1; clock_months++;
        }
      }
      break;

    case 4:
    case 6:
    case 9:
    case 11:
      if (clock_days > 30) {
        clock_days = 1; clock_months++;
      }
      break;
  } /* switch */

  if (clock_months > 12) {
    clock_months = 1; clock_years++;
  }

  time -= clock_hours * 3600L;
  clock_minutes  = time / 60L;
  time -= clock_minutes * 60L;
  clock_seconds = time;
} /* update_time */
#endif // FEATURE_CLOCK


// --------------------------------------------------------------

#ifdef FEATURE_GPS
void service_gps(){

  long gps_lat, gps_lon;
  unsigned long fix_age;
  int gps_year;
  byte gps_month, gps_day, gps_hours, gps_minutes, gps_seconds, gps_hundredths;
  static byte gps_sync_pin_active = 0;
  #ifdef DEBUG_GPS
  char tempstring[10] = "";
  #endif //#ifdef DEBUG_GPS

  static unsigned long last_sync = 0;

  if (gps_data_available) {
    // retrieves +/- lat/long in 100000ths of a degree
    gps.get_position(&gps_lat, &gps_lon, &fix_age);
    gps.crack_datetime(&gps_year, &gps_month, &gps_day, &gps_hours, &gps_minutes, &gps_seconds, &gps_hundredths, &fix_age);
    #ifdef DEBUG_GPS
      #ifdef DEBUG_GPS_SERIAL
        debug_println("");
      #endif //DEBUG_GPS_SERIAL    
      debug_print("service_gps: fix_age:");
      debug_print_int(fix_age);
      debug_print(" lat:");
      debug_print_float(gps_lat,4);
      debug_print(" long:");
      debug_print_float(gps_lon,4);
      debug_print(" ");
      debug_print_int(gps_year);
      debug_print("-");
      debug_print_int(gps_month);
      debug_print("-");
      debug_print_int(gps_day);
      debug_print(" ");
      debug_print_int(gps_hours);
      debug_print(":");
      debug_print_int(gps_minutes);
      debug_println("");
    #endif // DEBUG_GPS

    if (fix_age < GPS_VALID_FIX_AGE_MS) {

      if (SYNC_TIME_WITH_GPS) {
        clock_year_set = gps_year;
        clock_month_set = gps_month;
        clock_day_set = gps_day;
        clock_hour_set = gps_hours;
        clock_min_set = gps_minutes;
        clock_sec_set = gps_seconds;
        millis_at_last_calibration = millis() - GPS_UPDATE_LATENCY_COMPENSATION_MS;
        update_time();
        #ifdef DEBUG_GPS
          #ifdef DEBUG_GPS_SERIAL
            debug_println("");
          #endif //DEBUG_GPS_SERIAL        
          debug_print("service_gps: clock sync:");
          sprintf(tempstring,"%s",clock_string());
          debug_print(tempstring);
          debug_println("");
        #endif // DEBUG_GPS
      }

      #if defined(OPTION_SYNC_RTC_TO_GPS) && defined(FEATURE_RTC_DS1307)
        static unsigned long last_rtc_gps_sync_time;
        if ((millis() - last_rtc_gps_sync_time) >= (SYNC_RTC_TO_GPS_SECONDS * 1000)) {
          rtc.adjust(DateTime(gps_year, gps_month, gps_day, gps_hours, gps_minutes, gps_seconds));
          #ifdef DEBUG_RTC
            debug_println("service_gps: synced RTC");
          #endif // DEBUG_RTC
          last_rtc_gps_sync_time = millis();
        }
      #endif // defined(OPTION_SYNC_RTC_TO_GPS) && defined(FEATURE_RTC_DS1307)

      #if defined(OPTION_SYNC_RTC_TO_GPS) && defined(FEATURE_RTC_PCF8583)
        static unsigned long last_rtc_gps_sync_time;
        if ((millis() - last_rtc_gps_sync_time) >= (SYNC_RTC_TO_GPS_SECONDS * 1000)) {
          rtc.year = gps_year;
          rtc.month = gps_month;
          rtc.day = gps_day;
          rtc.hour  = gps_hours;
          rtc.minute = gps_minutes;
          rtc.second = gps_seconds;
          rtc.set_time();
          #ifdef DEBUG_RTC
            debug_println("service_gps: synced RTC");
          #endif // DEBUG_RTC
          last_rtc_gps_sync_time = millis();
        }
      #endif // defined(OPTION_SYNC_RTC_TO_GPS) && defined(FEATURE_RTC_PCF8583)


      #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING) || defined(FEATURE_REMOTE_UNIT_SLAVE)
        if (SYNC_COORDINATES_WITH_GPS) {
          latitude = float(gps_lat) / 1000000.0;
          longitude = float(gps_lon) / 1000000.0;
          #ifdef DEBUG_GPS
            debug_print("service_gps: coord sync:");
            debug_print_float(latitude,2);
            debug_print(" ");
            debug_print_float(longitude,2);
            debug_println("");
          #endif // DEBUG_GPS
        }
      #endif // defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

      last_sync = millis();
    }

    gps_data_available = 0;
  }

  if ((millis() > (GPS_SYNC_PERIOD_SECONDS * 1000)) && ((millis() - last_sync) < (GPS_SYNC_PERIOD_SECONDS * 1000)) && (SYNC_TIME_WITH_GPS)) {
    clock_status = GPS_SYNC;
  } else {
    clock_status = FREE_RUNNING;
  }

  if (gps_sync){
    if (clock_status == GPS_SYNC){
      if (!gps_sync_pin_active){
        digitalWriteEnhanced(gps_sync,HIGH);
        gps_sync_pin_active = 1;  
      }
    } else {
      if (gps_sync_pin_active){
        digitalWriteEnhanced(gps_sync,LOW);
        gps_sync_pin_active = 0;  
      }
    }
  }


} /* service_gps */
#endif // FEATURE_GPS

// --------------------------------------------------------------

#if defined(OPTION_SYNC_MASTER_COORDINATES_TO_SLAVE) && (defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE))
void sync_master_coordinates_to_slave(){

  static unsigned long last_sync_master_coordinates_to_slave = 10000;

  if ((millis() - last_sync_master_coordinates_to_slave) >= (SYNC_MASTER_COORDINATES_TO_SLAVE_SECS * 1000)){
    if (submit_remote_command(REMOTE_UNIT_RC_COMMAND, 0, 0)) {
      #ifdef DEBUG_SYNC_MASTER_COORDINATES_TO_SLAVE
      debug_println("sync_master_coordinates_to_slave: submitted REMOTE_UNIT_RC_COMMAND");
      #endif //DEBUG_SYNC_MASTER_COORDINATES_TO_SLAVE
      last_sync_master_coordinates_to_slave = millis();  
    }  
  }


}
#endif //defined(OPTION_SYNC_MASTER_COORDINATES_TO_SLAVE) && (defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE))
//------------------------------------------------------

#if defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE) && (defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE))
void sync_master_clock_to_slave(){

  static unsigned long last_sync_master_clock_to_slave = 5000;

  if ((millis() - last_sync_master_clock_to_slave) >= (SYNC_MASTER_CLOCK_TO_SLAVE_CLOCK_SECS * 1000)){
    if (submit_remote_command(REMOTE_UNIT_CL_COMMAND, 0, 0)) {
      #ifdef DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE
      debug_println("sync_master_clock_to_slave: submitted REMOTE_UNIT_CL_COMMAND");
      #endif //DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE
      last_sync_master_clock_to_slave = millis();  
    }  
  }

  // if REMOTE_UNIT_CL_COMMAND above was successful, issue a GS (query GPS sync command) to get GPS sync status on the remote
  if (clock_synced_to_remote){
    if (submit_remote_command(REMOTE_UNIT_GS_COMMAND, 0, 0)) {
      #ifdef DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE
      debug_println("sync_master_clock_to_slave: submitted REMOTE_UNIT_GS_COMMAND");
      #endif //DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE
      clock_synced_to_remote = 0; 
    }      
  }

}
#endif //defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE)

// --------------------------------------------------------------

#ifdef FEATURE_CLOCK
char * clock_status_string(){

  switch (clock_status) {
    case FREE_RUNNING: return("FREE_RUNNING"); break;
    case GPS_SYNC: return("GPS_SYNC"); break;
    case RTC_SYNC: return("RTC_SYNC"); break;
    case SLAVE_SYNC: return("SLAVE_SYNC"); break;
    case SLAVE_SYNC_GPS: return("SLAVE_SYNC_GPS"); break;
  }
}
#endif //FEATURE_CLOCK
// --------------------------------------------------------------

#ifdef FEATURE_CLOCK
char * clock_string(){

  char return_string[32] = "";
  char temp_string[16] = "";



  dtostrf(clock_years, 0, 0, temp_string);
  strcpy(return_string, temp_string);
  strcat(return_string, "-");
  if (clock_months < 10) {
    strcat(return_string, "0");
  }
  dtostrf(clock_months, 0, 0, temp_string);
  strcat(return_string, temp_string);
  strcat(return_string, "-");
  if (clock_days < 10) {
    strcat(return_string, "0");
  }
  dtostrf(clock_days, 0, 0, temp_string);
  strcat(return_string, temp_string);
  strcat(return_string, " ");

  if (clock_hours < 10) {
    strcat(return_string, "0");
  }
  dtostrf(clock_hours, 0, 0, temp_string);
  strcat(return_string, temp_string);
  strcat(return_string, ":");
  if (clock_minutes < 10) {
    strcat(return_string, "0");
  }
  dtostrf(clock_minutes, 0, 0, temp_string);
  strcat(return_string, temp_string);
  strcat(return_string, ":");
  if (clock_seconds < 10) {
    strcat(return_string, "0");
  }
  dtostrf(clock_seconds, 0, 0, temp_string);
  strcat(return_string, temp_string);
  strcat(return_string,"Z");
  return return_string;

} /* clock_string */
#endif // FEATURE_CLOCK


// --------------------------------------------------------------

  #ifdef FEATURE_RTC
void service_rtc(){

  static unsigned long last_rtc_sync_time = 0;

  if (((millis() - last_rtc_sync_time) >= (SYNC_WITH_RTC_SECONDS * 1000)) || (clock_status == FREE_RUNNING)){
    last_rtc_sync_time = millis();
    #ifdef FEATURE_GPS
      if (clock_status == GPS_SYNC) { // if we're also equipped with GPS and we're synced to it, don't sync to realtime clock
        #ifdef DEBUG_RTC
          debug_println("service_rtc: synced to GPS already.  Exiting.");
        #endif // DEBUG_RTC
        return;
      }
    #endif // FEATURE_GPS


    #ifdef FEATURE_RTC_DS1307
      if (rtc.isrunning()) {
        DateTime now = rtc.now();
        #ifdef DEBUG_RTC
          debug_print("service_rtc: syncing: ");
          debug_print_int(now.year());
          debug_print("/");
          debug_print_int(now.month());
          debug_print("/");
          debug_print_int(now.day());
          debug_print(" ");
          debug_print_int(now.hour());
          debug_print(":");
          debug_print_int(now.minute());
          debug_print(":");
          debug_print_int(now.second());
          debug_println("");
        #endif // DEBUG_RTC
        clock_year_set = now.year();
        clock_month_set = now.month();
        clock_day_set = now.day();
        clock_hour_set = now.hour();
        clock_min_set = now.minute();
        clock_sec_set = now.second();
        millis_at_last_calibration = millis();
        update_time();
        clock_status = RTC_SYNC;
      } else {
        clock_status = FREE_RUNNING;
        #ifdef DEBUG_RTC
          debug_println("service_rtc: error: RTC not running");
        #endif // DEBUG_RTC
      }
    #endif //#FEATURE_RTC_DS1307



    #ifdef FEATURE_RTC_PCF8583
      rtc.get_time();
      if ((rtc.year > 2000) && (rtc.month > 0) && (rtc.month < 13)){  // do we have a halfway reasonable date?
        #ifdef DEBUG_RTC
          control_port->print("service_rtc: syncing: ");
          control_port->print(rtc.year, DEC);
          control_port->print('/');
          control_port->print(rtc.month, DEC);
          control_port->print('/');
          control_port->print(rtc.day, DEC);
          control_port->print(' ');
          control_port->print(rtc.hour, DEC);
          control_port->print(':');
          control_port->print(rtc.minute, DEC);
          control_port->print(':');
          control_port->println(rtc.second, DEC);
        #endif // DEBUG_RTC
        clock_year_set = rtc.year;
        clock_month_set = rtc.month;
        clock_day_set = rtc.day;
        clock_hour_set = rtc.hour;
        clock_min_set = rtc.minute;
        clock_sec_set = rtc.second;
        millis_at_last_calibration = millis();
        update_time();
        clock_status = RTC_SYNC;
      } else {
        clock_status = FREE_RUNNING;
        #ifdef DEBUG_RTC
          control_port->print("service_rtc: error: RTC not returning valid date or time: ");
          control_port->print(rtc.year, DEC);
          control_port->print('/');
          control_port->print(rtc.month, DEC);
          control_port->print('/');
          control_port->print(rtc.day, DEC);
          control_port->print(' ');
          control_port->print(rtc.hour, DEC);
          control_port->print(':');
          control_port->print(rtc.minute, DEC);
          control_port->print(':');
          control_port->println(rtc.second, DEC);
        #endif // DEBUG_RTC
      }
    #endif //#FEATURE_RTC_PCF8583



  }
} /* service_rtc */
#endif // FEATURE_RTC

// --------------------------------------------------------------

