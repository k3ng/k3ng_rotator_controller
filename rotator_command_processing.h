

byte process_backslash_command(byte input_buffer[], int input_buffer_index, byte source_port, char * return_string){

  strcpy(return_string,"");
  static unsigned long serial_led_time = 0;
  float tempfloat = 0;

  #ifdef UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS
    float heading = 0;
  #endif

  #if !defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) && !defined(FEATURE_AZ_POSITION_PULSE_INPUT)
    long place_multiplier = 0;
    byte decimalplace = 0;
  #endif

  #ifdef FEATURE_CLOCK
    int temp_year = 0;
    byte temp_month = 0;
    byte temp_day = 0;
    byte temp_minute = 0;
    byte temp_hour = 0;
  #endif // FEATURE_CLOCK

  #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
    char grid[10] = "";
    byte hit_error = 0;
  #endif // defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

  #if defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_PULSE_INPUT)
    int new_azimuth = 9999;
  #endif
  #ifdef FEATURE_ELEVATION_CONTROL
    #if defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_PULSE_INPUT)
      int new_elevation = 9999;
    #endif // FEATURE_ELEVATION_CONTROL
  #endif // defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_PULSE_INPUT)

  char temp_string[20] = "";

  switch (input_buffer[1]) {

   #if defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_PULSE_INPUT)
    case 'A':      // \Ax[x][x] - manually set azimuth
      new_azimuth = 9999;
      switch (input_buffer_index) {
        case 3:
          new_azimuth = (input_buffer[2] - 48);
          break;
        case 4:
          new_azimuth = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
          break;
        case 5:
          new_azimuth = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
          break;
      }
      if ((new_azimuth >= 0) && (new_azimuth < 360)) {
        azimuth = new_azimuth * HEADING_MULTIPLIER;
        configuration.last_azimuth = new_azimuth;
        raw_azimuth = new_azimuth * HEADING_MULTIPLIER;
        configuration_dirty = 1;
        strcpy(return_string, "Azimuth set to ");
        dtostrf(new_azimuth, 0, 0, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.  Format: \\Ax[x][x] ");
      }
      break;
        #else // defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_PULSE_INPUT)
    case 'A':      // \Ax[xxx][.][xxxx] - manually set azimuth
      place_multiplier = 1;
      for (int x = input_buffer_index - 1; x > 1; x--) {
        if (char(input_buffer[x]) != '.') {
          tempfloat += (input_buffer[x] - 48) * place_multiplier;
          place_multiplier = place_multiplier * 10;
        } else {
          decimalplace = x;
        }
      }
      if (decimalplace) {
        tempfloat = tempfloat / pow(10, (input_buffer_index - decimalplace - 1));
      }
      if ((tempfloat >= 0) && (tempfloat <= 360)) {
        configuration.azimuth_offset = 0;
        read_azimuth(1);
        configuration.azimuth_offset = tempfloat - float(raw_azimuth / HEADING_MULTIPLIER);
        configuration_dirty = 1;
        strcpy(return_string, "Azimuth calibrated to ");
        dtostrf(tempfloat, 0, 2, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.");
      }

      break;
      #endif // defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_PULSE_INPUT)


    #if defined(FEATURE_ELEVATION_CONTROL)
    #if defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_PULSE_INPUT)
    case 'B':      // \Bx[x][x] - manually set elevation
      new_elevation = 9999;
      switch (input_buffer_index) {
        case 3:
          new_elevation = (input_buffer[2] - 48);
          break;
        case 4:
          new_elevation = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
          break;
        case 5:
          new_elevation = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
          break;
      }
      if ((new_elevation >= 0) && (new_elevation <= 180)) {
        elevation = new_elevation * HEADING_MULTIPLIER;
        configuration.last_elevation = new_elevation;
        configuration_dirty = 1;
        strcpy(return_string, "Elevation set to ");
        dtostrf(new_elevation, 0, 0, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.  Format: \\Bx[x][x]");
      }
      break;
      #else // defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_PULSE_INPUT)
    case 'B':      // \Bx[xxx][.][xxxx] - manually set elevation
      place_multiplier = 1;
      for (int x = input_buffer_index - 1; x > 1; x--) {
        if (char(input_buffer[x]) != '.') {
          tempfloat += (input_buffer[x] - 48) * place_multiplier;
          place_multiplier = place_multiplier * 10;
        } else {
          decimalplace = x;
        }
      }
      if (decimalplace) {
        tempfloat = tempfloat / pow(10, (input_buffer_index - decimalplace - 1));
      }
      if ((tempfloat >= 0) && (tempfloat <= 180)) {
        configuration.elevation_offset = 0;
        read_elevation(1);
        configuration.elevation_offset = tempfloat - float(elevation / HEADING_MULTIPLIER);
        configuration_dirty = 1;
        strcpy(return_string, "Elevation calibrated to ");
        dtostrf(tempfloat, 0, 2, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.");
      }
      break;
      #endif // defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_PULSE_INPUT)
      #endif //FEATURE_ELEVATION_CONTROL

    #ifdef FEATURE_CLOCK
    case 'C':         // show clock
      update_time();
      sprintf(return_string, "%s", clock_string());


      break;
    case 'O':         // set clock
      temp_year = ((input_buffer[2] - 48) * 1000) + ((input_buffer[3] - 48) * 100) + ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
      temp_month = ((input_buffer[6] - 48) * 10) + (input_buffer[7] - 48);
      temp_day = ((input_buffer[8] - 48) * 10) + (input_buffer[9] - 48);
      temp_hour = ((input_buffer[10] - 48) * 10) + (input_buffer[11] - 48);
      temp_minute = ((input_buffer[12] - 48) * 10) + (input_buffer[13] - 48);
      if ((temp_year > 2013) && (temp_year < 2070) &&
          (temp_month > 0) && (temp_month < 13) &&
          (temp_day > 0) && (temp_day < 32) &&
          (temp_hour >= 0) && (temp_hour < 24) &&
          (temp_minute >= 0) && (temp_minute < 60) &&
          (input_buffer_index == 14)) {

        clock_year_set = temp_year;
        clock_month_set = temp_month;
        clock_day_set = temp_day;
        clock_hour_set = temp_hour;
        clock_min_set = temp_minute;
        clock_sec_set = 0;
        millis_at_last_calibration = millis();





        #if defined(FEATURE_RTC_DS1307)
        rtc.adjust(DateTime(temp_year, temp_month, temp_day, temp_hour, temp_minute, 0));
        #endif // defined(FEATURE_RTC_DS1307)
        #if defined(FEATURE_RTC_PCF8583)
        rtc.year = temp_year;
        rtc.month = temp_month;
        rtc.day = temp_day;
        rtc.hour  = temp_hour;
        rtc.minute = temp_minute;
        rtc.second = 0;
        rtc.set_time();
        #endif // defined(FEATURE_RTC_PCF8583)

        #if (!defined(FEATURE_RTC_DS1307) && !defined(FEATURE_RTC_PCF8583))
        strcpy(return_string, "Clock set to ");
        update_time();
        strcat(return_string, clock_string());
        #else
        strcpy(return_string, "Internal clock and RTC set to ");
        update_time();
        strcat(return_string, clock_string());
        #endif


      } else {
        strcpy(return_string, "Error. Usage: \\OYYYYMMDDHHmm");
      }
      break;
          #endif // FEATURE_CLOCK


    case 'D': 
      if (debug_mode & source_port) {
        debug_mode = debug_mode & (~source_port);
      } else {
        debug_mode = debug_mode | source_port;
      } 
      break;                                              // D - Debug

    case 'E':                                                                      // E - Initialize eeprom
      initialize_eeprom_with_defaults();
      strcpy(return_string, "Initialized eeprom, resetting unit in 5 seconds...");
      reset_the_unit = 1;
      break;

    case 'L':                                                                      // L - rotate to long path
      if (azimuth < (180 * HEADING_MULTIPLIER)) {
        submit_request(AZ, REQUEST_AZIMUTH, (azimuth + (180 * HEADING_MULTIPLIER)), 15);
      } else {
        submit_request(AZ, REQUEST_AZIMUTH, (azimuth - (180 * HEADING_MULTIPLIER)), 16);
      }
      break;

 #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
    case 'G':   // G - set coordinates using grid square
      if (isalpha(input_buffer[2])) {
        grid[0] = input_buffer[2];
      } else { hit_error = 1; }
      if (isalpha(input_buffer[3])) {
        grid[1] = input_buffer[3];
      } else { hit_error = 1; }
      if (isdigit(input_buffer[4])) {
        grid[2] = input_buffer[4];
      } else { hit_error = 1; }
      if (isdigit(input_buffer[5])) {
        grid[3] = input_buffer[5];
      } else { hit_error = 1; }
      if (isalpha(input_buffer[6])) {
        grid[4] = input_buffer[6];
      } else { hit_error = 1; }
      if (isalpha(input_buffer[7])) {
        grid[5] = input_buffer[7];
      } else { hit_error = 1; }
      if ((input_buffer_index != 8) || (hit_error)) {
        strcpy(return_string, "Error.  Usage \\Gxxxxxx");
      } else {
        grid2deg(grid, &longitude, &latitude);
        strcpy(return_string, "Coordinates set to: ");
        dtostrf(latitude, 0, 4, temp_string);
        strcat(return_string, temp_string);
        strcat(return_string, " ");
        dtostrf(longitude, 0, 4, temp_string);
        strcat(return_string, temp_string);
      }
      break;
 #endif // defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

  #ifdef FEATURE_MOON_TRACKING
    case 'M':
      switch (input_buffer[2]) {
        case '0':
          submit_request(AZ, REQUEST_STOP, 0, 17);
          submit_request(EL, REQUEST_STOP, 0, 18);
          strcpy(return_string, "Moon tracking deactivated.");
          break;
        case '1':
          moon_tracking_active = 1;
          #ifdef FEATURE_SUN_TRACKING
            sun_tracking_active = 0;
          #endif // FEATURE_SUN_TRACKING
          strcpy(return_string, "Moon tracking activated.");
          break;
        default: strcpy(return_string, "Error."); break;
      }
      break;
  #endif // FEATURE_MOON_TRACKING

  #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
    case 'R':
      strcpy(return_string, "Remote port rx sniff o");
      if (remote_port_rx_sniff) {
        remote_port_rx_sniff = 0;
        strcat(return_string, "ff");
      } else {
        remote_port_rx_sniff = 1;
        strcat(return_string, "n");
      }
      break;
    case 'S':
      #if defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
      ethernetslavelinkclient0.print(ETHERNET_PREAMBLE);
      #endif    
      for (int x = 2; x < input_buffer_index; x++) {
        #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
        remote_unit_port->write(input_buffer[x]);
        #endif
        #if defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
        ethernetslavelinkclient0.write(input_buffer[x]);
        #endif
        if (remote_port_tx_sniff) {
          control_port->write(input_buffer[x]);
        }
      }
      #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
      remote_unit_port->write(13);
      #endif
      #if defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
      ethernetslavelinkclient0.write(13);
      #endif      
      if (remote_port_tx_sniff) {
        control_port->write(13);
      }
      break;
    case 'T':
      strcpy(return_string, "Remote port tx sniff o");
      if (remote_port_tx_sniff) {
        remote_port_tx_sniff = 0;
        strcat(return_string, "ff");
      } else {
        remote_port_tx_sniff = 1;
        strcat(return_string, "n");
      }
      break;
    case 'Z':
      strcpy(return_string, "Suspend auto remote commands o");
      if (suspend_remote_commands) {
        suspend_remote_commands = 0;
        strcat(return_string, "ff");
      } else {
        suspend_remote_commands = 1;
        strcat(return_string, "n");
      }
      break;
  #endif // defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) || defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)

  #ifdef FEATURE_SUN_TRACKING
    case 'U':     // activate / deactivate sun tracking
      switch (input_buffer[2]) {
        case '0':
          submit_request(AZ, REQUEST_STOP, 0, 19);
          submit_request(EL, REQUEST_STOP, 0, 20);
          strcpy(return_string, "Sun tracking deactivated.");
          break;
        case '1':
          sun_tracking_active = 1;
          strcpy(return_string, "Sun tracking activated.");
          #ifdef FEATURE_MOON_TRACKING
          moon_tracking_active = 0;
          #endif // FEATURE_MOON_TRACKING
          break;
        default: strcpy(return_string, "Error."); break;
      }
      break;

  #endif // FEATURE_SUN_TRACKING

  #if defined(FEATURE_SUN_TRACKING) || defined(FEATURE_MOON_TRACKING)
    case 'X':
      switch (toupper(input_buffer[2])) {
        #if defined(FEATURE_SUN_TRACKING)
        case 'S': 
          update_sun_position();
          if (calibrate_az_el(sun_azimuth, sun_elevation)) {
            strcpy(return_string, az_el_calibrated_string());
          } else {
            strcpy(return_string, "Error.");
          }
          break;
        #endif // FEATURE_SUN_TRACKING
        #if defined(FEATURE_MOON_TRACKING)
        case 'M':
          update_moon_position();
          if (calibrate_az_el(moon_azimuth, moon_elevation)) {
            strcpy(return_string, az_el_calibrated_string());
          } else {
            strcpy(return_string, "Error.");
          }
          break;
        #endif // FEATURE_MOON_TRACKING
        case '0':
          configuration.azimuth_offset = 0;
          configuration.elevation_offset = 0;
          configuration_dirty = 1;
          break;
        default: strcpy(return_string, "?>"); break;


      } /* switch */
      break;
      #endif // defined(FEATURE_SUN_TRACKING) || defined(FEATURE_MOON_TRACKING)

    #ifdef FEATURE_PARK
    case 'P':    // Park
      strcpy(return_string, "Parking...");
      initiate_park();
      park_serial_initiated = 1;
      break;
      #endif // FEATURE_PARK

    #ifdef FEATURE_ANCILLARY_PIN_CONTROL
    case 'N':      // \Nxx - turn pin on; xx = pin number
      if ((((input_buffer[2] > 47) && (input_buffer[2] < 58)) || (toupper(input_buffer[2]) == 'A')) && (input_buffer[3] > 47) && (input_buffer[3] < 58) && (input_buffer_index == 4)) {
        byte pin_value = 0;
        if (toupper(input_buffer[2]) == 'A') {
          pin_value = get_analog_pin(input_buffer[3] - 48);
        } else {
          pin_value = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
        }
        pinModeEnhanced(pin_value, OUTPUT);
        digitalWriteEnhanced(pin_value, HIGH);
        strcpy(return_string, "OK");
      } else {
        strcpy(return_string, "Error");
      }
      break;
    case 'F':      // \Fxx - turn pin off; xx = pin number
      if ((((input_buffer[2] > 47) && (input_buffer[2] < 58)) || (toupper(input_buffer[2]) == 'A')) && (input_buffer[3] > 47) && (input_buffer[3] < 58) && (input_buffer_index == 4)) {
        byte pin_value = 0;
        if (toupper(input_buffer[2]) == 'A') {
          pin_value = get_analog_pin(input_buffer[3] - 48);
        } else {
          pin_value = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
        }
        pinModeEnhanced(pin_value, OUTPUT);
        digitalWriteEnhanced(pin_value, LOW);
        strcpy(return_string, "OK");
      } else {
        strcpy(return_string, "Error");
      }
      break;
    case 'W':    // \Wxxyyy - turn on pin PWM; xx = pin number, yyy = PWM value (0-255)
      if (((input_buffer[2] > 47) && (input_buffer[2] < 58)) && (input_buffer[3] > 47) && (input_buffer[3] < 58)  && (input_buffer_index == 7)) {
        byte pin_value = 0;
        if (toupper(input_buffer[2]) == 'A') {
          pin_value = get_analog_pin(input_buffer[3] - 48);
        } else {
          pin_value = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
        }
        int write_value = ((input_buffer[4] - 48) * 100) + ((input_buffer[5] - 48) * 10) + (input_buffer[6] - 48);
        if ((write_value >= 0) && (write_value < 256)) {
          pinModeEnhanced(pin_value, OUTPUT);
          analogWriteEnhanced(pin_value, write_value);
          strcpy(return_string, "OK");
        } else {
          strcpy(return_string, "Error");
        }
      } else {
        strcpy(return_string, "Error");
      }
      break;
  #endif // FEATURE_ANCILLARY_PIN_CONTROL



// zzzzzzz

// TODO : one big status query command (get rid of all these little commands)      

  #ifdef UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS

    case '?':
      strcpy(return_string, "\\!??");  //  \\??xxyy - failed response back
      if (input_buffer_index == 4){
        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'Z')) {  // \\AZ - query AZ
          strcpy(return_string, "\\!OKAZ");
          if ((raw_azimuth/HEADING_MULTIPLIER) < 100) {
            strcat(return_string,"0");
            }
            if ((raw_azimuth/HEADING_MULTIPLIER) < 10) {
              strcat(return_string,"0");
            }
            dtostrf(float(raw_azimuth/(float)HEADING_MULTIPLIER),0,6,temp_string);
            strcat(return_string,temp_string); 
          }
          if ((input_buffer[2] == 'E') && (input_buffer[3] == 'L')) {  // \\EL - query EL
            #ifdef FEATURE_ELEVATION_CONTROL
              strcpy(return_string, "\\!OKEL");
              if ((elevation/HEADING_MULTIPLIER) >= 0) {
                strcat(return_string,"+");
              } else {
                strcat(return_string,"-");
              }
              if (abs(elevation/HEADING_MULTIPLIER) < 100) {
                strcat(return_string,"0");
              }
              if (abs(elevation/HEADING_MULTIPLIER) < 10) {
                strcat(return_string,"0");
              }
              dtostrf(float(abs(elevation/(float)HEADING_MULTIPLIER)),0,6,temp_string);
              strcat(return_string,temp_string); 
            #else // FEATURE_ELEVATION_CONTROL
              strcpy(return_string, "\\!??EL");
            #endif //FEATURE_ELEVATION_CONTROL 
          }
          if ((input_buffer[2] == 'A') && (input_buffer[3] == 'S')) {  // \\AS - AZ status
            strcpy(return_string, "\\!OKAS");
            dtostrf(az_state, 0, 0, temp_string);
            strcat(return_string, temp_string); 
          }  
          if ((input_buffer[2] == 'E') && (input_buffer[3] == 'S')) {  // \\ES - EL Status
            #ifdef FEATURE_ELEVATION_CONTROL
              strcpy(return_string, "\\!OKES");
              dtostrf(el_state, 0, 0, temp_string);
              strcat(return_string, temp_string);
            #else // FEATURE_ELEVATION_CONTROL  
              strcpy(return_string, "\\!??ES");
            #endif //FEATURE_ELEVATION_CONTROL              
          }   
          if ((input_buffer[2] == 'P') && (input_buffer[3] == 'G')) {  // \\PG - Ping
            strcpy(return_string, "\\!OKPG");     
          }      
          if ((input_buffer[2] == 'R') && (input_buffer[3] == 'L')) {  // \\RL - rotate left
            submit_request(AZ, REQUEST_CCW, 0, 121);
            strcpy(return_string, "\\!OKRL");
          }     
          if ((input_buffer[2] == 'R') && (input_buffer[3] == 'R')) {  // \\RR - rotate right
            submit_request(AZ, REQUEST_CW, 0, 122);
            strcpy(return_string, "\\!OKRL");
          }   
          if ((input_buffer[2] == 'R') && (input_buffer[3] == 'U')) {  //  \\RU - elevate up
            submit_request(EL, REQUEST_UP, 0, 129);
            strcpy(return_string, "\\!OKRU");
          } 
          if ((input_buffer[2] == 'R') && (input_buffer[3] == 'D')) {  // \\RD - elevate down
            submit_request(EL, REQUEST_DOWN, 0, 130);
            strcpy(return_string, "\\!OKRD");
          }  
          #ifdef FEATURE_GPS
              if ((input_buffer[2] == 'R') && (input_buffer[3] == 'C')) {  // \\RC - Read coordinates
                strcpy(return_string,"\\!OKRC");
                if (latitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
                dtostrf(abs(latitude),0,4,temp_string);
                strcat(return_string,temp_string);         
                strcat(return_string," ");
                if (longitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
                if (longitude < 100){strcat(return_string,"0");}
                dtostrf(abs(longitude),0,4,temp_string);
                strcat(return_string,temp_string); 
              }
          #endif //FEATURE_GPS
          #ifdef FEATURE_CLOCK
              if ((input_buffer[2] == 'G') && (input_buffer[3] == 'S')) { // \\GS - query GPS sync
                strcpy(return_string,"\\!OKGS");
                if (clock_status == GPS_SYNC){                
                  strcat(return_string,"1");
                } else {
                  strcat(return_string,"0");
                }        
              }
          #endif //FEATURE_CLOCK 

          if ((input_buffer[2] == 'S') && (input_buffer[3] == 'A')) {  // \\SA - stop azimuth rotation
            submit_request(AZ, REQUEST_STOP, 0, 124);
            strcpy(return_string,"\\!OKSA");
          }   
          if ((input_buffer[2] == 'S') && (input_buffer[3] == 'E')) {  // \\SE - stop elevation rotation
            #ifdef FEATURE_ELEVATION_CONTROL
              submit_request(EL, REQUEST_STOP, 0, 125);
            #endif
            strcpy(return_string,"\\!OKSE");
          } 
          if ((input_buffer[2] == 'S') && (input_buffer[3] == 'S')) {  // \\SS - stop all rotation
            submit_request(AZ, REQUEST_STOP, 0, 124);
            #ifdef FEATURE_ELEVATION_CONTROL
              submit_request(EL, REQUEST_STOP, 0, 125);
            #endif
            strcpy(return_string,"\\!OKSS");
          }   

          if ((input_buffer[2] == 'C') && (input_buffer[3] == 'L')) {  // \\CL - read the clock
            #ifdef FEATURE_CLOCK
              strcpy(return_string,"\\!OKCL");
              update_time();
              strcat(return_string,clock_string());
            #else //FEATURE_CLOCK
              strcpy(return_string,"\\!??CL");
            #endif //FEATURE_CLOCK
          }

          if ((input_buffer[2] == 'R') && (input_buffer[3] == 'B')) {  // \\RB - reboot
            wdt_enable(WDTO_30MS); while (1) {}  //ZZZZZZ - TODO - change to reboot flag
          }


        } //if (input_buffer_index == 4)

    if (input_buffer_index == 6){
      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'O')) {  // \\DOxx - digital pin initialize as output; xx = pin # (01, 02, A0,etc.)
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[4] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          strcpy(return_string,"\\!OKDO");
          pinModeEnhanced(pin_value, OUTPUT);
        }
      }

      if ((input_buffer[2] == 'D') && ((input_buffer[3] == 'H') || (input_buffer[3] == 'L'))) { // \\DLxx - digital pin write low; xx = pin #   \\DHxx - digital pin write high; xx = pin # 
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          if (input_buffer[3] == 'H') {
            digitalWriteEnhanced(pin_value, HIGH);
            strcpy(return_string,"\\!OKDH");
          } else {
            digitalWriteEnhanced(pin_value, LOW);
            strcpy(return_string,"\\!OKDL");
          }
        }
      }





/*

Not implemented yet:

\\SWxy - serial write byte; x = serial port # (0, 1, 2, 3), y = byte to write
\\SDx - deactivate serial read event; x = port #
\\SSxyyyyyy... - serial write string; x = port #, yyyy = string of characters to send (variable length)
\\SAx - activate serial read event; x = port #

*/

     if ((input_buffer[2] == 'D') && (input_buffer[3] == 'I')) {  // \\DIxx - digital pin initialize as input; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          strcpy(return_string,"\\!OKDI");
        }
      }

      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'P')) {  // \\DPxx - digital pin initialize as input with pullup; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          digitalWriteEnhanced(pin_value, HIGH);
          strcpy(return_string,"\\!OKDP");
        }
      }

      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'R')) {  // \\DRxx - digital pin read; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          byte pin_read = digitalReadEnhanced(pin_value);
          strcpy(return_string,"\\!OKDR");
          dtostrf((input_buffer[4]-48),0,0,temp_string);
          strcat(return_string,temp_string);              
          dtostrf((input_buffer[5]-48),0,0,temp_string);
          strcat(return_string,temp_string);  
          if (pin_read) {
            strcat(return_string,"1");
          } else {
            strcat(return_string,"0");
          }
        }
      }
      if ((input_buffer[2] == 'A') && (input_buffer[3] == 'R')) {  //  \\ARxx - analog pin read; xx = pin #
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          int pin_read = analogReadEnhanced(pin_value);
          strcpy(return_string,"\\!OKAR");
          if (toupper(input_buffer[4]) == 'A') {
            strcat(return_string,"A");
          } else {
            dtostrf((input_buffer[4]-48),0,0,temp_string);
            strcat(return_string,temp_string);
          }
                        
          dtostrf((input_buffer[5]-48),0,0,temp_string);
          strcat(return_string,temp_string);  
          if (pin_read < 1000) {
            strcat(return_string,"0");
          }
          if (pin_read < 100) {
            strcat(return_string,"0");
          }
          if (pin_read < 10) {
            strcat(return_string,"0");
          }
          dtostrf(pin_read,0,0,temp_string);
          strcat(return_string,temp_string);             
        }
      }

      if ((input_buffer[2] == 'N') && (input_buffer[3] == 'T')) { // \\NTxx - no tone; xx = pin #
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          noTone(pin_value);
          strcpy(return_string,"\\!OKNT");
        }
    }  //if ((input_buffer_index == 6)




    if (input_buffer_index == 9) {

      if ((input_buffer[2] == 'G') && (input_buffer[3] == 'A')) {  // \\GAxxx.x - go to AZ xxx.x
        heading = ((input_buffer[4] - 48) * 100.) + ((input_buffer[5] - 48) * 10.) + (input_buffer[6] - 48.) + ((input_buffer[8] - 48) / 10.);     
        if (((heading >= 0) && (heading < 451))  && (input_buffer[7] == '.')) {
          submit_request(AZ, REQUEST_AZIMUTH, (heading * HEADING_MULTIPLIER), 136);
          strcpy(return_string,"\\!OKGA");
        } else {
          strcpy(return_string,"\\!??GA");
        }
      }  
      if ((input_buffer[2] == 'G') && (input_buffer[3] == 'E')) {  // \\GExxx.x - go to EL
        #ifdef FEATURE_ELEVATION_CONTROL
          heading = ((input_buffer[4] - 48) * 100.) + ((input_buffer[5] - 48) * 10.) + (input_buffer[5] - 48) + ((input_buffer[8] - 48) / 10.);
          if (((heading >= 0) && (heading < 181)) && (input_buffer[7] == '.')) {
            submit_request(EL, REQUEST_ELEVATION, (heading * HEADING_MULTIPLIER), 37);
            strcpy(return_string,"\\!OKGE");
          } else {
            strcpy(return_string,"\\!??GE");
          }
        #else 
          strcpy(return_string,"\\!OKGE");  
        #endif // #FEATURE_ELEVATION_CONTROL  
      } 


      if ((input_buffer[2] == 'A') && (input_buffer[3] == 'W')) {  // \\AWxxyyy - analog pin write; xx = pin #, yyy = value to write (0 - 255)
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          int write_value = ((input_buffer[6] - 48) * 100) + ((input_buffer[7] - 48) * 10) + (input_buffer[8] - 48);
          if ((write_value >= 0) && (write_value < 256)) {
            analogWriteEnhanced(pin_value, write_value);
            strcpy(return_string,"\\!OKAW");
          }
        }
      }
    } //if (input_buffer_index == 9)

    if (input_buffer_index == 10) {
      if ((input_buffer[2] == 'D') && (input_buffer[3] == 'T')) { // \\DTxxyyyy - digital pin tone output; xx = pin #, yyyy = frequency
        if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          int write_value = ((input_buffer[6] - 48) * 1000) + ((input_buffer[7] - 48) * 100) + ((input_buffer[8] - 48) * 10) + (input_buffer[9] - 48);
          if ((write_value >= 0) && (write_value <= 9999)) {
            tone(pin_value, write_value);
            strcpy(return_string,"\\!OKDT");

          }
        }
      }
    }  //if (input_buffer_index == 10)


      break; //case '\\'





  #endif  //UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS


    default: strcpy(return_string, "Error.");



  } /* switch */
} /* process_backslash_command */

//-----------------------------------------------------------------------

#ifdef FEATURE_EASYCOM_EMULATION
void process_easycom_command(byte * easycom_command_buffer, int easycom_command_buffer_index, byte source_port, char * return_string){


  /* Easycom protocol implementation
   *
   * Implemented commands:
   *
   * Command      Meaning     Parameters
   * -------      -------     ----------
   *
   * ML           Move Left
   * MR           Move Right
   * MU           Move Up
   * MD           Move Down
   * SA           Stop azimuth moving
   * SE           Stop elevation moving
   *
   * VE           Request Version
   * AZ           Azimuth     number - 1 decimal place (activated with OPTION_EASYCOM_AZ_QUERY_COMMAND)
   * EL           Elevation   number - 1 decimal place (activated with OPTION_EASYCOM_EL_QUERY_COMMAND)
   *
   *
   */



  char tempstring[11] = "";
  float heading = -1;
  strcpy(return_string,"");

  switch (easycom_command_buffer[0]) { // look at the first character of the command
    #if defined(OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK) && defined(FEATURE_ELEVATION_CONTROL)  
    case 'Z':
      //strcpy(return_string,"+");
      strcpy(return_string,"AZ");
      dtostrf((float)azimuth/(float)HEADING_MULTIPLIER,0,1,tempstring);
      strcat(return_string,tempstring);
      //if (elevation >= 0){
        //strcat(return_string,"+");
        strcat(return_string," EL");
      //}
      dtostrf((float)elevation/(float)HEADING_MULTIPLIER,0,1,tempstring);      
      strcat(return_string,tempstring);
      break;
    #endif //OPTION_HAMLIB_EASYCOM_AZ_EL_COMMAND_HACK
    case 'A':  // AZ
      if (easycom_command_buffer[1] == 'Z') {  // format is AZx.x or AZxx.x or AZxxx.x (why didn't they make it fixed length?)
        switch (easycom_command_buffer_index) {
          #ifdef OPTION_EASYCOM_AZ_QUERY_COMMAND
          case 2:
            //strcpy(return_string,"AZ");
            strcpy(return_string,"+");
            dtostrf((float)azimuth/(float)HEADING_MULTIPLIER,0,1,tempstring);
            strcat(return_string,tempstring);
            return;
            break;
          #endif // OPTION_EASYCOM_AZ_QUERY_COMMAND
          case 5: // format AZx.x
            heading = (easycom_command_buffer[2] - 48) + ((easycom_command_buffer[4] - 48) / 10.);
            break;
          case 6: // format AZxx.x
            heading = ((easycom_command_buffer[2] - 48) * 10.) + (easycom_command_buffer[3] - 48) + ((easycom_command_buffer[5] - 48) / 10.);
            break;
          case 7: // format AZxxx.x
            heading = ((easycom_command_buffer[2] - 48) * 100.) + ((easycom_command_buffer[3] - 48) * 10.) + (easycom_command_buffer[4] - 48.) + ((easycom_command_buffer[6] - 48) / 10.);
            break;
            // default: control_port->println("?"); break;
        }
        if (((heading >= 0) && (heading < 451))  && (easycom_command_buffer[easycom_command_buffer_index - 2] == '.')) {
          submit_request(AZ, REQUEST_AZIMUTH, (heading * HEADING_MULTIPLIER), 36);
        } else {
          strcpy(return_string,"?");
        }
      } else {
        strcpy(return_string,"?");
      }
      break;
      #ifdef FEATURE_ELEVATION_CONTROL
    case 'E':  // EL
      if (easycom_command_buffer[1] == 'L') {
        switch (easycom_command_buffer_index) {
          #ifdef OPTION_EASYCOM_EL_QUERY_COMMAND
          case 2:
            //strcpy(return_string,"EL");
            if (elevation >= 0){
              strcpy(return_string,"+");
            }
            dtostrf((float)elevation/(float)HEADING_MULTIPLIER,0,1,tempstring);
            strcat(return_string,tempstring);            
            return;
            break;
          #endif // OPTION_EASYCOM_EL_QUERY_COMMAND
          case 5: // format ELx.x
            heading = (easycom_command_buffer[2] - 48) + ((easycom_command_buffer[4] - 48) / 10.);
            break;
          case 6: // format ELxx.x
            heading = ((easycom_command_buffer[2] - 48) * 10.) + (easycom_command_buffer[3] - 48) + ((easycom_command_buffer[5] - 48) / 10.);
            break;
          case 7: // format ELxxx.x
            heading = ((easycom_command_buffer[2] - 48) * 100.) + ((easycom_command_buffer[3] - 48) * 10.) + (easycom_command_buffer[4] - 48) + ((easycom_command_buffer[6] - 48) / 10.);
            break;
            // default: control_port->println("?"); break;
        }
        if (((heading >= 0) && (heading < 181)) && (easycom_command_buffer[easycom_command_buffer_index - 2] == '.')) {
          submit_request(EL, REQUEST_ELEVATION, (heading * HEADING_MULTIPLIER), 37);
        } else {
          strcpy(return_string,"?");
        }
      } else {
        strcpy(return_string,"?");
      }
      break;
      #endif // #FEATURE_ELEVATION_CONTROL
    case 'S':  // SA or SE - stop azimuth, stop elevation
      switch (easycom_command_buffer[1]) {
        case 'A':
          submit_request(AZ, REQUEST_STOP, 0, 38);
          break;
        #ifdef FEATURE_ELEVATION_CONTROL
        case 'E':
          submit_request(EL, REQUEST_STOP, 0, 39);
          break;
        #endif // FEATURE_ELEVATION_CONTROL
        default: strcpy(return_string,"?"); break;
      }
      break;
    case 'M':  // ML, MR, MU, MD - move left, right, up, down
      switch (easycom_command_buffer[1]) {
        case 'L': // ML - move left
          submit_request(AZ, REQUEST_CCW, 0, 40);
          break;
        case 'R': // MR - move right
          submit_request(AZ, REQUEST_CW, 0, 41);
          break;
        #ifdef FEATURE_ELEVATION_CONTROL
        case 'U': // MU - move up
          submit_request(EL, REQUEST_UP, 0, 42);
          break;
        case 'D': // MD - move down
          submit_request(EL, REQUEST_DOWN, 0, 43);
          break;
        #endif // FEATURE_ELEVATION_CONTROL
        default: strcpy(return_string,"?"); break;
      }
      break;
    case 'V': // VE - version query
      if (easycom_command_buffer[1] == 'E') {
        strcpy(return_string,"VE002");
      }                                                                       // not sure what to send back, sending 002 because this is easycom version 2?
      break;
    default: strcpy(return_string,"?"); break;
  } /* switch */



} /* easycom_serial_commmand */
#endif // FEATURE_EASYCOM_EMULATION





// --------------------------------------------------------------

    
#ifdef FEATURE_REMOTE_UNIT_SLAVE
void process_remote_slave_command(byte * slave_command_buffer, int slave_command_buffer_index, byte source_port, char * return_string){


/*
 *
 * This implements a protocol for host unit to remote unit communications
 *
 *
 * Remote Slave Unit Protocol Reference
 *
 *  PG - ping
 *  AZ - read azimuth  (returns AZxxx.xxxxxx)
 *  EL - read elevation (returns ELxxx.xxxxxx)
 *  RC - read coordinates (returns RC+xx.xxxx -xxx.xxxx)
 *  GS - query GPS status (returns GS0 (no sync) or GS1 (sync))
 *  DOxx - digital pin initialize as output;
 *  DIxx - digital pin initialize as input
 *  DPxx - digital pin initialize as input with pullup
 *  DRxx - digital pin read
 *  DLxx - digital pin write low
 *  DHxx - digital pin write high
 *  DTxxyyyy - digital pin tone output
 *  NTxx - no tone
 *  ARxx - analog pin read
 *  AWxxyyy - analog pin write
 *  SWxy - serial write byte
 *  SDx - deactivate serial read event; x = port #
 *  SSxyyyyyy... - serial write string; x = port #, yyyy = string of characters to send
 *  SAx - activate serial read event; x = port #
 *  RB - reboot
 *  CL - return clock date and time
 *
 * Responses
 *
 *  ER - report an error (remote to host only)
 *  EV - report an event (remote to host only)
 *  OK - report success (remote to host only)
 *  CS - report a cold start (remote to host only)
 *
 * Error Codes
 *
 *  ER01 - Serial port buffer timeout
 *  ER02 - Command syntax error
 *
 * Events
 *
 *  EVSxy - Serial port read event; x = serial port number, y = byte returned
 *
 *
 */



  byte command_good = 0;
  strcpy(return_string,"");
  char tempstring[25] = "";

  if (slave_command_buffer_index < 2) {
    strcpy(return_string,"ER02");  // we don't have enough characters - syntax error
  } else {

    #ifdef DEBUG_PROCESS_SLAVE
    debug_print("serial_serial_buffer: command_string: ");
    debug_print((char*)slave_command_buffer);
    debug_print("$ slave_command_buffer_index: ");
    debug_print_int(slave_command_buffer_index);
    debug_print("\n");
    #endif // DEBUG_PROCESS_SLAVE

    if (((slave_command_buffer[0] == 'S') && (slave_command_buffer[1] == 'S')) && (slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 53)) { // this is a variable length command
      command_good = 1;
      for (byte x = 3; x < slave_command_buffer_index; x++) {
        switch (slave_command_buffer[2] - 48) {
          case 0: control_port->write(slave_command_buffer[x]); break;
          #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
          case 1: remote_unit_port->write(slave_command_buffer[x]); break;
          #endif
        }
      }
    }

    if (slave_command_buffer_index == 2) {

      #ifdef FEATURE_CLOCK
      if ((slave_command_buffer[0] == 'C') && (slave_command_buffer[1] == 'L')) {
        strcpy(return_string,"CL");
        update_time();
        strcat(return_string,clock_string());
        command_good = 1;
      }
      #endif //FEATURE_CLOCK


      #ifdef FEATURE_GPS
      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'C')) {                    // RC - read coordinates
        strcpy(return_string,"RC");
        if (latitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
        dtostrf(abs(latitude),0,4,tempstring);
        strcat(return_string,tempstring);         
        strcat(return_string," ");
        if (longitude < 0){strcat(return_string,"-");} else {strcat(return_string,"+");}
        if (longitude < 100){strcat(return_string,"0");}
        dtostrf(abs(longitude),0,4,tempstring);
        strcat(return_string,tempstring);        
        command_good = 1;
      }   
      #ifdef FEATURE_CLOCK
      if ((slave_command_buffer[0] == 'G') && (slave_command_buffer[1] == 'S')) {                    // GS - query GPS sync
        strcpy(return_string,"GS");
        if (clock_status == GPS_SYNC){                
          strcat(return_string,"1");
        } else {
          strcat(return_string,"0");
        }        
        command_good = 1;
      }
      #endif //FEATURE_CLOCK                 
      #endif //FEATURE_GPS      

      if ((slave_command_buffer[0] == 'P') && (slave_command_buffer[1] == 'G')) {
        strcpy(return_string,"PG"); command_good = 1;
      }                                                                        // PG - ping
      if ((slave_command_buffer[0] == 'R') && (slave_command_buffer[1] == 'B')) {
        wdt_enable(WDTO_30MS); while (1) {
        }
      }                                                                        // RB - reboot
      if ((slave_command_buffer[0] == 'A') && (slave_command_buffer[1] == 'Z')) {
        strcpy(return_string,"AZ");
        //if ((raw_azimuth/HEADING_MULTIPLIER) < 1000) {
        //  strcat(return_string,"0");
        //}
        if ((raw_azimuth/HEADING_MULTIPLIER) < 100) {
          strcat(return_string,"0");
        }
        if ((raw_azimuth/HEADING_MULTIPLIER) < 10) {
          strcat(return_string,"0");
        }
        dtostrf(float(raw_azimuth/(float)HEADING_MULTIPLIER),0,6,tempstring);
        strcat(return_string,tempstring);          
        command_good = 1;
      }
      #ifdef FEATURE_ELEVATION_CONTROL
      if ((slave_command_buffer[0] == 'E') && (slave_command_buffer[1] == 'L')) {
        strcpy(return_string,"EL");
        if ((elevation/HEADING_MULTIPLIER) >= 0) {
          strcat(return_string,"+");
        } else {
          strcat(return_string,"-");
        }
        //if (abs(elevation/HEADING_MULTIPLIER) < 1000) {
        //  strcat(return_string,"0");
        //}
        if (abs(elevation/HEADING_MULTIPLIER) < 100) {
          strcat(return_string,"0");
        }
        if (abs(elevation/HEADING_MULTIPLIER) < 10) {
          strcat(return_string,"0");
        }
        dtostrf(float(abs(elevation/(float)HEADING_MULTIPLIER)),0,6,tempstring);
        strcat(return_string,tempstring);            
        command_good = 1;
      }
        #endif // FEATURE_ELEVATION_CONTROL
    } // end of two byte commands



    if (slave_command_buffer_index == 3) {
      if (((slave_command_buffer[0] == 'S') && (slave_command_buffer[1] == 'A')) & (slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 53)) {
        serial_read_event_flag[slave_command_buffer[2] - 48] = 1;
        command_good = 1;
        strcpy(return_string,"OK");
      }
      if (((slave_command_buffer[0] == 'S') && (slave_command_buffer[1] == 'D')) & (slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 53)) {
        serial_read_event_flag[slave_command_buffer[2] - 48] = 0;
        command_good = 1;
        strcpy(return_string,"OK");
      }

    }


    if (slave_command_buffer_index == 4) {
      if ((slave_command_buffer[0] == 'S') && (slave_command_buffer[1] == 'W')) { // Serial Write command
        switch (slave_command_buffer[2]) {
          case '0': control_port->write(slave_command_buffer[3]); command_good = 1; break;
          #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
          case '1': remote_unit_port->write(slave_command_buffer[3]); command_good = 1; break;
          #endif
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'O')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          #ifdef DEBUG_PROCESS_SLAVE
          debug_print("service_serial_buffer: pin_value: ");
          debug_print_int(pin_value);
          #endif // DEBUG_PROCESS_SLAVE
          strcpy(return_string,"OK");
          pinModeEnhanced(pin_value, OUTPUT);
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'H')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          digitalWriteEnhanced(pin_value, HIGH);
          strcpy(return_string,"OK");
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'L')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          digitalWriteEnhanced(pin_value, LOW);
          strcpy(return_string,"OK");
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'I')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          strcpy(return_string,"OK");
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'P')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          pinModeEnhanced(pin_value, INPUT);
          digitalWriteEnhanced(pin_value, HIGH);
          strcpy(return_string,"OK");
        }
      }

      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'R')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          byte pin_read = digitalReadEnhanced(pin_value);
          strcpy(return_string,"DR");
          dtostrf((slave_command_buffer[2]-48),0,0,tempstring);
          strcat(return_string,tempstring);              
          dtostrf((slave_command_buffer[3]-48),0,0,tempstring);
          strcat(return_string,tempstring);  
          if (pin_read) {
            strcat(return_string,"1");
          } else {
            strcat(return_string,"0");
          }
        }
      }
      if ((slave_command_buffer[0] == 'A') && (slave_command_buffer[1] == 'R')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          int pin_read = analogReadEnhanced(pin_value);
          strcpy(return_string,"AR");
          if (toupper(slave_command_buffer[2]) == 'A') {
            strcat(return_string,"A");
          } else {
            dtostrf((slave_command_buffer[2]-48),0,0,tempstring);
            strcat(return_string,tempstring);
          }
                        
          dtostrf((slave_command_buffer[3]-48),0,0,tempstring);
          strcat(return_string,tempstring);  
          if (pin_read < 1000) {
            strcat(return_string,"0");
          }
          if (pin_read < 100) {
            strcat(return_string,"0");
          }
          if (pin_read < 10) {
            strcat(return_string,"0");
          }
          dtostrf(pin_read,0,0,tempstring);
          strcat(return_string,tempstring);             
        }
      }

      if ((slave_command_buffer[0] == 'N') && (slave_command_buffer[1] == 'T')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          command_good = 1;
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          noTone(pin_value);
          strcpy(return_string,"OK");
        }
      }

    } // if (slave_command_buffer_index == 4)

    if (slave_command_buffer_index == 7) {
      if ((slave_command_buffer[0] == 'A') && (slave_command_buffer[1] == 'W')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          int write_value = ((slave_command_buffer[4] - 48) * 100) + ((slave_command_buffer[5] - 48) * 10) + (slave_command_buffer[6] - 48);
          if ((write_value >= 0) && (write_value < 256)) {
            analogWriteEnhanced(pin_value, write_value);
            strcpy(return_string,"OK");
            command_good = 1;
          }
        }
      }
    }

    if (slave_command_buffer_index == 8) {
      if ((slave_command_buffer[0] == 'D') && (slave_command_buffer[1] == 'T')) {
        if ((((slave_command_buffer[2] > 47) && (slave_command_buffer[2] < 58)) || (toupper(slave_command_buffer[2]) == 'A')) && (slave_command_buffer[3] > 47) && (slave_command_buffer[3] < 58)) {
          byte pin_value = 0;
          if (toupper(slave_command_buffer[2]) == 'A') {
            pin_value = get_analog_pin(slave_command_buffer[3] - 48);
          } else {
            pin_value = ((slave_command_buffer[2] - 48) * 10) + (slave_command_buffer[3] - 48);
          }
          int write_value = ((slave_command_buffer[4] - 48) * 1000) + ((slave_command_buffer[5] - 48) * 100) + ((slave_command_buffer[6] - 48) * 10) + (slave_command_buffer[7] - 48);
          if ((write_value >= 0) && (write_value <= 9999)) {
            tone(pin_value, write_value);
            strcpy(return_string,"OK");
            command_good = 1;
          }
        }
      }
    }


    if (!command_good) {
      strcpy(return_string,"ER0289");
    }
  }

  slave_command_buffer_index = 0;

}
#endif //FEATURE_REMOTE_UNIT_SLAVE

// --------------------------------------------------------------


#ifdef FEATURE_YAESU_EMULATION
void process_yaesu_command(byte * yaesu_command_buffer, int yaesu_command_buffer_index, byte source_port, char * return_string){



    char tempstring[11] = "";
    int parsed_value = 0;
  
    int parsed_elevation = 0;
  

    #ifdef FEATURE_TIMED_BUFFER
    int parsed_value2 = 0;
    #endif //FEATURE_TIMED_BUFFER

    strcpy(return_string,"");

    switch (yaesu_command_buffer[0]) {          // look at the first character of the command
      case 'C':                                // C - return current azimuth
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: C\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef OPTION_DELAY_C_CMD_OUTPUT
        delay(400);
        #endif    
        //strcpy(return_string,"");
        #ifndef OPTION_GS_232B_EMULATION
        strcat(return_string,"+0");
        #else
        strcat(return_string,"AZ=");
        #endif
        dtostrf(int(azimuth / HEADING_MULTIPLIER),0,0,tempstring);
        if (int(azimuth / HEADING_MULTIPLIER) < 10) {
          strcat(return_string,"0");
        }
        if (int(azimuth / HEADING_MULTIPLIER) < 100) {
          strcat(return_string,"0");
        }
        strcat(return_string,tempstring);
      
        #ifdef FEATURE_ELEVATION_CONTROL
        #ifndef OPTION_C_COMMAND_SENDS_AZ_AND_EL
        if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the C2 command?
        #endif


        #ifndef OPTION_GS_232B_EMULATION
        if (elevation < 0) {
          strcat(return_string,"-0");
        } else {
          strcat(return_string,"+0");
        }
        #endif
        #ifdef OPTION_GS_232B_EMULATION
        strcat(return_string,"EL=");
        #endif
        dtostrf(int(elevation / HEADING_MULTIPLIER),0,0,tempstring);
        if (int(elevation / HEADING_MULTIPLIER) < 10) {
          strcat(return_string,("0"));
        }
        if (int(elevation / HEADING_MULTIPLIER) < 100) {
          strcat(return_string,"0");
        }
        strcat(return_string,tempstring);

        #ifndef OPTION_C_COMMAND_SENDS_AZ_AND_EL
        } else {
          //strcat(return_string,"\n");
        }
        #endif // OPTION_C_COMMAND_SENDS_AZ_AND_EL
        #endif // FEATURE_ELEVATION_CONTROL
      
        #ifndef FEATURE_ELEVATION_CONTROL
        if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the C2 command?
          #ifndef OPTION_GS_232B_EMULATION
          strcat(return_string,"+0000");    // return a dummy elevation since we don't have the elevation feature turned on
          #else
          strcat(return_string,"EL=000");
          #endif
        } else {
          //strcat(return_string,"\n");
        }
        #endif // FEATURE_ELEVATION_CONTROL   
        break;
        
        
        //-----------------end of C command-----------------
        
      #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
      case 'F': // F - full scale calibration
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: F\n");
        }
        #endif // DEBUG_PROCESS_YAESU
      
      
        #ifdef FEATURE_ELEVATION_CONTROL
        if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the F2 command?

          clear_serial_buffer();
          if (source_port == CONTROL_PORT0){
            control_port->println(F("Elevate to 180 (or max elevation) and send keystroke..."));
          }
          get_keystroke();
          read_elevation(1);
          configuration.analog_el_max_elevation = analog_el;
          write_settings_to_eeprom();
          strcpy(return_string,"Wrote to memory");
          return;
        }
        #endif
      
        clear_serial_buffer();
        if (source_port == CONTROL_PORT0){
          control_port->println(F("Rotate to full CW and send keystroke..."));
          get_keystroke();
        }
        read_azimuth(1);
        configuration.analog_az_full_cw = analog_az;
        write_settings_to_eeprom();
        strcpy(return_string,"Wrote to memory");     
        break;
        #endif // FEATURE_AZ_POSITION_POTENTIOMETER
      case 'H': print_help(source_port); break;                     // H - print help - depricated
      case 'L':  // L - manual left (CCW) rotation
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: L\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        submit_request(AZ, REQUEST_CCW, 0, 21);
        //strcpy(return_string,"\n");
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        break;
        
      #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
      case 'O':  // O - offset calibration
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: O\n");
        }
        #endif // DEBUG_PROCESS_YAESU

        #ifdef FEATURE_ELEVATION_CONTROL
        if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the O2 command?       
          clear_serial_buffer();
          if (source_port == CONTROL_PORT0){  
            control_port->println(F("Elevate to 0 degrees and send keystroke..."));
          }
          get_keystroke();
          read_elevation(1);
          configuration.analog_el_0_degrees = analog_el;
          write_settings_to_eeprom();
          strcpy(return_string,"Wrote to memory");
          return;
        }
        #endif
      
        clear_serial_buffer();  
        if (source_port == CONTROL_PORT0){    
          control_port->println(F("Rotate to full CCW and send keystroke..."));
        }
        get_keystroke();
        read_azimuth(1);
        configuration.analog_az_full_ccw = analog_az;
        write_settings_to_eeprom();
        strcpy(return_string,"Wrote to memory");
        break;
        #endif // FEATURE_AZ_POSITION_POTENTIOMETER
      
      case 'R':  // R - manual right (CW) rotation
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: R\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        submit_request(AZ, REQUEST_CW, 0, 22);
        strcpy(return_string,"\n");
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        break;
        
      case 'A':  // A - CW/CCW rotation stop
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: A\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        submit_request(AZ, REQUEST_STOP, 0, 23);
        //strcpy(return_string,"\n");
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        break;
        
      case 'S':         // S - all stop
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: S\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        submit_request(AZ, REQUEST_STOP, 0, 24);
        #ifdef FEATURE_ELEVATION_CONTROL
        submit_request(EL, REQUEST_STOP, 0, 25);
        #endif
        #ifdef FEATURE_TIMED_BUFFER
        clear_timed_buffer();
        #endif // FEATURE_TIMED_BUFFER
        //strcpy(return_string,"");
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        break;
        
      case 'M': // M - auto azimuth rotation
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: M\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
  
        if (yaesu_command_buffer_index > 4) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
          #ifdef FEATURE_TIMED_BUFFER
          clear_timed_buffer();
          parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
          if ((parsed_value > 0) && (parsed_value < 1000)) {
            timed_buffer_interval_value_seconds = parsed_value;
            for (int x = 5; x < yaesu_command_buffer_index; x = x + 4) {
              parsed_value = ((int(yaesu_command_buffer[x]) - 48) * 100) + ((int(yaesu_command_buffer[x + 1]) - 48) * 10) + (int(yaesu_command_buffer[x + 2]) - 48);
              if ((parsed_value >= 0) && (parsed_value <= 360)) {  // is it a valid azimuth?
                timed_buffer_azimuths[timed_buffer_number_entries_loaded] = parsed_value * HEADING_MULTIPLIER;
                timed_buffer_number_entries_loaded++;
                timed_buffer_status = LOADED_AZIMUTHS;
                if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) {   // is the array full?
                  submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[0], 26);  // array is full, go to the first azimuth
                  timed_buffer_entry_pointer = 1;
                  return;
                }
              } else {   // we hit an invalid bearing
                timed_buffer_status = EMPTY;
                timed_buffer_number_entries_loaded = 0;
                strcpy(return_string,"?>");  // error
                return;
              }
            }
            submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[0], 27);   // go to the first azimuth
            timed_buffer_entry_pointer = 1;       
          } else {
            strcpy(return_string,"?>");  // error
          }
          #else
          strcpy(return_string,"?>");
          #endif // FEATURE_TIMED_BUFFER
          return;
        } else {                         // if there are four characters, this is just a single direction setting
          if (yaesu_command_buffer_index == 4) {
            parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
            #ifdef FEATURE_TIMED_BUFFER
            clear_timed_buffer();
            #endif // FEATURE_TIMED_BUFFER
            if ((parsed_value >= 0) && (parsed_value <= (azimuth_starting_point + azimuth_rotation_capability))) {
              submit_request(AZ, REQUEST_AZIMUTH, (parsed_value * HEADING_MULTIPLIER), 28);
              return;
            }
          }
        }
        strcpy(return_string,"?>");      
        break;
        
      #ifdef FEATURE_TIMED_BUFFER
      case 'N': // N - number of loaded timed interval entries
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: N\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        sprintf(return_string,"%d",timed_buffer_number_entries_loaded);
        break;
        #endif // FEATURE_TIMED_BUFFER
        
      #ifdef FEATURE_TIMED_BUFFER
      case 'T': // T - initiate timed tracking
        initiate_timed_buffer(source_port);
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        break;           
        #endif // FEATURE_TIMED_BUFFER
        
      case 'X':  // X - azimuth speed change
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: X\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        
        
        if (yaesu_command_buffer_index > 1) {
          switch (yaesu_command_buffer[1]) {
            case '4':
              normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
              update_az_variable_outputs(PWM_SPEED_VOLTAGE_X4);
              #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
              normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
              update_el_variable_outputs(PWM_SPEED_VOLTAGE_X4);
              #endif
              strcpy(return_string,"Speed X4");
              break;
            case '3':
              normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X3;
              update_az_variable_outputs(PWM_SPEED_VOLTAGE_X3);
              #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
              normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X3;
              update_el_variable_outputs(PWM_SPEED_VOLTAGE_X3);
              #endif
              strcpy(return_string,"Speed X3");
              break;
            case '2':
              normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X2;
              update_az_variable_outputs(PWM_SPEED_VOLTAGE_X2);
              #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
              normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X2;
              update_el_variable_outputs(PWM_SPEED_VOLTAGE_X2);
              #endif
              strcpy(return_string,"Speed X2");
              break;
            case '1':
              normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X1;
              update_az_variable_outputs(PWM_SPEED_VOLTAGE_X1);
              #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
              normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X1;
              update_el_variable_outputs(PWM_SPEED_VOLTAGE_X1);
              #endif
              strcpy(return_string,"Speed X1");
              break;
            default: strcpy(return_string,"?>"); break;
          } /* switch */
        } else {
          strcpy(return_string,"?>");
        }
        break;
        
      #ifdef FEATURE_ELEVATION_CONTROL
      case 'U':  // U - manual up rotation
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: U\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        submit_request(EL, REQUEST_UP, 0, 29);
        //strcpy(return_string,"\n");
        break;
        
      case 'D':  // D - manual down rotation
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: D\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        submit_request(EL, REQUEST_DOWN, 0, 30);
        //strcpy(return_string,"\n");
        break;
        
      case 'E':  // E - stop elevation rotation
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: E\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        submit_request(EL, REQUEST_STOP, 0, 31);
        //strcpy(return_string,"\n");
        break;
        
      case 'B': // B - return current elevation
        #ifndef OPTION_GS_232B_EMULATION
        if (elevation < 0) {
          strcat(return_string,"-0");
        } else {
          strcat(return_string,"+0");
        }
        #else
        strcat(return_string,"EL=");
        #endif //OPTION_GS_232B_EMULATION
        dtostrf(int(elevation / HEADING_MULTIPLIER),0,0,tempstring);
        if (int(elevation / HEADING_MULTIPLIER) < 10) {
          strcat(return_string,("0"));
        }
        if (int(elevation / HEADING_MULTIPLIER) < 100) {
          strcat(return_string,"0");
        }
        strcat(return_string,tempstring);
      break;        
      
      #endif /* ifdef FEATURE_ELEVATION_CONTROL */
      
      case 'W':  // W - auto elevation rotation
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("yaesu_serial_command: W\n");
        }
        #endif // DEBUG_PROCESS_YAESU
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
        
        
        // parse out W command
        // Short Format: WXXX YYYY           XXX = azimuth YYY = elevation
        // Long Format : WSSS XXX YYY        SSS = timed interval   XXX = azimuth    YYY = elevation

        if (yaesu_command_buffer_index > 8) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
          #if defined(FEATURE_TIMED_BUFFER) && defined(FEATURE_ELEVATION_CONTROL) 
          parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
          if ((parsed_value > 0) && (parsed_value < 1000)) {
            timed_buffer_interval_value_seconds = parsed_value;
            for (int x = 5; x < yaesu_command_buffer_index; x = x + 8) {
              parsed_value = ((int(yaesu_command_buffer[x]) - 48) * 100) + ((int(yaesu_command_buffer[x + 1]) - 48) * 10) + (int(yaesu_command_buffer[x + 2]) - 48);
              parsed_value2 = ((int(yaesu_command_buffer[x + 4]) - 48) * 100) + ((int(yaesu_command_buffer[x + 5]) - 48) * 10) + (int(yaesu_command_buffer[x + 6]) - 48);
              if ((parsed_value > -1) && (parsed_value < 361) && (parsed_value2 > -1) && (parsed_value2 < 181)) {  // is it a valid azimuth?
                timed_buffer_azimuths[timed_buffer_number_entries_loaded] = (parsed_value * HEADING_MULTIPLIER);
                timed_buffer_elevations[timed_buffer_number_entries_loaded] = (parsed_value2 * HEADING_MULTIPLIER);
                timed_buffer_number_entries_loaded++;
                timed_buffer_status = LOADED_AZIMUTHS_ELEVATIONS;
                if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) {   // is the array full?
                  x = yaesu_command_buffer_index;  // array is full, go to the first azimuth and elevation
      
                }
              } else {   // we hit an invalid bearing
                timed_buffer_status = EMPTY;
                timed_buffer_number_entries_loaded = 0;
                strcpy(return_string,"?>");  // error
                return;
              }
            }
          }
          timed_buffer_entry_pointer = 1;             // go to the first bearings
          parsed_value = timed_buffer_azimuths[0];
          parsed_elevation = timed_buffer_elevations[0];
          #else /* ifdef FEATURE_TIMED_BUFFER FEATURE_ELEVATION_CONTROL*/
          strcpy(return_string,"?>");
          #endif // FEATURE_TIMED_BUFFER FEATURE_ELEVATION_CONTROL
        } else {
          // this is a short form W command, just parse the azimuth and elevation and initiate rotation
          parsed_value = (((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48)) * HEADING_MULTIPLIER;
          parsed_elevation = (((int(yaesu_command_buffer[5]) - 48) * 100) + ((int(yaesu_command_buffer[6]) - 48) * 10) + (int(yaesu_command_buffer[7]) - 48)) * HEADING_MULTIPLIER;
        }
      
         #ifndef FEATURE_ELEVATION_CONTROL
        if ((parsed_value >= 0) && (parsed_value <= (360 * HEADING_MULTIPLIER))) {
          submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 32);
        } else {
          #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug_print("process_yaesu_command: W cmd az error");
          }
          #endif // DEBUG_PROCESS_YAESU
          strcpy(return_string,"?>");      // bogus elevation - return and error and don't do anything
        }
        
        #else
         if ((parsed_value >= 0) && (parsed_value <= (360 * HEADING_MULTIPLIER)) && (parsed_elevation >= 0) && (parsed_elevation <= (180 * HEADING_MULTIPLIER))) {
          submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 33);
          submit_request(EL, REQUEST_ELEVATION, parsed_elevation, 34);
        } else {
          #ifdef DEBUG_PROCESS_YAESU
          if (debug_mode) {
            debug_print("process_yaesu_command: W cmd az/el error");
          }
          #endif // DEBUG_PROCESS_YAESU
          strcpy(return_string,"?>");      // bogus elevation - return and error and don't do anything
        } 
        #endif // FEATURE_ELEVATION_CONTROL
        
        
        break;
        
      #ifdef OPTION_GS_232B_EMULATION
      case 'P':  // P - switch between 360 and 450 degree mode

        if ((yaesu_command_buffer[1] == '3') && (yaesu_command_buffer_index > 2)) {  // P36 command
          azimuth_rotation_capability = 360;
          strcpy(return_string,"Mode 360 degree");
          // write_settings_to_eeprom();
        } else {
          if ((yaesu_command_buffer[1] == '4') && (yaesu_command_buffer_index > 2)) { // P45 command
            azimuth_rotation_capability = 450;
            strcpy(return_string,"Mode 450 degree");
            // write_settings_to_eeprom();
          } else {
            strcpy(return_string,"?>");
          }
        }
   
      
      break;                 
      case 'Z':                                           // Z - Starting point toggle
        if (azimuth_starting_point == 180) {
          azimuth_starting_point = 0;
          strcpy(return_string,"N");
        } else {
          azimuth_starting_point = 180;
          strcpy(return_string,"S");
        }
        strcat(return_string," Center");
        // write_settings_to_eeprom();
        break;
        #endif
        
      default:
        strcpy(return_string,"?>");
        #ifdef DEBUG_PROCESS_YAESU
        if (debug_mode) {
          debug_print("process_yaesu_command: yaesu_command_buffer_index: ");
          debug_print_int(yaesu_command_buffer_index);
          for (int debug_x = 0; debug_x < yaesu_command_buffer_index; debug_x++) {
            debug_print("process_yaesu_command: yaesu_command_buffer[");
            debug_print_int(debug_x);
            debug_print("]: ");
            debug_print_int(yaesu_command_buffer[debug_x]);
            debug_print(" ");
            debug_write_int(yaesu_command_buffer[debug_x]);
            debug_print("\n");;
          }
        }
        #endif // DEBUG_PROCESS_YAESU
    } /* switch */

} /* yaesu_serial_command */
  #endif // FEATURE_YAESU_EMULATION
// --------------------------------------------------------------

        