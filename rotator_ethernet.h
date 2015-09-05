#ifdef FEATURE_ETHERNET
void service_ethernet(){


  byte incoming_byte = 0;
  static unsigned long last_incoming_byte_receive_time = 0;
  char return_string[100] = ""; 
  static byte ethernet_port_buffer0[COMMAND_BUFFER_SIZE];
  static int ethernet_port_buffer_index0 = 0;
  static byte first_connect_occurred = 0;
  static long last_received_byte0 = 0;

  #ifdef FEATURE_REMOTE_UNIT_SLAVE
  static byte preamble_received = 0;
  #endif //FEATURE_REMOTE_UNIT_SLAVE

  /*  this is the server side (receiving bytes from a client such as a master unit receiving commands from a computer
      or a slave receiving commands from a master unit

  */

 
  // clear things out if we received a partial message and it's been awhile
  if ((ethernet_port_buffer_index0) && ((millis()-last_received_byte0) > ETHERNET_MESSAGE_TIMEOUT_MS)){
    ethernet_port_buffer_index0 = 0;
    #ifdef FEATURE_REMOTE_UNIT_SLAVE
    preamble_received = 0;
    #endif //FEATURE_REMOTE_UNIT_SLAVE
  }


  if (ethernetserver0.available()){
    ethernetclient0 = ethernetserver0.available();

    last_received_byte0 = millis();

    if (!first_connect_occurred){  // clean out the cruft that's alway spit out on first connect
      while(ethernetclient0.available()){ethernetclient0.read();}
      first_connect_occurred = 1;
      return;
    }    

    if (ethernetclient0.available() > 0){        // the client has sent something
      incoming_byte = ethernetclient0.read();
      last_incoming_byte_receive_time = millis();

      #ifdef DEBUG_ETHERNET
      debug_print("service_ethernet: client:") ;
      debug_print(" char:");
      debug_print_char((char) incoming_byte);
      debug_print("\n");
      #endif //DEBUG_ETHERNET  

      if ((incoming_byte > 96) && (incoming_byte < 123)) {  // uppercase it
        incoming_byte = incoming_byte - 32;
      }          

      char ethernet_preamble[] = ETHERNET_PREAMBLE;

      #ifdef FEATURE_REMOTE_UNIT_SLAVE
      if (preamble_received < 254){         // the master/slave ethernet link has each message prefixed with a preamble
        if (ethernet_preamble[preamble_received] == 0){
          preamble_received = 254;
        } else {
          if (incoming_byte == ethernet_preamble[preamble_received]){
            preamble_received++;
          } else {
            preamble_received = 0;
          }
        }
      }
      // add it to the buffer if it's not a line feed or carriage return and we've received the preamble
      if ((incoming_byte != 10) && (incoming_byte != 13) && (preamble_received == 254)) { 
        ethernet_port_buffer0[ethernet_port_buffer_index0] = incoming_byte;
        ethernet_port_buffer_index0++;
      }
      #else 
      if ((incoming_byte != 10) && (incoming_byte != 13)) { // add it to the buffer if it's not a line feed or carriage return
        ethernet_port_buffer0[ethernet_port_buffer_index0] = incoming_byte;
        ethernet_port_buffer_index0++;
      }
      #endif //FEATURE_REMOTE_UNIT_SLAVE


      if (((incoming_byte == 13) || (ethernet_port_buffer_index0 >= COMMAND_BUFFER_SIZE)) && (ethernet_port_buffer_index0 > 0)){  // do we have a carriage return?
        if ((ethernet_port_buffer0[0] == '\\') or (ethernet_port_buffer0[0] == '/')) {
          process_backslash_command(ethernet_port_buffer0, ethernet_port_buffer_index0, ETHERNET_PORT0, return_string);
        } else {
          #ifdef FEATURE_YAESU_EMULATION
          process_yaesu_command(ethernet_port_buffer0,ethernet_port_buffer_index0,ETHERNET_PORT0,return_string);
          #endif //FEATURE_YAESU_EMULATION
          #ifdef FEATURE_EASYCOM_EMULATION
          process_easycom_command(ethernet_port_buffer0,ethernet_port_buffer_index0,ETHERNET_PORT0,return_string);
          #endif //FEATURE_EASYCOM_EMULATION
          #ifdef FEATURE_REMOTE_UNIT_SLAVE
          process_remote_slave_command(ethernet_port_buffer0,ethernet_port_buffer_index0,ETHERNET_PORT0,return_string);
          #endif //FEATURE_REMOTE_UNIT_SLAVE          
        }  
        ethernetclient0.println(return_string);
        ethernet_port_buffer_index0 = 0;
        #ifdef FEATURE_REMOTE_UNIT_SLAVE
        preamble_received = 0;
        #endif //FEATURE_REMOTE_UNIT_SLAVE
      }

    }
  }


  #ifdef ETHERNET_TCP_PORT_1
  static byte ethernet_port_buffer1[COMMAND_BUFFER_SIZE];
  static int ethernet_port_buffer_index1 = 0;

  if (ethernetserver1.available()){

    ethernetclient1 = ethernetserver1.available();

    if (ethernetclient1.available() > 0){        // the client has sent something
      incoming_byte = ethernetclient1.read();
      last_incoming_byte_receive_time = millis();

      #ifdef DEBUG_ETHERNET
      debug_print("service_ethernet: client:") ;
      debug_print(" char:");
      debug_print_char((char) incoming_byte);
      debug_print("\n");
      #endif //DEBUG_ETHERNET  

      if ((incoming_byte > 96) && (incoming_byte < 123)) {  // uppercase it
        incoming_byte = incoming_byte - 32;
      }                                                                                                                    
      if ((incoming_byte != 10) && (incoming_byte != 13)) { // add it to the buffer if it's not a line feed or carriage return
        ethernet_port_buffer1[ethernet_port_buffer_index1] = incoming_byte;
        ethernet_port_buffer_index1++;
      }
      if (incoming_byte == 13) {  // do we have a carriage return?
        if ((ethernet_port_buffer1[0] == '\\') or (ethernet_port_buffer1[0] == '/')) {
          process_backslash_command(ethernet_port_buffer1, ethernet_port_buffer_index1, ETHERNET_PORT1, return_string);
        } else {
          #ifdef FEATURE_YAESU_EMULATION
          process_yaesu_command(ethernet_port_buffer1,ethernet_port_buffer_index1,ETHERNET_PORT1,return_string);
          #endif //FEATURE_YAESU_EMULATION
          #ifdef FEATURE_EASYCOM_EMULATION
          process_easycom_command(ethernet_port_buffer1,ethernet_port_buffer_index1,ETHERNET_PORT1,return_string);
          #endif //FEATURE_EASYCOM_EMULATION
          #ifdef FEATURE_REMOTE_UNIT_SLAVE
          process_remote_slave_command(ethernet_port_buffer1,ethernet_port_buffer_index1,ETHERNET_PORT1,return_string);
          #endif //FEATURE_REMOTE_UNIT_SLAVE
        }  
        ethernetclient1.println(return_string);
        ethernet_port_buffer_index1 = 0;
      }

    }
  }
  #endif //ETHERNET_TCP_PORT_1

  #ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
  static long last_connect_try = 0;
  static long last_received_byte_time = 0;
  byte incoming_ethernet_byte = 0;
  static byte first_ethernet_slave_connect_occurred = 0;

  // are we disconnected and is it time to reconnect?
  if ((ethernetslavelinkclient0_state == ETHERNET_SLAVE_DISCONNECTED)  && (((millis()-last_connect_try) >= ETHERNET_SLAVE_RECONNECT_TIME_MS) || (last_connect_try == 0))){

    #ifdef DEBUG_ETHERNET
    debug_println("service_ethernet: master_slave_ethernet: connecting");
    #endif //DEBUG_ETHERNET

    if (ethernetslavelinkclient0.connect(slave_unit_ip, ETHERNET_SLAVE_TCP_PORT)){
      ethernetslavelinkclient0_state = ETHERNET_SLAVE_CONNECTED;
      if (!first_ethernet_slave_connect_occurred){
        first_ethernet_slave_connect_occurred = 1;
        ethernet_slave_reconnects = 65535;
      }
    } else {
      ethernetslavelinkclient0.stop();
      #ifdef DEBUG_ETHERNET
      debug_println("service_ethernet: master_slave_ethernet: connect failed");
      #endif //DEBUG_ETHERNET
    }

    ethernet_slave_reconnects++;
    last_connect_try = millis();
  }


  if (ethernetslavelinkclient0.available()) {
    incoming_ethernet_byte = ethernetslavelinkclient0.read();

    #ifdef DEBUG_ETHERNET
    debug_print("service_ethernet: slave rx: ");
    debug_print_char(incoming_ethernet_byte);
    debug_print(" : ");
    debug_print_int(incoming_ethernet_byte);
    debug_println("");
    #endif //DEBUG_ETHERNET      

    if (remote_port_rx_sniff) {
      control_port->write(incoming_ethernet_byte);
    }

    if ((incoming_ethernet_byte != 10) && (remote_unit_port_buffer_index < COMMAND_BUFFER_SIZE)) {
      remote_unit_port_buffer[remote_unit_port_buffer_index] = incoming_ethernet_byte;
      remote_unit_port_buffer_index++;
      if ((incoming_ethernet_byte == 13) || (remote_unit_port_buffer_index >= COMMAND_BUFFER_SIZE)) {
        remote_unit_port_buffer_carriage_return_flag = 1;
        #ifdef DEBUG_ETHERNET
        debug_println("service_ethernet: remote_unit_port_buffer_carriage_return_flag");
        #endif //DEBUG_ETHERNET          
      }
    }
    last_received_byte_time = millis();

  }

  if (((millis() - last_received_byte_time) >= ETHERNET_MESSAGE_TIMEOUT_MS) && (remote_unit_port_buffer_index > 1) && (!remote_unit_port_buffer_carriage_return_flag)){
    remote_unit_port_buffer_index = 0;
    #ifdef DEBUG_ETHERNET
    debug_println("service_ethernet: master_slave_ethernet: remote_unit_incoming_buffer_timeout");
    #endif //DEBUG_ETHERNET    
    remote_unit_incoming_buffer_timeouts++;
  }

  if ((ethernetslavelinkclient0_state == ETHERNET_SLAVE_CONNECTED) && (!ethernetslavelinkclient0.connected())){
    ethernetslavelinkclient0.stop();
    ethernetslavelinkclient0_state = ETHERNET_SLAVE_DISCONNECTED;
    remote_unit_port_buffer_index = 0;
    #ifdef DEBUG_ETHERNET
    debug_println("service_ethernet: master_slave_ethernet: lost connection");
    #endif //DEBUG_ETHERNET    
  }


  #endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE

}
#endif //FEATURE_ETHERNET
// --------------------------------------------------------------

#ifdef FEATURE_MASTER_WITH_ETHERNET_SLAVE
byte ethernet_slave_link_send(char * string_to_send){

  if (ethernetslavelinkclient0_state == ETHERNET_SLAVE_CONNECTED){
    ethernetslavelinkclient0.print(ETHERNET_PREAMBLE);
    ethernetslavelinkclient0.println(string_to_send);
    #ifdef DEBUG_ETHERNET
    debug_print("ethernet_slave_link_send: ");
    debug_println(string_to_send);
    #endif //DEBUG_ETHERNET
    return 1;
  } else {
    #ifdef DEBUG_ETHERNET
    debug_print("ethernet_slave_link_send: link down not sending:");
    debug_println(string_to_send);
    #endif //DEBUG_ETHERNET    
    return 0;
  }

}
#endif //FEATURE_MASTER_WITH_ETHERNET_SLAVE


//-------------------------------------------------------

