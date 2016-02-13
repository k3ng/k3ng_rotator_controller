
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#define K3NG_REMOTE_UNIT_LIBRARY_VERSION "1.0.2015061901"

/*

  How this thing works....

+-----------+
|           |------serial/ethernet----->byte_in -->  {                      }   <-- send_command()
|  remote   |                                        { class K3NGremoteunit }   --> azimuth
|   unit    |<-----serial/ethernet-----byte_out <--  {                      }   --> elevation
|           | 
+-----------+


*/

class K3NGremoteunit {

public:

    K3NGremoteunit();
    void initialize();
    void service();
    /*
    void byte_in(uint8_t byte_received);
    void byte_out(uint8_t byte_to_send);
    uint8_t bytes_out_available();
    uint8_t send_command(uint8_t command_to_send);
    uint8_t send_command(uint8_t command_to_send,float float_parameter);
    void stop_automatic_polling();
    void resume_automatic_polling();


    unsigned long last_valid_response_received_time;
    uint8_t link_state;
    int poll_rate_ms;

    float azimuth;
    float elevation;
    uint8_t gps_state;
    uint8_t azimuth_state;
    uint8_t elevation_state;*/





private:



};