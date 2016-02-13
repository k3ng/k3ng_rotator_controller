
#ifndef K3NG_REMOTE_H
#define K3NG_REMOTE_H

// K3NG_DISPLAY_LIBRARY_VERSION "1.0.2015061901"


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "k3ng_remote_rotator_controller.h"


uint8_t automatic_polling_active = 0;



//-----------------------------------------------------------------------------------------------------

K3NGremoteunit::K3NGremoteunit(){



}

//-----------------------------------------------------------------------------------------------------

void K3NGremoteunit::initialize(){





}

//-----------------------------------------------------------------------------------------------------

void K3NGremoteunit::service(){


  if (automatic_polling_active){

  	
  }


}

#endif //K3NG_REMOTE_H