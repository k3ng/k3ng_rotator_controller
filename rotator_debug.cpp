// 
// contributed by Matt VK5ZM
// 

#include "rotator_debug.h"

#define FEATURE_ETHERNET

void DebugClass::print(const char *str)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	  if (debug_mode & CONTROL_PORT0){
	  	control_port->print(str);
	  }
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	//   if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.print(str);
	//   }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
 //   	  if (debug_mode & ETHERNET_PORT1){
	//     ethernetclient1.print(str);
	//   }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::print(const __FlashStringHelper *str)
{
	char c;
	if(!str) return;
	
	/* since str is a const we can't increment it, so do this instead */
	char *p = (char *)str;
	
	/* keep going until we find the null */
	while((c = pgm_read_byte(p++)))
	{
		#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
		 if (debug_mode & CONTROL_PORT0)
		 {
			control_port->write(c);
		 }
		#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

		// #ifdef FEATURE_ETHERNET
		//  if (debug_mode & ETHERNET_PORT0)
		//  {
		// 	ethernetclient0.write(c);
		//  }
		// #endif //FEATURE_ETHERNET

		// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
		//  if (debug_mode & ETHERNET_PORT1)
		//  {
		// 	ethernetclient1.write(c);
		//  }
		// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	}
}

void DebugClass::println(const __FlashStringHelper *str)
{
	char c;
	if(!str) return;
	
	/* since str is a const we can't increment it, so do this instead */
	char *p = (char *)str;
	
	/* keep going until we find the null */
	while((c = pgm_read_byte(p++)))
	{
		#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
		 if (debug_mode & CONTROL_PORT0)
		 {
			control_port->write(c);
		 }
		#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

		// #ifdef FEATURE_ETHERNET
		//  if (debug_mode & ETHERNET_PORT0)
		//  {
		// 	ethernetclient0.write(c);
		//  }
		// #endif //FEATURE_ETHERNET

		// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
		//  if (debug_mode & ETHERNET_PORT1)
		//  {
		// 	ethernetclient1.write(c);
		//  }
		// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	}
	control_port->println();
}

void DebugClass::print(char ch)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->print(ch);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.print(ch);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.print(ch);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::print(int i)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->print(i);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.print(i);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.print(i);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::print(unsigned int i)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->print(i);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.print(i);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.print(i);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::print(long unsigned int i)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->print(i);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.print(i);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.print(i);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::print(long i)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->print(i);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.print(i);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.print(i);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::print(double i)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->print(i);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.print(i);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.print(i);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::println(double i)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->println(i);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.println(i);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.println(i);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::print(float f,byte places)
{
	char tempstring[16] = "";

	dtostrf( f,0,places,tempstring);

	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->print(tempstring);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.print(tempstring);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.print(tempstring);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::print(float f)
{
	char tempstring[16] = "";

	dtostrf( f,0,2,tempstring);

	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->print(tempstring);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.print(tempstring);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.print(tempstring);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
}

void DebugClass::println(const char *str)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->println(str);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.println(str);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.println(str);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)

}

void DebugClass::write(const char *str)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->write(str);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.write(str);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.write(str);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)

}

void DebugClass::write(int i)
{
	#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
	if (debug_mode & CONTROL_PORT0){
		control_port->write(i);
	}
	#endif //defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

	// #ifdef FEATURE_ETHERNET
	// if (debug_mode & ETHERNET_PORT0){
	// 	ethernetclient0.write(i);
	// }
	// #endif //FEATURE_ETHERNET

	// #if defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)
	// if (debug_mode & ETHERNET_PORT1){
	// 	ethernetclient1.write(i);
	// }
	// #endif //defined(FEATURE_ETHERNET) && defined(ETHERNET_TCP_PORT_1)

}
