/*
  Implements a simple interface to the time function of the PCF8583 RTC chip

  Works around the device's limited year storage by keeping the year in the
  first two bytes of user accessible storage

  Assumes device is attached in the standard location - Analog pins 4 and 5
  Device address is the 8 bit address (as in the device datasheet - normally A0)

  Copyright (c) 2009, Erik DeBill


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


/*

   Usage:
      PCF8583 pcf(0xA0);
      pcf.get_time();

      Serial.print("year: ");
      Serial.println(pcf.year);


      pcf.hour = 14;
      pcf.minute = 30
      pcf.second = 0
      pcf.year = 2009
      pcf.month = 9
      pcf.day = 12
      pcf.set_time();


*/

#ifndef PCF8583_H
#define PCF8583_H

#include <Arduino.h>
#include <../Wire/Wire.h>

class PCF8583 {
    int address;
    int dow;
  public:
    int second;
    int minute;
    int hour;
    int day;
    int month;
    int year;
    int year_base;

    int alarm_milisec;
    int alarm_second;
    int alarm_minute;
    int alarm_hour;
    int alarm_day;

    PCF8583(int device_address);
    void init ();
    
    void get_time();
    void set_time();
    void get_alarm();
    int  get_day_of_week() const {
      return dow;
    }    

    void set_daily_alarm();
    int bcd_to_byte(byte bcd);
    byte int_to_bcd(int in);
};


#endif  //PCF8583_H
