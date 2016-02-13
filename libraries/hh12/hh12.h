#ifndef hh12_h
#define hh12_h

#define HH12_DELAY 100 // microseconds
//#define OPTION_HH12_10_BIT_READINGS


class hh12 {

  public:
    hh12();
    void initialize(int _hh12_clock_pin, int _hh12_cs_pin, int _hh12_data_pin);
    float heading();
		
  private:	  
    int hh12_clock_pin;
    int hh12_cs_pin;
    int hh12_data_pin;

};


#endif //hh12_h
