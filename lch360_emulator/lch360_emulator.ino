
#include <SerialCommand.h>

SerialCommand sCmd;     // The SerialCommand object
int delayauto = 1000;
String output = "+181.010";
boolean automa = true ;
int val = 0;

//float val3 = 0.0;s
float r;
void setup() {


  // put your setup code here, to run once:
  Serial.begin(9600);           // set up Serial library at 9600 bps
  while (! Serial);
  sCmd.addCommand("get-360",    print_angle);          // prints angle to serial
  sCmd.addCommand("str0100",   set_delay_100ms);         // sets auto print delay to 100ms
  sCmd.addCommand("str1000",   set_delay_1000ms);         // sets auto print delay to 1000ms
  sCmd.addCommand("setcasc", setauto);        // set auto print on
  sCmd.addCommand("stpcasc",     stopauto);  // stop auto print
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
  Serial.println("LCH-360 emulator");
  Serial.println("get-360 gives onetime readings, setcasc gives reading every 0.1 sec (set by str0100),stpcasc stops ");
}

void loop() {
  
  // put your main code here, to run repeatedly:
   sCmd.readSerial(); 
    if (automa)
     {
        Serial.println(output);
        delay(delayauto); 
     }
     val = analogRead(5);
     //val3 = 360/1024;
     float val2 = 0.0; 
     val2 = ((float)val*((float)360/(float)1024))-(float)180; 
     delay(delayauto/10);
     char buf[16];

     r = sprintf(buf, "%04d", val2);
     String str2 =  String(val2, 3);
     
     // creating somewhat similar output as sensor, hax hax
     String outout =  String();
     //Serial.println(sizeof(str2));
     //Serial.println(str2.length());
     //Serial.println(str2);
     //char outout[7] = "+000.000";
     //Serial.println(sizeof(outout));
     if (val2 < 0){
         outout +=  "-";
         str2.remove(0,1);
         //if
     }
     else
     {
      outout +=  "+";
     }
     if (8-str2.length()==4){
      outout += "000";
     }
     if (8-str2.length()==3){
      outout += "00";
     }
     if (8-str2.length()==2){
      outout += "0";
     }
     
     outout += str2;
     output = outout;
     //Serial.println(outout);
    
     //Serial.println(str2);
     // now print the string inside of 'buf'
     //Serial.println(buf);
     //Serial.println(val);
     //Serial.println(val2);
     
     //Serial.println(val3);
     //String output = String(analogRead(5), DEC);

}
void print_angle() {
  Serial.println(output);
  //digitalWrite(arduinoLED, HIGH);
}
void set_delay_100ms() {
  Serial.println("OK");
  delayauto = 100;
  //digitalWrite(arduinoLED, HIGH);
}
void set_delay_1000ms() {
  Serial.println("OK");
  delayauto = 1000;
  //digitalWrite(arduinoLED, HIGH);
}
void setauto() {
  Serial.println("OK");
   automa = true;
  //digitalWrite(arduinoLED, HIGH);
}
void stopauto() {
  Serial.println("OK");
  automa = false;
  //digitalWrite(arduinoLED, HIGH);
}
void unrecognized(const char *command) {
  Serial.println("What?");
}

