/*
    HMC5883L Triple Axis Digital Compass.
    Processing for HMC5883L_calibrate.ino
    Processing for HMC5883L_calibrate_MPU6050.ino
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
    GIT: https://github.com/jarzebski/Arduino-HMC5883L
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

import processing.serial.*;

Serial myPort;

// Data samples
float x = 0;
float y = 0;

float minX = 0;
float maxX = 0;
float minY = 0;
float maxY = 0;
float offX = 0;
float offY = 0;

void setup ()
{
  size(500, 500, P2D);
  background(0);
  stroke(255);
  
  strokeWeight(2);

  line(250, 0, 250, 500);
  line(0, 250, 500, 250);

  strokeWeight(3);
  textSize(12);

  myPort = new Serial(this, Serial.list()[0], 9600);
  myPort.bufferUntil(10);
}

void draw() 
{
  strokeWeight(0);
  fill(0);  // Set fill to white
  rect(0, 0, 240, 50); 

  strokeWeight(2);
  fill(255);  // Set fill to white
  text(minX+" "+maxX+" = "+offX, 10, 20);
  text(minY+" "+maxY+" = "+offY, 10, 35);
  point((x*0.5)+250, (y*0.5)+250);
}

void serialEvent (Serial myPort)
{
  String inString = myPort.readStringUntil(10);

  if (inString != null)
  {
    inString = trim(inString);
    String[] list = split(inString, ':');
    String testString = trim(list[0]);

    if (list.length != 8) return;

    x = (float(list[0]));
    y = (float(list[1]));
    minX = (float(list[2]));
    maxX = (float(list[3]));
    minY = (float(list[4]));
    maxY = (float(list[5]));   
    offX = (float(list[6]));
    offY = (float(list[7]));  
  }
}
