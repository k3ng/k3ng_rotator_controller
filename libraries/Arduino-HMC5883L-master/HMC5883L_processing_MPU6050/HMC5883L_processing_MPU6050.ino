/*
  HMC5883L Triple Axis Digital Compass + MPU6050 (GY-86 / GY-87). Output for HMC5883L_processing.pde
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <HMC5883L.h>
#include <MPU6050.h>

HMC5883L compass;
MPU6050 mpu;

int previousDegree;

void setup()
{
  Serial.begin(9600);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }

  // Enable bypass mode
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true);
  mpu.setSleepEnabled(false);

  // Initialize HMC5883L
  while (!compass.begin())
  {
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0); 
}

void loop()
{
  long x = micros();
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }
 
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  // Fix HMC5883L issue with angles
  float fixedHeadingDegrees;
 
  if (headingDegrees >= 1 && headingDegrees < 240)
  {
    fixedHeadingDegrees = map(headingDegrees, 0, 239, 0, 179);
  } else
  if (headingDegrees >= 240)
  {
    fixedHeadingDegrees = map(headingDegrees, 240, 360, 180, 360);
  }

  // Smooth angles rotation for +/- 3deg
  int smoothHeadingDegrees = round(fixedHeadingDegrees);

  if (smoothHeadingDegrees < (previousDegree + 3) && smoothHeadingDegrees > (previousDegree - 3))
  {
    smoothHeadingDegrees = previousDegree;
  }
  
  previousDegree = smoothHeadingDegrees;

  // Output
  Serial.print(norm.XAxis);
  Serial.print(":");
  Serial.print(norm.YAxis);
  Serial.print(":");
  Serial.print(norm.ZAxis);
  Serial.print(":");
  Serial.print(headingDegrees);
  Serial.print(":");
  Serial.print(fixedHeadingDegrees);
  Serial.print(":");
  Serial.print(smoothHeadingDegrees);  
  Serial.println();

  // One loop: ~5ms @ 115200 serial.
  // We need delay ~28ms for allow data rate 30Hz (~33ms)
  delay(30);
}

