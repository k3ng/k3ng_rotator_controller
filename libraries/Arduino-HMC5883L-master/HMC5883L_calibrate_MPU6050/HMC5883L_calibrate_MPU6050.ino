/*
  Calibrate HMC5883L + MPU6050 (GY-86 / GY-87). Output for HMC5883L_calibrate_processing.pde
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

int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

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

  // Initialize Initialize HMC5883L
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
}

void loop()
{
  Vector mag = compass.readRaw();

  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;

  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;

  Serial.print(mag.XAxis);
  Serial.print(":");
  Serial.print(mag.YAxis);
  Serial.print(":");
  Serial.print(minX);
  Serial.print(":");
  Serial.print(maxX);
  Serial.print(":");
  Serial.print(minY);
  Serial.print(":");
  Serial.print(maxY);
  Serial.print(":");
  Serial.print(offX);
  Serial.print(":");
  Serial.print(offY);
  Serial.print("\n");
}
