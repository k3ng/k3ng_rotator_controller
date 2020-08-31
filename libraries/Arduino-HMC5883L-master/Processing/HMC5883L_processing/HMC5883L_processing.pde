/*
    HMC5883L Triple Axis Digital Compass.
    Processing for HMC5883L_processing.ino
    Processing for HMC5883L_processing_MPU6050.ino
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
    GIT: https://github.com/jarzebski/Arduino-HMC5883L
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

import processing.serial.*;

Serial myPort;

// Data samples
int actualSample = 0;
int maxSamples = 400;
int sampleStep = 1;
boolean hasData = false;

// Charts
PGraphics pgChart;
int[] colors = { #ff4444, #33ff99, #5588ff };
String[] magneticSeries = { "XAxis", "YAxis", "ZAxis" };
String[] headingSeries = { "Normal", "Fixed", "Smooth" };

// Data for compare
float[][] magneticValues = new float[3][maxSamples];
float[][] headingValues = new float[3][maxSamples];

// Artificial Horizon
PGraphics pgCompassPlate;
PImage imgCompass;
PImage imgCompassRing;
PImage imgCompassPlateWhite;
PImage imgCompassPlateBlack;
int compassWidth;
int compassHeight;

void setup ()
{
  size(755, 550, P2D);
  background(0);

  // Init
  initCompass();

  // Serial
  myPort = new Serial(this, Serial.list()[0], 9600);
  myPort.bufferUntil(10);
}

void drawChart(String title, String[] series, float[][] chart, int x, int y, int h, boolean symmetric, boolean fixed, int fixedMin, int fixedMax, int hlines) 
{
  int actualColor = 0;
  
  int maxA = 0;
  int maxB = 0;
  int maxAB = 0;
  
  int min = 0;
  int max = 0;
  int step = 0;
  int divide = 0;
 
  if (fixed)
  {
    min = fixedMin;
    max = fixedMax;
    step = hlines;
  } else
  {
    if (hlines > 2)
    {
      divide = (hlines - 2);
    } else
    {
      divide = 1;
    }
      
    if (symmetric)
    {
      maxA = (int)abs(getMin(chart));
      maxB = (int)abs(getMax(chart));
      maxAB = max(maxA, maxB);
      step = (maxAB * 2) / divide;
      min = -maxAB-step;
      max = maxAB+step;
    } else
    {
      min = (int)(getMin(chart));
      max = (int)(getMax(chart));
      
      if ((max >= 0) && (min <= 0)) step = (abs(min) + abs(max)) / divide; 
      if ((max < 0) && (min < 0)) step = abs(min - max) / divide; 
      if ((max > 0) && (min > 0)) step = (max - min) / divide; 
      
      if (divide > 1)
      {
        min -= step;
        max += step;
      }
    }
  }

  pgChart = createGraphics((maxSamples*sampleStep)+50, h+60);

  pgChart.beginDraw();

  // Draw chart area and title
  pgChart.background(0);
  pgChart.strokeWeight(1);
  pgChart.noFill();
  pgChart.stroke(50);
  pgChart.rect(0, 0, (maxSamples*sampleStep)+49, h+59);
  pgChart.text(title, ((maxSamples*sampleStep)/2)-(textWidth(title)/2)+40, 20);

  // Draw chart description
  String Description[] = new String[chart.length];
  int DescriptionWidth[] = new int[chart.length];
  int DesctiptionTotalWidth = 0;
  int DescriptionOffset = 0;

  for (int j = 0; j < chart.length; j++)
  {
     Description[j] = "  "+series[j]+" = ";
     DescriptionWidth[j] += textWidth(Description[j]+"+0000.00");
     Description[j] += nf(chart[j][actualSample-1], 0, 2)+"  ";
     DesctiptionTotalWidth += DescriptionWidth[j];
  }

  actualColor = 0;

  for (int j = 0; j < chart.length; j++)
  {
    pgChart.fill(colors[actualColor]);
    pgChart.text(Description[j], ((maxSamples*sampleStep)/2)-(DesctiptionTotalWidth/2)+DescriptionOffset+40, h+50);
    DescriptionOffset += DescriptionWidth[j];
    actualColor++;
    if (actualColor >= colors.length) actualColor = 0;
  }

  // Draw H-Lines 
  pgChart.stroke(100);

  for (float t = min; t <= max; t=t+step)
  {
    float line = map(t, min, max, 0, h);
    pgChart.line(40, h-line+30, (maxSamples*sampleStep)+40, h-line+30);
    pgChart.fill(200, 200, 200);
    pgChart.textSize(12);
    pgChart.text(int(t), 5, h-line+34);
  }

  // Draw data series
  pgChart.strokeWeight(2);

  for (int i = 1; i < actualSample; i++)
  {
    actualColor = 0;

    for (int j = 0; j < chart.length; j++)
    {
      pgChart.stroke(colors[actualColor]);

      float d0 = chart[j][i-1];
      float d1 = chart[j][i];

      if (d0 < min) d0 = min;
      if (d0 > max) d0 = max;
      if (d1 < min) d1 = min;
      if (d1 > max) d1 = max;

      float v0 = map(d0, min, max, 0, h);
      float v1 = map(d1,   min, max, 0, h);

      pgChart.line(((i-1)*sampleStep)+40, h-v0+30, (i*sampleStep)+40, h-v1+30);

      actualColor++;

      if (actualColor >= colors.length) actualColor = 0;
    }
  }

  pgChart.endDraw();

  image(pgChart, x, y);
}

void initCompass()
{
  imgCompass = loadImage("compass.png");
  imgCompassRing = loadImage("compassRing.png");
  imgCompassPlateWhite = loadImage("compassPlateWhite.png");
  imgCompassPlateBlack = loadImage("compassPlateBlack.png");
  compassWidth = imgCompass.width;
  compassHeight = imgCompass.height;
}

void drawCompass(int x, int y, float[][] head, PImage plate)
{
  pgCompassPlate = createGraphics(compassWidth, compassWidth); 

  float heading = head[2][actualSample-1];
  float north = 180 + heading;
  
  pgCompassPlate.beginDraw();
  pgCompassPlate.clear();
  pgCompassPlate.translate(100,100);
  pgCompassPlate.rotate(-radians(heading));
  pgCompassPlate.image(plate, -100, -100);
  pgCompassPlate.endDraw();

  image(pgCompassPlate, x+30, y+30);
  image(imgCompass, x, y);
  image(imgCompassRing, x, y);

  textAlign(CENTER);
  text((int)heading+" deg", x+130, y+265);
  textAlign(LEFT);
}

void draw() 
{
  if (!hasData) return;

  background(0);

  drawChart("Magnetic field [mG]", magneticSeries, magneticValues, 10, 10, 200, false, false, 0, 0, 10);
  drawChart("Heading [deg]", headingSeries, headingValues, 10, 280, 200, true, true, 0, 360, 30);
  drawCompass(480, 5, headingValues, imgCompassPlateWhite);
  drawCompass(480, 275, headingValues, imgCompassPlateBlack);

}

float getMin(float[][] chart)
{
  float minValue = 0;
  float[] testValues = new float[chart.length];
  float testMin = 0;

  for (int i = 0; i < actualSample; i++)
  {
    for (int j = 0; j < testValues.length; j++)
    {
      testValues[j] = chart[j][i];
    }
    
    testMin = min(testValues);
    
    if (i == 0)
    {
      minValue = testMin;
    } else
    {
      if (minValue > testMin) minValue = testMin;
    }
  }
 
  return ceil(minValue)-1; 
}

float getMax(float[][] chart)
{
  float maxValue = 0;
  float[] testValues = new float[chart.length];
  float testMax = 0;

  for (int i = 0; i < actualSample; i++)
  {
    for (int j = 0; j < testValues.length; j++)
    {
      testValues[j] = chart[j][i];
    }
    
    testMax = max(testValues);

    if (i == 0)
    {
      maxValue = testMax;
    } else
    {
      if (maxValue < testMax) maxValue = testMax;
    }
  }
 
  return ceil(maxValue); 
}

void nextSample(float[][] chart)
{
    for (int j = 0; j < chart.length; j++)
    {
      float last = chart[j][maxSamples-1];

      for (int i = 1; i < maxSamples; i++)
      {
        chart[j][i-1] = chart[j][i];
      }

      chart[j][(maxSamples-1)] = last;
    }
}

void serialEvent (Serial myPort)
{
  String inString = myPort.readStringUntil(10);

  if (inString != null)
  {
    inString = trim(inString);
    String[] list = split(inString, ':');
    String testString = trim(list[0]);

    if (list.length != 6) return;

    // Magnetic field
    magneticValues[0][actualSample] = (float(list[0]));
    magneticValues[1][actualSample] = (float(list[1]));
    magneticValues[2][actualSample] = (float(list[2]));

    // Headings
    headingValues[0][actualSample] = (float(list[3]));
    headingValues[1][actualSample] = (float(list[4]));
    headingValues[2][actualSample] = (float(list[5]));

    if (actualSample > 1)
    {
      hasData = true;
    }

    if (actualSample == (maxSamples-1))
    {
      nextSample(magneticValues);
      nextSample(headingValues);
   } else
    {
      actualSample++;
    }
  }
}
