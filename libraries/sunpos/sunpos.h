// This file is available in electronic form at http://www.psa.es/sdg/sunpos.htm

#ifndef __SUNPOS_H
#define __SUNPOS_H


// Declaration of some constants 
#define PI_SUNPOS    3.14159265358979323846
#define TWOPI_SUNPOS (2*PI_SUNPOS)
#define RAD_SUNPOS   (PI_SUNPOS/180)
#define dEarthMeanRadius     6371.01	// In km
#define dAstronomicalUnit    149597890	// In km

struct cTime
{
  int iYear;
  int iMonth;
  int iDay;
  double dHours;
  double dMinutes;
  double dSeconds;
};

struct cLocation
{
  double dLongitude;
  double dLatitude;
};

struct cSunCoordinates
{
  double dZenithAngle;
  double dAzimuth;
};

void sunpos(cTime udtTime, cLocation udtLocation, cSunCoordinates *udtSunCoordinates);

#endif

