#include <Arduino.h>
#include <math.h>

#include <string.h>
#include <ctype.h>

// Translated from the WSJT Fortran code by Pete VE5VA

/////////////////////////////////////////////////////////
/////////////        G R I D 2 D E G        /////////////
/////////////////////////////////////////////////////////
// Choose which version of the grid conversion to use.
// Defining WSJT uses the one from WSJT
// Removing the define of WSJT uses the code based on
// the PERL script at wikipedia which seems to be
// slightly more accurate.
//#define WSJT

#ifdef WSJT
// grid = DO62QC  <->  lat = 52.104167 lon = -106.658332
// grid = JO62MM  <->  lat = 52.520832 lon = 13.008334
// The WSJT code returns West as positive but moon2 uses
// West is negative
// CHANGE this so that it returns West longitude as NEGATIVE
void grid2deg(char *grid0,double *dlong,double *dlat)
{
  char grid[8];
  int nlong,n20d;
  int nlat;
  double xminlong,xminlat;
  
  // Initialize the grid with a default string
  strncpy(grid,"AA00MM",6);
  // copy in the grid
  strncpy(grid,grid0,6);
  
  // Convert the grid to upper case
  for(int i = 0;i < 6;i++)grid[i] = toupper(grid[i]);
  
  // Fix any errors
  if(!isalpha(grid[0]))grid[0] = 'A';
  if(!isalpha(grid[1]))grid[1] = 'A';
  if(!isdigit(grid[2]))grid[2] = '0';
  if(!isdigit(grid[3]))grid[3] = '0';
  if(!isalpha(grid[4]))grid[4] = 'M';
  if(!isalpha(grid[5]))grid[5] = 'M';
  

  nlong = 180 - 20*(grid[0] - 'A');
  n20d = 2*(grid[2] - '0');
  xminlong = 5*(grid[4]-'A')+ 0.5;
  // Make west longitude negative
  *dlong = -(nlong - n20d -xminlong/60.);
  
  nlat = -90 + 10*(grid[1] - 'A') + grid[3] - '0';
  xminlat = 2.5*(grid[5] - 'A' + 0.5);
  *dlat = nlat + xminlat/60.;
}
#else
// grid = DO62QC  <->  lat = 52.104164 lon = -106.625000
// grid = JO62MM  <->  lat = 52.520832 lon = 13.041667

// From: http://en.wikipedia.org/wiki/Maidenhead_Locator_System
void grid2deg(char *grid0,double *dlong,double *dlat)
{
  char grid[8];
  
  // Initialize the grid with a default string
  strncpy(grid,"AA00MM",6);
  // copy in the grid
  strncpy(grid,grid0,6);

  // Convert the grid to upper case
  for(int i = 0;i < 6;i++)grid[i] = toupper(grid[i]);
  
  // Fix any errors
  if(!isalpha(grid[0]))grid[0] = 'A';
  if(!isalpha(grid[1]))grid[1] = 'A';
  if(!isdigit(grid[2]))grid[2] = '0';
  if(!isdigit(grid[3]))grid[3] = '0';
  if(!isalpha(grid[4]))grid[4] = 'M';
  if(!isalpha(grid[5]))grid[5] = 'M';
  

  *dlong = 20*(grid[0] - 'A') - 180;
  *dlat = 10*(grid[1] - 'A') - 90;
  *dlong += (grid[2] - '0') * 2;
  *dlat += (grid[3] - '0');
  
  // subsquares
  *dlong += (grid[4] - 'A') * 5/60.;
  *dlat += (grid[5] - 'A') * 2.5/60.;
  *dlong += 2.5/60.;
  *dlat += 1.25/60;
}
#endif



/////////////////////////////////////////////////////////
//////////////        D C O O R D         ///////////////
/////////////////////////////////////////////////////////
// In WSJT this is used in various places to do coordinate
// system conversions but moon2 only uses it once.
void DCOORD(double xA0,double xB0,double AP,double BP,
              double xA1,double xB1,double *xA2,double *B2)
{
  double TA2O2;
  double SB0,CB0,SBP,CBP,SB1,CB1,SB2,CB2;
  double SAA,CAA,SBB,CBB,CA2,SA2;
  
  SB0 = sin(xB0);
  CB0 = cos(xB0);
  SBP = sin(BP);
  CBP = cos(BP);
  SB1 = sin(xB1);
  CB1 = cos(xB1);
  SB2 = SBP*SB1 + CBP*CB1*cos(AP-xA1);
  CB2 = sqrt(1.-(SB2*SB2));
  *B2 = atan(SB2/CB2);
  SAA = sin(AP-xA1)*CB1/CB2;
  CAA = (SB1-SB2*SBP)/(CB2*CBP);
  CBB = SB0/CBP;
  SBB = sin(AP-xA0)*CB0;
  SA2 = SAA*CBB-CAA*SBB;
  CA2 = CAA*CBB+SAA*SBB;
  TA2O2 = 0.0;
  if(CA2 <= 0.) TA2O2 = (1.-CA2)/SA2;
  if(CA2 > 0.) TA2O2 = SA2/(1.+CA2);
  *xA2=2.*atan(TA2O2);
  if(*xA2 < 0.) *xA2 = *xA2+6.2831853071795864; 
}


/////////////////////////////////////////////////////////
////////////////        M O O N 2        ////////////////
/////////////////////////////////////////////////////////

// You can derive the lat/long from the grid square using
// the grid2deg function to translate from grid to lat/long
// Example call to this function for 2014/01/04 1709Z: 
//   moon2(2014,1,4,17+9/60.,-106.625,52.104168,
//       &RA, &Dec, &topRA, &topDec, &LST, &HA, &Az, &El, &dist);
// I have not used or checked any of the outputs other than Az and El
void moon2(int y,int m,int Day,
        double UT,
        double lon,double lat,
        double *RA,double *Dec,
        double *topRA,double *topDec,
        double *LST,double *HA,
        double *Az,double *El,double *dist)
{
  // The strange position of some of the semicolons is because some
  // of this was translated using a TCL script that I wrote - it isn't
  // too smart but it saved some typing
  double NN                           ;//Longitude of ascending node
  double i                            ;//Inclination to the ecliptic
  double w                            ;//Argument of perigee
  double a                            ;//Semi-major axis
  double e                            ;//Eccentricity
  double MM                           ;//Mean anomaly

  double v                            ;//True anomaly
  double EE                           ;//Eccentric anomaly
  double ecl                          ;//Obliquity of the ecliptic

  double d                            ;//Ephemeris time argument in days
  double r                            ;//Distance to sun, AU
  double xv,yv                        ;//x and y coords in ecliptic
  double lonecl,latecl                ;//Ecliptic long and lat of moon
  double xg,yg,zg                     ;//Ecliptic rectangular coords
  double Ms                           ;//Mean anomaly of sun
  double ws                           ;//Argument of perihelion of sun
  double Ls                           ;//Mean longitude of sun (Ns=0)
  double Lm                           ;//Mean longitude of moon
  double DD                           ;//Mean elongation of moon
  double FF                           ;//Argument of latitude for moon
  double xe,ye,ze                     ;//Equatorial geocentric coords of moon
  double mpar                         ;//Parallax of moon (r_E / d)
  //  double lat,lon                      ;//Station coordinates on earth
  double gclat                        ;//Geocentric latitude
  double rho                          ;//Earth radius factor
  double GMST0; //,LST,HA;
  double g;
  //  double topRA,topDec                 ;//Topocentric coordinates of Moon
  //  double Az,El;
  //  double dist;

  double rad = 57.2957795131,twopi = 6.283185307,pi,pio2;
  //      data rad/57.2957795131d0/,twopi/6.283185307d0/

  //Note the use of 'L' to force 32-bit integer arithmetic here. 
  d=367L*y - 7L*(y+(m+9L)/12L)/4L + 275L*m/9L + Day - 730530L + UT/24.;
  ecl = 23.4393 - 3.563e-7 * d;

//Serial.print("d = ");
//Serial.println(d,3);

  //  Orbital elements for Moon:
  NN = 125.1228 - 0.0529538083 * d;
  i = 5.1454;
  w = fmod(318.0634 + 0.1643573223 * d + 360000.,360.);
  a = 60.2666;
  e = 0.054900;
  MM = fmod(115.3654 + 13.0649929509 * d + 360000.,360.);

  // Orbital elements for Sun:
  /*
  NN = 0.0;
  i = 0.0;
  w = fmod(282.9404 + 4.70935e-5 * d + 360000.,360.);
  a = 1.000000;
  e = 0.016709 - 1.151e-9 * d;
  MM = fmod(356.0470 + 0.99856002585 * d + 360000.,360.);
  */

  /*
  Serial.print("\nmoon2: w=");
  Serial.print(w,3);
  Serial.print(" e=");
  Serial.print(e,3);  
  Serial.print(" MM=");
  Serial.println(MM,3);
  */

  EE = MM + e*rad*sin(MM/rad) * (1. + e*cos(MM/rad));
  EE = EE - (EE - e*rad*sin(EE/rad)-MM) / (1. - e*cos(EE/rad));
  EE = EE - (EE - e*rad*sin(EE/rad)-MM) / (1. - e*cos(EE/rad));

  xv = a * (cos(EE/rad) - e);
  yv = a * (sqrt(1.-e*e) * sin(EE/rad));

  v = fmod(rad*atan2(yv,xv)+720.,360.);
  r = sqrt(xv*xv + yv*yv);

  //  Get geocentric position in ecliptic rectangular coordinates:

  xg = r * (cos(NN/rad)*cos((v+w)/rad) - sin(NN/rad)*sin((v+w)/rad)*cos(i/rad));
  yg = r * (sin(NN/rad)*cos((v+w)/rad) + cos(NN/rad)*sin((v+w)/rad)*cos(i/rad));
  zg = r * (sin((v+w)/rad)*sin(i/rad));

  //  Ecliptic longitude and latitude of moon:
  lonecl = fmod(rad*atan2(yg/rad,xg/rad)+720.,360.);
  latecl = rad * atan2(zg/rad,sqrt(xg*xg + yg*yg)/rad);

  //  Now include orbital perturbations:
  Ms = fmod(356.0470 + 0.9856002585 * d + 3600000.,360.);
  ws = 282.9404 + 4.70935e-5*d;
  Ls = fmod(Ms + ws + 720.,360.);
  Lm = fmod(MM + w + NN+720.,360.);
  DD = fmod(Lm - Ls + 360.,360.);
  FF = fmod(Lm - NN + 360.,360.);

  lonecl = lonecl
       -1.274 * sin((MM-2.*DD)/rad)
       +0.658 * sin(2.*DD/rad)
       -0.186 * sin(Ms/rad)
       -0.059 * sin((2.*MM-2.*DD)/rad)
       -0.057 * sin((MM-2.*DD+Ms)/rad)
       +0.053 * sin((MM+2.*DD)/rad)
       +0.046 * sin((2.*DD-Ms)/rad)
       +0.041 * sin((MM-Ms)/rad)
       -0.035 * sin(DD/rad)
       -0.031 * sin((MM+Ms)/rad)
       -0.015 * sin((2.*FF-2.*DD)/rad)
       +0.011 * sin((MM-4.*DD)/rad);

  latecl = latecl
       -0.173 * sin((FF-2.*DD)/rad)
       -0.055 * sin((MM-FF-2.*DD)/rad)
       -0.046 * sin((MM+FF-2.*DD)/rad)
       +0.033 * sin((FF+2.*DD)/rad)
       +0.017 * sin((2.*MM+FF)/rad);

  r = 60.36298
       - 3.27746*cos(MM/rad)
       - 0.57994*cos((MM-2.*DD)/rad)
       - 0.46357*cos(2.*DD/rad)
       - 0.08904*cos(2.*MM/rad)
       + 0.03865*cos((2.*MM-2.*DD)/rad)
       - 0.03237*cos((2.*DD-Ms)/rad)
       - 0.02688*cos((MM+2.*DD)/rad)
       - 0.02358*cos((MM-2.*DD+Ms)/rad)
       - 0.02030*cos((MM-Ms)/rad)
       + 0.01719*cos(DD/rad)
       + 0.01671*cos((MM+Ms)/rad);

  *dist = r * 6378.140;

  //  Geocentric coordinates:
  //  Rectangular ecliptic coordinates of the moon:

  xg = r * cos(lonecl/rad)*cos(latecl/rad);
  yg = r * sin(lonecl/rad)*cos(latecl/rad);
  zg = r *                 sin(latecl/rad);

  //  Rectangular equatorial coordinates of the moon:
  xe = xg;
  ye = yg * cos(ecl/rad) - zg*sin(ecl/rad);
  ze = yg * sin(ecl/rad) + zg*cos(ecl/rad);

  //  Right Ascension, Declination:
  *RA = fmod(rad*atan2(ye,xe)+360.,360.);
  *Dec = rad * atan2(ze,sqrt(xe*xe + ye*ye));

  //  Now convert to topocentric system:
  mpar=rad * asin(1./r);
  //      alt_topoc = alt_geoc - mpar*cos(alt_geoc);
  gclat = lat - 0.1924 * sin(2.*lat/rad);
  rho = 0.99883 + 0.00167 * cos(2.*lat/rad);
  GMST0 = (Ls + 180.)/15.;
  *LST = fmod(GMST0+UT+lon/15.+48.,24.)    ;//LST in hours

  *HA = 15. * *LST - *RA                          ;//HA in degrees
  g = rad*atan(tan(gclat/rad)/cos(*HA/rad));
  *topRA = *RA - mpar*rho*cos(gclat/rad)*sin(*HA/rad)/cos(*Dec/rad);
  *topDec = *Dec - mpar*rho*sin(gclat/rad)*sin((g-*Dec)/rad)/sin(g/rad);

  *HA = 15. * *LST - *topRA                       ;//HA in degrees
  if(*HA > 180.) *HA=*HA-360.;
  if(*HA < -180.) *HA=*HA+360.;

  pi = 0.5 * twopi;
  pio2 = 0.5 * pi;
  DCOORD(pi,pio2-lat/rad,0.,lat/rad,*HA*twopi/360,*topDec/rad,Az,El);
  *Az = *Az * rad;
  *El = *El * rad;

  return;
}
