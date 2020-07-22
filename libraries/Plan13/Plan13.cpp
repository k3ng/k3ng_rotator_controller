#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Plan13.h"




/*
 * Convert degrees to radians
 */

double Plan13::rad(double deg)
{
  return (M_PI / 180.0 * deg);
}

double Plan13::deg(double rad)
{
  return (rad * 180.0 / M_PI);
}

double Plan13::FNatn(double y, double x)
{
  float a;
  if (x != 0)
  {
     a = atan(y / x);
  }
  else
  {
     a = M_PI / 2.0 * sin(y);
  }
  if (x < 0)
  {
     a = a + M_PI;
  }
  if (a < 0)
  {
     a = a + 2.0 * M_PI;
  }
  return (a);
}

/** Convert date to day number
 *
 * Function returns a general day number from year, month, and day.
 * Value is (JulianDay - 1721409.5) or (AmsatDay + 722100)
 *
**/
double Plan13::FNday(int year, int month, int day)
{
  double JulianDate;

  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.print("## FNDay:  ");
    Serial.print("Year: ");
    Serial.print(year);
    Serial.print(" Month: ");
    Serial.print(month);
    Serial.print(" Day: ");
    Serial.print(day);
  #endif

  if (month <= 2)
  {
     year -= 1;
     month += 12;
  }
  
 
  JulianDate = (long)(year * YM) + (int)((month + 1) * 30.6) + (day - 428);

  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.print(" JD: ");
    Serial.println(JulianDate);
  #endif

  return (JulianDate);
}


void Plan13::initSat(void)
{
  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.println("Start initSat()");
  #endif

  /* Observer's location */
  LA = rad( observer_lat );
  LO = rad( observer_lon );
  HT = ((float) observer_height)/1000.0; // this needs to be in km
  CL = cos(LA);
  SL = sin(LA);
  CO = cos(LO);
  SO = sin(LO);
  /* WGS-84 Earth Ellipsoid */
  RE = 6378.137;
  FL = 1.0 / 298.257224;

  /* IAU-76 Earth Ellipsoid */
  // RE = 6378.140;
  // FL = 1.0 / 298.257;

  RP = RE * (1.0 - FL);
  XX = RE * RE;
  ZZ = RP * RP;

  D = sqrt( XX * CL * CL + ZZ * SL * SL );

  Rx = XX / D + HT;
  Rz = ZZ / D + HT;

  /* Observer's unit vectors Up EAST and NORTH in geocentric coordinates */
  Ux = CL * CO;
  Ex = -SO;
  Nx = -SL * CO;

  Uy = CL * SO;
  Ey = CO;
  Ny = -SL * SO;

  Uz = SL;
  Ez = 0;
  Nz = CL;

  /* Observer's XYZ coordinates at earth's surface */
  Ox = Rx * Ux;
  Oy = Rx * Uy;
  Oz = Rz * Uz;

  /* Convert angles to radians, etc. */
  RA = rad(RA);
  IN = rad(IN);
  WP = rad(WP);
  MA = rad(MA);
  MM = MM * 2.0 * M_PI;
  M2 = M2 * 2.0 * M_PI;

  //  YM = 365.25;           /* Mean year, days                           */
  //  YT = 365.2421970;      /* Tropical year, days                       */
  WW = 2.0 * M_PI / YT;      /* Earth's rotation rate, rads/whole day     */
  WE = 2.0 * M_PI + WW;      /* Earth's rotation rate, rads/day           */
  W0 = WE / 86400;           /* Earth's rotation rate, rads/sec           */

  /* Observer's velocity, geocentric coordinates */
  VOx = -Oy * W0;
  VOy = Ox * W0;

  /* Convert satellite epoch to Day No. and fraction of a day */
  DE = FNday(YE, 1, 0) + (int)TE;
  
  TE = TE - (int)TE;

  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.print("DE: ");
    Serial.println(DE);
    Serial.print("TE: ");
    Serial.println(TE);
  #endif

  /* Average Precession rates */
  GM = 3.986E5;               /* Earth's gravitational constant km^3/s^2   */
  J2 = 1.08263E-3;            /* 2nd Zonal coeff, Earth's gravity Field    */
  N0 = MM / 86400.0;                    /* Mean motion rads/s                */
  A0 = pow(GM / N0 / N0, 1.0 / 3.0);  /* Semi major axis km                */
  b0 = A0 * sqrt(1.0 - EC * EC);      /* Semi minor axis km                */
  SI = sin(IN);
  CI = cos(IN);
  PC = RE * A0 / (b0 * b0);
  PC = 1.5 * J2 * PC * PC * MM;       /* Precession const, rad/day         */
  QD = -PC * CI;                      /* Node Precession rate, rad/day     */
  WD = PC *(5.0 * CI*CI - 1.0) / 2.0; /* Perigee Precession rate, rad/day  */
  DC = -2.0 * M2 / MM / 3.0;          /* Drag coeff                        */

   /* Sideral and solar data. Never needs changing. Valid to year 2000+ */


               /* GHAA, Year YG, Jan 0.0 */
 YG = 2010;
 G0 = 99.5578;
  /* MA Sun and rate, deg, deg/day */
 MAS0 = 356.4485;
 MASD = 0.98560028;
  /* Sun's inclination */
 INS = rad(23.4380);
 CNS = cos(INS);
 SNS = sin(INS);
  /* Sun's equation of center terms */
 EQC1 = 0.03341;
 EQC2 = 0.00035;


   /* Bring Sun data to satellite epoch */
  TEG = (DE - FNday(YG, 1, 0)) + TE;  /* Elapsed Time: Epoch - YG          */
  GHAE = rad(G0) + TEG * WE;          /* GHA Aries, epoch                  */
  MRSE = rad(G0) + (TEG * WW) + M_PI;   /* Mean RA Sun at Sat Epoch          */
  MASE = rad(MAS0 + MASD * TEG);      /* Mean MA Sun                       */

   /* Antenna unit vector in orbit plane coordinates */
  CO = cos(rad(ALON));
  SO = sin(rad(ALON));
  CL = cos(rad(ALAT));
  SL = sin(rad(ALAT));
  ax = -CL * CO;
  ay = -CL * SO;
  az = -SL;

   /* Miscellaneous */
  OLDRN = -99999;
  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.println("End initSat()");
  #endif
}

/**
 * Calculate satellite position at DN, TN
**/
void Plan13::satvec(void)
{
  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.println("Start satvec()");
  #endif

  T = (DN - DE) + (TN - TE);//83.848 ;          /* Elapsed T since epoch             */

  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.print("T: ");
    Serial.println(T);
  #endif

  DT = DC * T / 2.0;                  /* Linear drag terms                 */
  KD = 1.0 + 4.0 * DT;
  KDP = 1.0 - 7.0 * DT;
  M = MA + MM * T * (1.0 - 3.0 * DT); /* Mean anomaly at YR,/ TN           */
  DR = (int)(M / (2.0 * M_PI));         /* Strip out whole no of revs        */
  M = M - DR * 2.0 * M_PI;              /* M now in range 0 - 2PI            */
  RN = RV + DR + 1;                   /* Current orbit number              */

   /* Solve M = EA - EC * sin(EA) for EA given M, by Newton's method        */
  EA = M;                             /* Initail solution                  */
  do
  {
     C = cos(EA);
     S = sin(EA);
     DNOM = 1.0 - EC * C;
     D = (EA - EC * S - M) / DNOM; /* Change EA to better resolution    */
     EA = EA - D;                    /* by this amount until converged    */
  }
  while (fabs(D) > 1.0E-5);

   /* Distances */
  A = A0 * KD;
  B = b0 * KD;
  RS = A * DNOM;

   /* Calculate satellite position and velocity in plane of ellipse */
  Sx = A * (C - EC);
  Vx = -A * S / DNOM * N0;
  Sy = B * S;
  Vy = B * C / DNOM * N0;

  AP = WP + WD * T * KDP;
  CWw = cos(AP);
  SW = sin(AP);
  RAAN = RA + QD * T * KDP;
  CQ = cos(RAAN);
  SQ = sin(RAAN);

   /* Plane -> celestial coordinate transformation, [C] = [RAAN]*[IN]*[AP] */
  CXx = CWw * CQ - SW * CI * SQ;
  CXy = -SW * CQ - CWw * CI * SQ;
  CXz = SI * SQ;
  CYx = CWw * SQ + SW * CI * CQ;
  CYy = -SW * SQ + CWw * CI * CQ;
  CYz = -SI * CQ;
  CZx = SW * SI;
  CZy = CWw * SI;
  CZz = CI;

   /* Compute satellite's position vector, ANTenna axis unit vector    */
   /*   and velocity  in celestial coordinates. (Note: Sz = 0, Vz = 0) */
  SATx = Sx * CXx + Sy * CXy;
  ANTx = ax * CXx + ay * CXy + az * CXz;
  VELx = Vx * CXx + Vy * CXy;
  SATy = Sx * CYx + Sy * CYy;
  ANTy = ax * CYx + ay * CYy + az * CYz;
  VELy = Vx * CYx + Vy * CYy;
  SATz = Sx * CZx + Sy * CZy;
  ANTz = ax * CZx + ay * CZy + az * CZz;
  VELz = Vx * CZx + Vy * CZy;

   /* Also express SAT, ANT, and VEL in geocentric coordinates */
  GHAA = GHAE + WE * T;       /* GHA Aries at elaprsed time T */
  C = cos(-GHAA);
  S = sin(-GHAA);
  Sx = SATx * C - SATy * S;
  Ax = ANTx * C - ANTy * S;
  Vx = VELx * C - VELy * S;
  Sy = SATx * S + SATy * C;
  Ay = ANTx * S + ANTy * C;
  Vy = VELx * S + VELy * C;
  Sz = SATz;
  Az = ANTz;
  Vz = VELz;

  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.println("End satvec()");
  #endif
}

/*
 * Compute and manipulate range/velocity/antenna vectors
 */
void Plan13::rangevec(void)
{
  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.println("Start rangevec()");
  #endif
  
  /* Range vector = sat vector - observer vector */
  Rx = Sx - Ox;
  Ry = Sy - Oy;
  Rz = Sz - Oz;

  R = sqrt(Rx * Rx + Ry * Ry + Rz * Rz);    /* Range Magnitute */

  /* Normalize range vector */
  Rx = Rx / R;
  Ry = Ry / R;
  Rz = Rz / R;
  U = Rx * Ux + Ry * Uy + Rz * Uz;
  E = Rx * Ex + Ry * Ey;
  N = Rx * Nx + Ry * Ny + Rz * Nz;

  AZ_P13 = deg(FNatn(E, N));
  EL_P13 = deg(asin(U));

  // Solve antenna vector along unit range vector, -r.a = cos(SQ)
  //  SQ = deg(acos(-(Ax * Rx + Ay * Ry + Az * Rz)));
  // Calculate sub-satellite Lat/Lon 
  SLON = deg(FNatn(Sy, Sx));   /* Lon, + East  */
  SLAT = deg(asin(Sz / RS));   /* Lat, + North */

   /* Resolve Sat-Obs velocity vector along unit range vector. (VOz = 0) */
  RR = (Vx - VOx) * Rx + (Vy - VOy) * Ry + Vz * Rz; /* Range rate, km/sec */
  //FR = rxFrequency * (1 - RR / 299792);

  dopplerFactor = RR / 299792.0;
  int rxDoppler = getDoppler(rxFrequencyLong);
  int txDoppler = getDoppler(txFrequencyLong);
  rxOutLong = rxFrequencyLong - rxDoppler;
  txOutLong = txFrequencyLong + txDoppler;


  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.println("End rangevec()");
  #endif
}

void Plan13::sunvec(void)
{
	//TODO
}


int Plan13::getDoppler(unsigned long freq) {
  freq = (freq + 50000L) / 100000L;
  long factor = dopplerFactor * 1E11;
  int digit;
  float tally = 0;
  for (int x = 4; x > -1; x--) {
    digit = freq/pow(10,x);
    long bare = digit * pow(10,x);
    freq = freq - bare;
    float inBetween =  factor * (float(bare) / 1E6);
    tally += inBetween;
}
return int( tally + .5); //round
}

int Plan13::getDoppler64(unsigned long freq) {
	long factor = dopplerFactor * 1E11;
	uint64_t doppler_sixfour = freq * dopplerFactor;
	return (int) doppler_sixfour/1E11;
}


void Plan13::printdata(void)
{
  Serial.print("AZ:");
  Serial.print(AZ_P13);
  Serial.print(" EL:");
  Serial.print(EL_P13);
  Serial.print(" RX:");
  Serial.print(rxOutLong);
  Serial.print(" TX:");
  Serial.print(txOutLong);
  Serial.print(" Sat_Lat:");
  Serial.print(SLAT);
  Serial.print(" Sat_Lon:");
  Serial.print(SLON);
  Serial.print(" RR:");
  Serial.print(RR);

/** Setter method for uplink (tx) and downlink (rx) frequencies in Hz.
These data need not be set if the doppler shifted frequencies are not needed.

\param rxFrequncy_in the downlink frequency
\param txFrequency_in the uplink frequency
**/
}
void Plan13::setFrequency(unsigned long rxFrequency_in, unsigned long txFrequency_in) {

	rxFrequencyLong = rxFrequency_in;
	txFrequencyLong = txFrequency_in;

	if (TEST) {
		rxFrequencyLong = 435300000L;
		txFrequencyLong = 45920000L;
	}
}

/** Setter method for indicating the location of the ground station. 
This and setElements() must be done before the calculate method is applied for the first time.
Thereafter, however, it doesn't matter unless the groundstation changes position.

\param observer_lon_in the longitude of the observer, with east positive and west negative
\param observer_lat_in the latitude of the observer with north positive and south negative
\param height height of the observer, in meters
**/
void Plan13::setLocation(double observer_lon_in, double observer_lat_in, int height) {
	observer_lon = observer_lon_in;//-64.375; //0.06; // lon east is positive, west is negative
	observer_lat = observer_lat_in;//45.8958; //52.21; //Cambridge UK
	observer_height = height; //60m height in meters
	if (TEST) {
		observer_lon = -64.375;
		observer_lat = 45.8958;
		observer_height = 60;
	}
}

/**
Setter method for UTC time at which the satellite is to be observed. 
This is usually the current tim.
\param yearIn The current year in four digits, like 2010
\param  monthIn The month in two digits, like '10' for October
\param mDayIn the day of the month, like '1' for the first day
\param hourIn the hour of the day, like '19' for 19
\param minIn the minute of the day, like '00' for the first
\param secIn the seconds
**/
void Plan13::setTime(int yearIn, int monthIn, int mDayIn, int hourIn, int minIn, int secIn) {

  #if defined(DEBUG_PLAN13_LIBRARY)
    Serial.println("Start setTime()");
  #endif

    int aYear = yearIn;
    int aMonth = monthIn;
    int aMday = mDayIn;
    int aHour = hourIn;
    int aMin  = minIn;
    int aSec  = secIn;

    if (TEST) {
    aYear = 2009; 
    aMonth = 10;
    aMday = 1; 
     aHour = 19; 
     aMin = 5;
    aSec = 0;
    }

    DN = FNday(aYear, aMonth, aMday);
    TN = ((float)aHour + ((float)aMin + ((float)aSec/60.0)) /60.0)/24.0;
    DN = (long)DN;

    #if defined(DEBUG_PLAN13_LIBRARY)
      Serial.print(aYear);
      Serial.print("/");
      Serial.print(aMonth);
      Serial.print("/");
      Serial.print(aMday);
      Serial.print(" ");
      Serial.print(aHour);
      Serial.print(":");
      Serial.print(aMin);
      Serial.print(":");
      Serial.print(aSec);
      Serial.print(" ");
      Serial.print("DN: ");
      Serial.println(DN);
      Serial.print("TN: ");
      Serial.println(TN);
      Serial.println("End setTime()");
    #endif
}
/*******************
* Sets the keplerian elements for the following calculations. If the TEST variable
is defined 'true', some sample elements will be applied.

\param YE_in the YE element
\param TE_in the TE element
\param IN_in the IN element
\param RA_in the RA element
\param EC_in the EC element
\param WP_in the WP element
\param MA_in the MA element
\param MM_in the MM element
\param M2_in the M2 element
\param RV_in the RV element
\param ALON_in the ALON element
*******/

 void   Plan13::setElements(double YE_in, double TE_in, double IN_in, double 
         RA_in, double EC_in, double WP_in, double MA_in, double MM_in, 
         double M2_in, double RV_in, double ALON_in ) {
	
			YE = YE_in;
			TE = TE_in;
			IN = IN_in;
			RA = RA_in;
			EC = EC_in;
			WP = WP_in;
			MA = MA_in;
			MM = MM_in;
			M2 = M2_in;
			RV = RV_in;
			ALON = ALON_in;
			 
	  if (TEST) {// sample elements for AO-51
	    YE = 1997.0;
        TE = 126.71404377;
        IN = 98.5440;
        RA = 201.9624;
        EC = 0.0009615;
        WP = 356.5498;
        MA = 3.5611;
        MM = 14.27977735;
        M2 = 2.500E-07;
        RV = 18818;
        ALON = 180.0;

  }

}
/**
A function that joins together the necessary functions for calculating 
the satellite position. You must set the keplerian elements, time and observer lat/long
before using this.
**/
  void  Plan13::calculate() {
		initSat();
		satvec();
		rangevec();
   }

 void Plan13::footprintOctagon(float *points, float SLATin, float SLONin, float REin, float RSin) {
	//static float points[16];
	Serial.print("SLAT: ");
	Serial.print(SLATin);
	Serial.print(", SLON: ");
	Serial.print(SLONin);
	Serial.print(", RE: ");
	Serial.print(REin);
	Serial.print(", RS: ");
	Serial.println(RSin);
	float srad = acos(REin/RSin); // Beta in Davidoff diag. 13.2, this is in rad
	Serial.print("srad: ");
	Serial.println(srad);
	float cla= cos(rad(SLATin));
	float sla = sin(rad(SLATin));
	float clo = cos(rad(SLONin));
	float slo = sin(rad(SLONin));
	float sra = sin(srad);
	float cra = cos(srad);
	for (int i = 0; i < 16; i = i +2) {
		float a = 2 * M_PI * i / 16;
		Serial.print("\ta: ");
		Serial.println(a);
		float X = cra;
		Serial.print("\t first X: ");
		Serial.println(X);
		Serial.print("\t first Y: ");
		float Y = sra*sin(a);
		Serial.println(Y);
		float Z = sra*cos(a);
		Serial.print("\t first Z: ");
		Serial.println(Z);
		float x = X*cla - Z*sla;
		float y = Y;
		float z = X*sla + Z*cla;
		X = x*clo - y*slo;
		Serial.print("\tX: ");
		Serial.println(X);
		Y = x*slo + y*clo;
		Serial.print("\tY: ");
		Serial.println(Y);
		Z = z; 
		Serial.print("\tZ: ");
		Serial.println(Z);
		points[i] = deg(FNatn(Y,X));
		Serial.print("\t Long: ");
		Serial.print(points[i]);
		points[i+1] = deg(asin(Z));
		Serial.print("\t Lat: ");
		Serial.println(points[i+1]);
	}
}

