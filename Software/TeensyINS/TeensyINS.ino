/*
 * Teensy-based INS by Husayn Kartodirdjo
 * Sensors: GY-91 (MPU9250 and BMP280) and Adafruit Ultimate GPS Board v3
 * North-East-Down reference frame for Inertial Reference Frame
 * Roll-Pitch-Yaw reference frame for body motions
 */

#include <MPU9250_asukiaaa.h> // IMU
#include <Adafruit_BMP280.h> // Barometer
#include <BasicLinearAlgebra.h> 
using namespace BLA;
#include <TinyGPS.h>
#include <MadgwickAHRS.h>
#include <Chrono.h>

// Definitions
#define HWSERIAL Serial1
#define MICROS 1000000
// GPS Module Update Rate Commands
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F" ///<  1 Hz
#define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C"  ///<  5 Hz
#define PMTK_API_SET_FIX_CTL_1HZ "$PMTK300,1000,0,0,0,0*1C" ///< 1 Hz
#define PMTK_API_SET_FIX_CTL_5HZ "$PMTK300,200,0,0,0,0*2F"  ///< 5 Hz

// Global Variables
double cosInitialLat = 49.20689; // Vancouver
bool newGPSData = false;
double radiusEarth = 6371000; // metres
const int predictRate = 50; // Hz
const int updateRate = 1; // Hz
const int debugRate = 5; // Hz
unsigned long microsPerFilter = MICROS/predictRate;
unsigned long microsPerUpdate = MICROS/updateRate;
unsigned long microsPerDebug = MICROS/debugRate;
double deltaTime = 1.0/(double)predictRate; // seconds
double aR, aP, aY;
double gR, gP, gY;
double mR, mP, mY;
double q0, q1, q2, q3; // normalized rotation quaternion provided by the Madgwick AHRS
float flat, flon; unsigned long age; // variables needed for TinyGPS

const double localAirPressure = (double) 1013.25; // in hPa - must update to local conditions before use!

// Unit Conversions
const double DEG2RAD = PI / 180.0;
const double RAD2DEG = 180.0 / 3.14159;
const double KPH2MPS = 1.0/3.6;
const double Gs2SI = 9.81;

// Initialize Objects
Madgwick filter;
TinyGPS GPS;
MPU9250_asukiaaa MPU;
Adafruit_BMP280 BMP;
Chrono chronoFilter(Chrono::MICROS);
Chrono chronoUpdate(Chrono::MICROS);
Chrono chronoDebug(Chrono::MICROS);

// Matrices
#define NUM_STATES 6
#define NUM_OBS_GPS 3
#define NUM_OBS_BARO 1
#define NUM_COM 3

BLA::Matrix<NUM_STATES, 1, Array<NUM_STATES,1,double> > x; // posE posN posD velE velN velD
  auto posEND = x.Submatrix<3, 1>(0, 0);
  auto velEND = x.Submatrix<3, 1>(3, 0);
BLA::Matrix<NUM_COM, 1, Array<NUM_COM,1,double> > u; // accE accN accD
BLA::Matrix<NUM_COM, 1, Array<NUM_COM,1,double> > accRPY;
BLA::Matrix<NUM_OBS_GPS, 1, Array<NUM_OBS_GPS,1,double> > zGPS; // posE, posN, posD
BLA::Matrix<NUM_OBS_BARO,1, Array<NUM_OBS_BARO,1,double> > zBaro; // posD
BLA::Matrix<NUM_STATES, NUM_STATES, Array<NUM_STATES,NUM_STATES,double> > f = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                               0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0,                                                                               
                                                                               0.0, 0.0, 0.0, 0.0, 0.0, 1.0};                                                                               
BLA::Matrix<NUM_STATES, NUM_COM, Array<NUM_STATES,NUM_COM,double> > b;
BLA::Matrix<NUM_STATES, NUM_STATES, Array<NUM_STATES,NUM_STATES,double> > Q = {8.0, 0.0, 0.0, 0.0, 0.0,
                                                                               0.0, 8.0, 0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 5.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0, 5.0, 0.0,
                                                                               0.0, 0.0, 0.0, 0.0, 5.0};// model covariance
BLA::Matrix<NUM_STATES, NUM_STATES, Array<NUM_STATES,NUM_STATES,double> > p;
BLA::Identity<NUM_STATES, NUM_STATES> I;

void setup() {
  // Initiate Connections
  Serial.begin(115200); // For debugging
  HWSERIAL.begin(9600); // For GPS
  Wire.begin(); // For GY-91
  
  // Initialize Objects
  MPU.beginAccel();
  MPU.beginGyro();
  MPU.beginMag();

  // Boost GPS Update Rate to 5Hz
  // HWSERIAL.println(PMTK_SET_NMEA_UPDATE_5HZ);
  // HWSERIAL.println(PMTK_API_SET_FIX_CTL_5HZ);

  filter.begin(predictRate);

  x.Fill(0);
  p.Fill(0);
  b.Fill(0);

  cosInitialLat = cos(cosInitialLat*DEG2RAD);
}

void loop() {
  if (chronoFilter.hasPassed(microsPerFilter,true)) {
    predictKalman();
  }
  if (chronoUpdate.hasPassed(microsPerUpdate,true)) {
    if (newGPSData) {
      newGPSData = false;  
    }
  }
  if (chronoDebug.hasPassed(microsPerDebug,true)) {
    // Predict debugging
    Serial.println( String(x(0),2) + "," + String(x(1),2) + "," + String(x(2),2) + "," + String(x(3),2) + "," + String(x(4),2) ); 
  }
  if (HWSERIAL.available()) {
    char c = HWSERIAL.read();
    if(GPS.encode(c)) {
      newGPSData = true;
    }
  }
}

void predictKalman() {
  // read raw data from IMU into RPY coordinate system
  MPU.accelUpdate();
  MPU.gyroUpdate();  
  MPU.magUpdate();

  // Convert to Body Coordinates and correct units
  ax = MPU.accelY()*Gs2SI; // ROLL
  ay = MPU.accelX()*Gs2SI; // PITCH
  az = -MPU.accelZ()*Gs2SI;// YAW
  gx = MPU.gyroY();
  gy = MPU.gyroX();
  gz = -MPU.gyroZ();
  mx = MPU.magX();
  my = MPU.magY();
  mz = MPU.magZ();
  
  // update the filter, which computes orientation
  // filter.updateIMU(-gx, -gy, -gz, ax, ay, az);
  filter.update(-gx, -gy, -gz, ax, ay, az, mx, my, mz);

  q0 = (double) filter.q0;
  q1 = (double) filter.q1;
  q2 = (double) filter.q2;
  q3 = (double) filter.q3;
     
  accRPY(0) = ax;
  accRPY(1) = ay;
  accRPY(2) = az;

  // Transform accRPY to accEND
  

  
  // update F matrix
  f(0,3) = deltaTime;
  f(1,4) = deltaTime;
  f(2,5) = deltaTime;

  // update B matrix
  b(0,0) = 0.5*pow(deltaTime,2);
  b(1,1) = b(0,0);
  b(2,2) = b(0,0);
  b(3,0) = deltaTime;
  b(4,1) = deltaTime;
  b(5,2) = deltaTime;

  x = f*x + b*u;
  p = f*p*~f + Q;
}

void updateKalmanBaro() {
  
}

void updateKalmanGPS() {

}

void getBaroObservations() {
  zBaro(0) = -BMP.readAltitude(localAirPressure);
}

void getGPSObservations() {
  // Uses equirectangular projection based off of https://stackoverflow.com/a/16271669
  GPS.f_get_position(&flat, &flon, &age);
  double dlat = (double) flat;
  double dlon = (double) flon;
  zGPS(0) = radiusEarth*dlon*DEG2RAD*cosInitialLat; // E in NED
  zGPS(1) = radiusEarth*dlat*DEG2RAD; // N in NED
  zGPS(2) = (double) GPS.f_altitude();
}
