 /*
 * Teensy-based GPS/INS by Husayn Kartodirdjo
 * Loosely coupled MadgwickAHRS for orientation and Kalman Filter for position and velocity
 * Sensors: GY-91 (MPU9250 and BMP280) and Adafruit Ultimate GPS Board v3
 * North-East-Down reference frame for Inertial Reference Frame
 * Roll-Pitch-Yaw reference frame for body motions
 */

#include <MPU9250_asukiaaa.h>   // MPU9250
#include <Adafruit_BMP280.h>    // Barometer
#include <BasicLinearAlgebra.h> // Matrix Operations
  using namespace BLA;
#include <Adafruit_GPS.h>       //GPS
#include <MadgwickAHRS.h>       // AHRS
#include <Chrono.h>             // Timing
#include <BLAforQUAT.h>         // Quaternion Operations

// DEFINITIONS
#define HWSerial Serial1

// USER INPUTS
double cosInitialLat = 43.4675; // Oakville, ON
const int predictRate = 200; // Hz
const int updateRate = 10; // Hz
const int debugRate = 10; // Hz
const double localAirPressure = 1013.00; // in hPa - must update to local conditions before use!

// CONSTANTS
const double radiusEarth = 6371000; // metres
const unsigned long microsPerFilter = 1000000/predictRate;
const unsigned long microsPerUpdate = 1000000/updateRate;
const unsigned long microsPerDebug = 1000000/debugRate;
// Unit Conversions
const double DEG2RAD = PI / 180.0;
const double RAD2DEG = 180.0 / 3.14159;
const double KPH2MPS = 1.0/3.6;
const double Gs2SI = 9.81;
// Matrices
const int NUM_STATES = 6;
const int NUM_OBS_GPS = 3;
const int NUM_OBS_BARO = 1;
const int NUM_COM = 3;

// GLOBALS
int GPSLock;
bool newGPSData = false;
double deltaTime = 1.0/(double)predictRate; // seconds
double aR, aP, aY;
double gR, gP, gY;
double mR, mP, mY;
double q_0, q_1, q_2, q_3; // normalized rotation quaternion provided by the Madgwick AHRS
BLA::Matrix<NUM_STATES, 1, double > x; // posE posN posD velE velN velD
  auto posEND = x.Submatrix<3, 1>(0, 0);
  auto velEND = x.Submatrix<3, 1>(3, 0);
BLA::Matrix<NUM_COM, 1, double > u; // accE accN accD
BLA::Matrix<NUM_COM, 1, double > accRPY;
BLA::Matrix<NUM_COM, 1, double > gRPY;
BLA::Matrix<NUM_COM, 1, double > gNED = {0.0, 0.0, 9.51};
BLA::Matrix<NUM_OBS_GPS, 1, double > zGPS; // posE, posN, posD
BLA::Matrix<NUM_OBS_BARO,1, double > zBaro; // posD
BLA::Matrix<NUM_STATES, NUM_STATES, double >                              f = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                               0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                                                               0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
BLA::Matrix<NUM_STATES, NUM_COM, double > b;
BLA::Matrix<NUM_STATES, NUM_STATES, double >                              q = {8.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                               0.0, 8.2, 0.0, 0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 8.3, 0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0, 5.2, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0, 0.0, 5.3, 0.0,
                                                                               0.0, 0.0, 0.0, 0.0, 0.0, 5.3};// model covariance
BLA::Matrix<NUM_STATES, NUM_STATES, double > p;
BLA::Eye<NUM_STATES, NUM_STATES,double> I; // should be type Eye?

// Initialize Objects
Madgwick AHRSfilter;
MPU9250_asukiaaa MPU;
Adafruit_BMP280 BMP;
Chrono chronoFilter(Chrono::MICROS);
Chrono chronoUpdate(Chrono::MICROS);
Chrono chronoDebug(Chrono::MICROS);
Adafruit_GPS GPS(&HWSerial); // For GPS

void setup() {
  Serial.begin(115200); // For debugging
  
  GPS.begin(9600); 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  
  Wire.begin(); // For GY-91
  MPU.beginAccel();
  MPU.beginGyro();
  MPU.beginMag();
  
  BMP.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  /* Default settings from datasheet. */
  BMP.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  AHRSfilter.begin(predictRate);

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
      updateKalmanGPS();
      newGPSData = false;
      GPSLock = 1;  
    }
    else {
      GPSLock = 0;
      newGPSData = false;
    }
    updateKalmanBaro();
  }
  if (chronoDebug.hasPassed(microsPerDebug,true)) {
    // Predict debugging
    // Serial.println( String(x(0),2) + "," + String(x(1),2) + "," + String(x(2),2) ); // Position State
    Serial.println( String(x(3),2) + "," + String(x(4),2)+ "," + String(x(5),2) ); // Velocity State
    // Serial.println( String(x(0),2) + "," + String(x(1),2) + "," + String(x(2),2) + "," + String(x(3),2) + "," + String(x(4),2)+ "," + String(x(5),2) + "," + String(GPSLock) );
    // Serial.println(String(filter.getRoll(),2) + "," + String(filter.getPitch(),2) + "," + String(filter.getYaw(),2) ); // AHRS
    // Serial.println( String(u(0),2) + "," + String(u(1),2) + "," + String(u(2),2)); // Accelerometer Readings
    // Serial.println(zBaro(0)); // Barometer Reading
    // Serial.println( String(zGPS(0),2) + "," + String(zGPS(1),2)+ "," + String(zGPS(2),2) ); // GPS Readings
    // Serial.println( String(zBaro(0),2) + "," + String(zGPS(2),2) ); // Compare GPS and Barometer altitudes
    // Serial.println( String(p(0,0),2) + "," + String(p(1,1),2) + "," + String(p(2,2),2) + "," + String(p(3,3),2) + "," + String(p(4,4),2) + "," + String(p(5,5),2) );
    // Serial.println( String(q(1,1),2) );
  }
  if (HWSerial.available()) {
    char c = GPS.read();
    if(GPS.newNMEAreceived()) {
      newGPSData = true;
    }
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

void predictKalman() {
  // read raw data from IMU into RPY coordinate system
  MPU.accelUpdate();
  MPU.gyroUpdate();  
  MPU.magUpdate();

  // Convert to Body Coordinates and correct units
  aR = MPU.accelY()*Gs2SI; // ROLL
  aP = MPU.accelX()*Gs2SI; // PITCH
  aY = -MPU.accelZ()*Gs2SI;// YAW
  gR = MPU.gyroY();
  gP = MPU.gyroX();
  gY = -MPU.gyroZ();
  mR = -MPU.magX();
  mP = -MPU.magY();
  mY = -MPU.magZ();
  
  // update the Madgwick filter, which computes orientation
  AHRSfilter.update(-gR, -gP, -gY, aR, aP, aY, mR, mP, mY);

  accRPY(0) = aR;
  accRPY(1) = aP;
  accRPY(2) = aY;

  BLA::Matrix<4,1,double> q_AHRS;
  q_AHRS(0) = (double) AHRSfilter.q0;
  q_AHRS(1) = (double) AHRSfilter.q1;
  q_AHRS(2) = (double) AHRSfilter.q2;
  q_AHRS(3) = (double) AHRSfilter.q3;
  Quaternion RPYtoNED(q_AHRS);
  Quaternion NEDtoRPY(RPYtoNED.Inverse());
  
  gRPY = NEDtoRPY.Rotate(gNED);
  accRPY -= gRPY;
  u = RPYtoNED.Rotate(accRPY);
 
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
  p = f*p*~f + q;
}

void getBaroObservations() {
  zBaro(0) = -BMP.readAltitude(localAirPressure);
}

void updateKalmanBaro() {
  getBaroObservations();
  BLA::Matrix<NUM_OBS_BARO, NUM_STATES, double > h = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  BLA::Matrix<NUM_OBS_BARO, NUM_OBS_BARO, double > r = {1.0};
  BLA::Matrix<NUM_STATES, NUM_OBS_BARO, double > k;
  BLA::Matrix<NUM_OBS_BARO, NUM_OBS_BARO, double > temp;
  BLA::Matrix<NUM_STATES, NUM_STATES,double > temp2;

  temp = h*p*~h + r;
  k = p*~h*Inverse(temp);
  x += k*(zBaro - h*x);
  temp2 = I - k*h;
  p = temp2*p*~temp2 + k*r*~k;
}

void getGPSObservations() {
  // Uses equirectangular projection based off of https://stackoverflow.com/a/16271669
  double dlat = (double) GPS.latitude;
  double dlon = (double) GPS.longitude;
  zGPS(0) = radiusEarth*dlon*DEG2RAD*cosInitialLat; // E in NED
  zGPS(1) = radiusEarth*dlat*DEG2RAD; // N in NED
  zGPS(2) = (double) GPS.altitude;
}

void updateKalmanGPS() {
  getGPSObservations();
  BLA::Matrix< NUM_OBS_GPS, NUM_STATES, double > h = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                      0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                                      0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  BLA::Matrix<NUM_OBS_GPS, NUM_OBS_GPS, double > r = {3.24, 0.00, 0.00,
                                                      0.00, 3.24, 0.00,
                                                      0.00, 0.00, 11.4};
  BLA::Matrix<NUM_STATES, NUM_OBS_GPS, double > k;
  BLA::Matrix<NUM_OBS_GPS, NUM_OBS_GPS, double > temp;
  BLA::Matrix<NUM_STATES, NUM_STATES, double > temp2;

  temp = h*p*~h + r;
  k = p*~h*Inverse(temp);
  x += k*(zGPS - h*x);
  temp2 = I - k*h;
  p = temp2*p*~temp2 + k*r*~k;
}
