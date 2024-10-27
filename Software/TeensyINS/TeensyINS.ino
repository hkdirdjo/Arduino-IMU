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
#include <BLAforKalman.h>       // Kalman Filter

// DEFINITIONS
#define HWSerial Serial1

// USER INPUTS
double cosInitialLat = 43.4675; // Oakville, ON
const int predictRate = 20; // Hz
const int updateGPSRate = 10; // Hz
const int updateBaroRate = 20; // Hz
const int debugRate = 10; // Hz
const double localAirPressure = 1025.00; // in hPa - must update to local conditions before use!

// CONSTANTS
double deltaTime = 1.0/(double)predictRate; // seconds
const double radiusEarth = 6371000; // metres
const unsigned long microsPerFilter = 1000000/predictRate;
const unsigned long microsPerGPSUpdate = 1000000/updateGPSRate;
const unsigned long microsPerBaroUpdate = 1000000/updateBaroRate;
const unsigned long microsPerDebug  = 1000000/debugRate;
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
BLA::Matrix<NUM_COM, 1, double > accRPY; // aR, aP, aY
BLA::Matrix<NUM_COM, 1, double > gyroRPY; // gR, gP, gY
BLA::Matrix<NUM_COM, 1, double > magRPY; // mR, mP, mY
BLA::Matrix<NUM_COM, 1, double > gRPY;
BLA::Matrix<NUM_COM, 1, double > accNED;
BLA::Matrix<NUM_COM, 1, double > u; // accE accN accD, accNED after gravity has been removed
BLA::Matrix<NUM_COM, 1, double > gNED = {0.0, 0.0, 9.50};
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
BLA::Eye<NUM_STATES, NUM_STATES,double> I;

// Initialize Objects
Madgwick AHRSfilter;
MPU9250_asukiaaa MPU;
Adafruit_BMP280 BMP;
Chrono chronoFilter(Chrono::MICROS);
Chrono chronoUpdateGPS(Chrono::MICROS);
Chrono chronoUpdateBaro(Chrono::MICROS);
Chrono chronoDebug(Chrono::MICROS);
Chrono chronoDebugElapsed(Chrono::MICROS);
Adafruit_GPS GPS(&HWSerial); 
KalmanFilter<NUM_STATES, NUM_OBS_GPS, NUM_OBS_BARO, NUM_COM> Filter(f,q,p,b);

void setup() {
  Serial.begin(115200); // For debugging
  
  GPS.begin(9600); // for GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGAGSA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_BAUD_57600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ );
  
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
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

  AHRSfilter.begin(predictRate);

  cosInitialLat = cos(cosInitialLat*DEG2RAD);
}

void loop() {
  if (chronoFilter.hasPassed(microsPerFilter,true)) {
    readPredictSensors();

    // update F matrix
    Filter.f(0,3) = deltaTime;
    Filter.f(1,4) = deltaTime;
    Filter.f(2,5) = deltaTime;

    // update B matrix
    Filter.b(0,0) = 0.5*pow(deltaTime,2);
    Filter.b(1,1) = Filter.b(0,0);
    Filter.b(2,2) = Filter.b(0,0);
    Filter.b(3,0) = deltaTime;
    Filter.b(4,1) = deltaTime;
    Filter.b(5,2) = deltaTime;

    Filter.predict(u);
  }
  if (chronoUpdateBaro.hasPassed(microsPerBaroUpdate,true)) {
    getBaroObservations();
    Filter.updateBaro(zBaro, 1.0);
  }
  if (chronoUpdateGPS.hasPassed(microsPerGPSUpdate,true)) {
    if (newGPSData){
      getGPSObservations();
      Filter.updateGPS(zGPS, GPS.HDOP, GPS.VDOP);
      newGPSData = false;
    }

  }
  if (chronoDebug.hasPassed(microsPerDebug,true)){
    // Serial.println( String(accRPY(0)) + " " + String(accRPY(1)) + " " + String(accRPY(2)) + " " + String(magnitudeAccRPY) + " " + String(accNED(0)) + " " + String(accNED(1)) + " " + String(accNED(2)) + " " + String(magnitudeAccNED)  + " " + String(u(0)) + " " +String(u(1)) + " " +String(u(2)) + " " + String(magnitudeU) + " " + String(AHRSfilter.getRoll(),2) + " " + String(AHRSfilter.getPitch(),2) + " " +String(AHRSfilter.getYaw(),2));
    Serial.println( String(Filter.x(0),2) + "," + String(Filter.x(1),2) + "," + String(Filter.x(2),2) ); // Position State
    // Serial.println( zGPS ); // GPS Readings
    // Serial.println(newGPSData);
    // Serial.println( String(Filter.x(3),2) + "," + String(Filter.x(4),2) + "," + String(Filter.x(5),2) ); // Velocity State
    // Serial.println(deltaTime);
    // Serial.println(Filter.x);
    // Serial.println(newGPSData);
  }
  if (HWSerial.available()) {
    GPS.read();
    if(GPS.newNMEAreceived()) {
      newGPSData = true;
    }
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

void readPredictSensors(){
  // read raw data from IMU into RPY coordinate system
  MPU.accelUpdate();
  MPU.gyroUpdate();  
  MPU.magUpdate();
  accRPY  = {MPU.accelY()*Gs2SI, MPU.accelX()*Gs2SI, -MPU.accelZ()*Gs2SI}; // combined gravity and acceleration
  gyroRPY = {MPU.gyroY(),        MPU.gyroX(),        -MPU.gyroZ()       };
  magRPY  = {-MPU.magX(),       -MPU.magY(),         -MPU.magZ()        };
  // update the Madgwick filter, which computes orientation
  AHRSfilter.update(-gyroRPY(0), -gyroRPY(1), -gyroRPY(2), accRPY(0), accRPY(1), accRPY(2), magRPY(0), magRPY(1), magRPY(2));
  BLA::Matrix<4,1,double> q_AHRS = { (double) AHRSfilter.q0,
                                     (double) AHRSfilter.q1,
                                     (double) AHRSfilter.q2,
                                     (double) AHRSfilter.q3 };
                                     
  Quaternion RPYtoNED(q_AHRS);
  Quaternion NEDtoRPY(RPYtoNED.Inverse());

  accNED = RPYtoNED.Rotate(accRPY);
  u = accNED - gNED;
}

void getBaroObservations() {
  zBaro(0) = BMP.readAltitude(localAirPressure);
}

void getGPSObservations() {
  // Uses equirectangular projection based off of https://stackoverflow.com/a/16271669
  double dlat = (double) GPS.latitude;
  double dlon = (double) GPS.longitude;
  zGPS(0) = radiusEarth*dlon*DEG2RAD*cosInitialLat; // E in NED
  zGPS(1) = radiusEarth*dlat*DEG2RAD; // N in NED
  zGPS(2) = (double) GPS.altitude;
}


