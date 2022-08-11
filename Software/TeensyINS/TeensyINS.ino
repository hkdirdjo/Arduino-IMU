/*
 * Teensy-based INS by Husayn Kartodirdjo
 * Sensors: GY-91 (MPU9250 and BMP280) and Adafruit Ultimate GPS Board v3
 * North-East-Down reference frame for Inertial Reference Frame
 * Roll-Pitch-Yaw reference frame for body motions
 */

#include <MPU9250_asukiaaa.h> // IMU
#include <BasicLinearAlgebra.h> 
using namespace BLA;
#include <TinyGPS.h>
#include <MadgwickAHRS.h>
#include <Chrono.h>

// Definitions
#define NUM_STATES 5
#define NUM_OBS 5
#define NUM_COM 3
#define HWSERIAL Serial1

// Global Variables
double cosInitialLat = 49.20689;
bool newGPSData = false;
unsigned long previousTime; // milliseconds
double radiusEarth = 6371000; // metres
const int predictRate = 50; // Hz
const int updateRate = 1; // Hz
const int debugRate = 10; // Hz
unsigned long microsPerFilter = 1000000/predictRate;
unsigned long microsPerUpdate = 1000000/updateRate;
unsigned long microsPerDebug = 1000000/debugRate;
double deltaTime = 1.0/ (double)predictRate; // seconds
double ax, ay, az;
double gx, gy, gz;
double mx, my, mz;
double roll, pitch, heading;
float flat, flon;
unsigned long age;

// Unit Conversions
const double DEG2RAD = PI / 180.0;
const double RAD2DEG = 180.0 / 3.14159;
const double KPH2MPS = 1.0/3.6;
const double Gs2SI = 9.81;

// Initialize Objects
Madgwick filter;
TinyGPS GPS;
MPU9250_asukiaaa MPU;
Chrono chronoFilter(Chrono::MICROS);
Chrono chronoUpdate(Chrono::MICROS);
Chrono chronoDebug(Chrono::MICROS);

// Matrices
BLA::Matrix<NUM_STATES, 1, Array<NUM_STATES,1,double> > x; // posE posN velE velN theta
auto posEN = x.Submatrix<2, 1>(0, 0);
auto velEN = x.Submatrix<2, 1>(2, 0);
auto theta = x.Submatrix<1, 1>(4, 0);
BLA::Matrix<NUM_COM, 1, Array<NUM_COM,1,double> > u; // accE accN omega
auto accEN = u.Submatrix<2, 1>(0, 0);
auto omega = u.Submatrix<1, 1>(2, 0);
BLA::Matrix<2,1, Array<2,1,double> > accRP;
BLA::Matrix<NUM_OBS, 1, Array<NUM_OBS,1,double> > zGPS; // posE, posN, velE, velN, theta
BLA::Matrix<NUM_STATES, NUM_STATES, Array<NUM_STATES,NUM_STATES,double> > f = {1.0, 0.0, 0.0, 0.0, 0.0,
                                                                               0.0, 1.0, 0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 1.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0, 1.0, 0.0,
                                                                               0.0, 0.0, 0.0, 0.0, 0.0};
BLA::Matrix<NUM_STATES, NUM_COM, Array<NUM_STATES,NUM_COM,double> > B = {9.9, 0.0, 0.0,
                                                                         0.0, 9.9, 0.0,
                                                                         9.9, 0.0, 0.0,
                                                                         0.0, 9.9, 0.0,
                                                                         0.0, 0.0, 1.0};
BLA::Matrix<NUM_STATES, NUM_STATES, Array<NUM_STATES,NUM_STATES,double> > Q = {8.0, 0.0, 0.0, 0.0, 0.0,
                                                                               0.0, 8.0, 0.0, 0.0, 0.0,
                                                                               0.0, 0.0, 5.0, 0.0, 0.0,
                                                                               0.0, 0.0, 0.0, 5.0, 0.0,
                                                                               0.0, 0.0, 0.0, 0.0, 1.0};// model covariance
BLA::Matrix<NUM_STATES, NUM_STATES, Array<NUM_STATES,NUM_STATES,double> > P;
BLA::Matrix<NUM_STATES, NUM_OBS, Array<NUM_STATES,NUM_OBS,double> > K; // Kalman Gains
BLA::Identity<NUM_STATES, NUM_STATES> I;
BLA::Matrix<NUM_STATES, NUM_STATES, Array<NUM_STATES,NUM_STATES,double> > J; // Temp variable
BLA::Matrix<NUM_OBS, NUM_STATES, Array<NUM_OBS, NUM_STATES,double> > H = {1.0, 0.0, 0.0, 0.0, 0.0,
                                                                          0.0, 1.0, 0.0, 0.0, 0.0,
                                                                          0.0, 0.0, 1.0, 0.0, 0.0,
                                                                          0.0, 0.0, 0.0, 1.0, 0.0,
                                                                          0.0, 0.0, 0.0, 0.0, 1.0};  // Observation matrices
BLA::Matrix<NUM_OBS, NUM_OBS, Array<NUM_OBS,NUM_OBS,double> > R = {5.0, 0.0, 0.0, 0.0, 0.0,
                                                                   0.0, 5.0, 0.0, 0.0, 0.0,
                                                                   0.0, 0.0, 1.0, 0.0, 0.0,
                                                                   0.0, 0.0, 0.0, 1.0, 0.0,
                                                                   0.0, 0.0, 0.0, 0.0, 5.0}; // Measurement covariance matrix
BLA::Matrix<NUM_OBS, NUM_OBS, Array<NUM_OBS,NUM_OBS,double> > S; // Temp variable
BLA::Matrix<NUM_STATES, NUM_STATES, Array<NUM_STATES,NUM_STATES,double> > T; // Temp variable
BLA::Matrix<2,2, Array<2,2,double> > CoordinateTransform;

void setup() {
  // Initiate Connections
  Serial.begin(115200); // For debugging
  HWSERIAL.begin(9600); // For GPS
  Wire.begin(); // For GY-91
  
  // Initialize Objects
  MPU.beginAccel();
  MPU.beginGyro();
  MPU.beginMag();

  filter.begin(predictRate);

  x.Fill(0);
  P.Fill(0);

  cosInitialLat = cos(cosInitialLat*DEG2RAD);
}

void loop() {
  if (chronoFilter.hasPassed(microsPerFilter,true)) {
    predictKalman();
  }
  if (chronoUpdate.hasPassed(microsPerUpdate,true) && newGPSData) {
    updateKalman();
    newGPSData = false;
  }
  if (chronoDebug.hasPassed(microsPerDebug,true)) {

    // Predict debugging
    // Serial.println( String(x(0),9) + "," + String(x(1),9) + "," + String(x(2),5) + "," + String(x(3),5) + "," + String(x(4),5) ); 
    // Serial.println( String(roll,3) + "," + String(pitch,3) );
    // Serial.println( String(heading,3) );
    // Serial.println( String(B(0,0),9) ); 
    // Serial.println( String(deltaTime,9) );        
    // Serial.println( String(roll,3) + "," + String(pitch,3) + "," + String(heading,3) );
    // Serial.println( String(ax,3) + "," + String(ay,3) + "," + String(az,3));    
    // Serial.println( String(accRP(0),3) + "," + String(accRP(1),3) );
    // Serial.println( String(u(0),3) + "," + String(u(1),3) );

    // GPS debugging
    // Serial.println(String(flat,8) + "," + String(flon,8) );
    // Serial.println( String(zGPS(4),8) );
    //Serial.println( String(zGPS(0),12) + "," + String(zGPS(1),12) + "," + String(zGPS(2),12) + "," + String(zGPS(3),12) + "," + String(zGPS(4),12) ); 

    // Magnetometer Debugging
    Serial.println( String(mx,5) + "," + String(my,5) + "," + String(mz,5) + "," + String(sqrt( mx*mx + my*my + mz*mz ),5) );
     
    // Filter debugging
    // Serial.println( String(x(0),9) + "," + String(x(1),9) + "," + String(x(2),9) + "," + String(x(3),9) + "," + String(x(4),9) + "," + String(flat,9) + "," + String(flon,9) ); 
    // Serial.println( String(B(2,0),5) + "," + String(u(0),5)+ "," + String(B(2,0)*u(0),5) ); 
    // Serial.println( String(B(0,0),5) + "," + String(u(0),5)+ "," + String(B(0,0)*u(0),5) ); 
    // Serial.println(String(f(0,0)*x(0)+f(0,2)*x(2),5));
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
  ax = -MPU.accelY()*Gs2SI; // ROLL
  ay = -MPU.accelX()*Gs2SI; // PITCH
  az = -MPU.accelZ()*Gs2SI;// YAW
  MPU.gyroUpdate();
  gx = MPU.gyroY();
  gy = MPU.gyroX();
  gz = MPU.gyroZ();
  MPU.magUpdate();
  mx = MPU.magX();
  my = MPU.magY();
  mz = MPU.magZ();
  
  // update the filter, which computes orientation
  filter.updateIMU(-gx, -gy, gz, -ax, -ay, az);
    
  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw() - 180;
    
  accRP(0) = ax;
  accRP(1) = ay;
  CoordinateTransform = {sin(heading*DEG2RAD),  cos(heading*DEG2RAD),
                         cos(heading*DEG2RAD), -sin(heading*DEG2RAD)};

  // update u matrix                      
  accEN = CoordinateTransform*accRP;
  omega(0) = heading;
  
  // update F matrix
  f(0,2) = deltaTime;
  f(1,3) = deltaTime;

  // update B matrix
  B(0,0) = 0.5*pow(deltaTime,2);
  B(1,1) = B(0,0);
  B(2,0) = deltaTime;
  B(3,1) = deltaTime;

  x = f*x + B*u;
  P = f*P*~f + Q;
}

void updateKalman() {
  getGPSObservations();
  S = H*P*~H + R;
  Invert(S);
  K = P*(~H)*S;
  x += K*(zGPS - H*x);
  J = I-K*H;
  P = J*P*~J + K*R*~K;
}

void getGPSObservations() {
  // Uses equirectangular projection based off of https://stackoverflow.com/a/16271669
  GPS.f_get_position(&flat, &flon, &age);
  double dlat = (double) flat;
  double dlon = (double) flon;
  zGPS(0) = radiusEarth*dlon*DEG2RAD*cosInitialLat; // E in NED
  zGPS(1) = radiusEarth*dlat*DEG2RAD; // N in NED
  zGPS(2) = (GPS.f_speed_mps()) * sin(GPS.f_course() * DEG2RAD);
  zGPS(3) = (GPS.f_speed_mps()) * cos(GPS.f_course() * DEG2RAD);
  zGPS(4) = GPS.f_course(); // Yaw angle with respect to N in NED
}
