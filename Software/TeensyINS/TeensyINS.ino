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
#define NUM_OBS 3
#define HWSERIAL Serial1

// Global Variables
float cosInitialLat;
unsigned long previousTime; // milliseconds
const float radiusEarth = 6378100; // metres
const int predictRate = 25; // Hz
const int updateRate = 1; // Hz
const int debugRate = 10; // Hz
unsigned long microsPerFilter = 1000000/predictRate;
unsigned long microsPerUpdate = 1000000/updateRate;
unsigned long microsPerDebug = 1000000/debugRate;
float deltaTime = 1/predictRate; // seconds
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;

// Unit Conversions
const float DEG2RAD = 3.14159 / 180;
const float RAD2DEG = 180 / 3.14159;
const float KPH2MPS = 1.0/3.6;
const float Gs2SI = 9.81;

// Initialize Objects
Madgwick filter;
TinyGPS GPS;
MPU9250_asukiaaa MPU;
Chrono chronoFilter(Chrono::MICROS);
Chrono chronoUpdate(Chrono::MICROS);
Chrono chronoDebug(Chrono::MICROS);

// Matrices
BLA::Matrix<NUM_STATES, 1> x; // posE posN velE velN theta
auto posEN = x.Submatrix<2, 1>(0, 0);
auto velEN = x.Submatrix<2, 1>(2, 0);
auto theta = x.Submatrix<1, 1>(4, 0);
BLA::Matrix<3, 1> u; // accE accN omega
auto accEN = u.Submatrix<2, 1>(0, 0);
auto omega = u.Submatrix<1, 1>(2, 0);
BLA::Matrix<2,1> accRP;
BLA::Matrix<NUM_OBS, 1> zGPS; // posE, posN, velE, velN, theta
BLA::Matrix<NUM_STATES, NUM_STATES> f = {1.0, 0.0, 9.9, 0.0, 0.0,
                                         0.0, 1.0, 0.0, 9.9, 0.0,
                                         0.0, 0.0, 1.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 1.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0};
BLA::Matrix<NUM_STATES, NUM_OBS> B = {9.9, 0.0, 0.0,
                                      0.0, 9.9, 0.0,
                                      9.9, 0.0, 0.0,
                                      0.0, 9.9, 0.0,
                                      0.0, 0.0, 1.0};
BLA::Matrix<NUM_STATES, NUM_STATES> Q; // model covariance
BLA::Matrix<NUM_STATES, NUM_STATES> P;
BLA::Matrix<NUM_STATES, NUM_OBS> K; // Kalman Gains
BLA::Identity<NUM_STATES, NUM_STATES> I;
BLA::Matrix<NUM_STATES, NUM_STATES> J; // temp
BLA::Matrix<NUM_OBS, NUM_STATES> H; // Observation matrices
BLA::Matrix<NUM_OBS, NUM_OBS> R; // Measurement covariance matrix
BLA::Matrix<NUM_OBS, NUM_OBS> S; // Temp variable
BLA::Matrix<NUM_STATES, NUM_STATES> T; // Temp variable
BLA::Matrix<2,2> CoordinateTransform;

void setup() {
  // Initiate Connections
  Serial.begin(115200); // For debugging
  // HWSERIAL.begin(9600); // For GPS
  Wire.begin(); // For GY-91
  
  // Initialize Objects
  MPU.beginAccel();
  MPU.beginGyro();

  filter.begin(predictRate);

  x = {0.0, 0.0, 0.0, 0.0, 0.0};
}

void loop() {
  if (chronoFilter.hasPassed(microsPerFilter,true)) {
    // predictKalman();
  }
  if (chronoUpdate.hasPassed(microsPerUpdate,true)) {
    // updateKalman();
  }
  if (chronoDebug.hasPassed(microsPerDebug,true)) {

    // Predict debugging
    
    // Serial.println( String(x(0),5) + "," + String(x(1),5) + "," + String(x(2),5) + "," + String(x(3),5) + "," + String(x(4),5) ); 
    // Serial.println( String(roll,3) + "," + String(pitch,3) );
    // Serial.println( String(heading,3) );
    // Serial.println( String(B(0,0),9) );    
    // Serial.println( String(roll,3) + "," + String(pitch,3) + "," + String(heading,3) );
    // Serial.println( String(ax,3) + "," + String(ay,3) + "," + String(az,3));    
    // Serial.println( String(accRP(0),3) + "," + String(accRP(1),3) );
    // Serial.println( String(u(0),3) + "," + String(u(1),3) );

    // GPS debugging
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
  B(3,0) = deltaTime;
  B(4,1) = deltaTime;

  x = B*u;
  //x = f*x + B*u;
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
  float flat, flon;
  unsigned long age;
  GPS.f_get_position(&flat, &flon, &age);
  zGPS(0) = radiusEarth * (flon * DEG2RAD) * cosInitialLat; // E in NED
  zGPS(1) = radiusEarth * (flat * DEG2RAD); // N in NED
  zGPS(2) = (GPS.f_speed_kmph() * KPH2MPS) * sin(GPS.f_course() * DEG2RAD);
  zGPS(3) = (GPS.f_speed_kmph() * KPH2MPS) * cos(GPS.f_course() * DEG2RAD);
  zGPS(4) = GPS.f_course(); // Yaw angle with respect to N in NED
}
