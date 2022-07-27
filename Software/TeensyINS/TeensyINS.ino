/*
 * Teensy-based INS by Husayn Kartodirdjo
 * Sensors: GY-91 (MPU9250 and BMP280) and Adafruit Ultimate GPS Board v3
 * North-East-Down reference frame for Inertial Reference Frame
 * Roll-Pitch-Yaw reference frame for body motions
 */

#include <BasicLinearAlgebra.h> 
using namespace BLA;
#include <TinyGPS.h> // GPS
#include <MPU9250_asukiaaa.h> // IMU
#include <Adafruit_BMP280.h> // Barometer

// Definitions
#define NUM_STATES 5
#define HWSERIAL Serial3

// Global Variables
float accelBias[2] = {0.0}; // [R,P]
float gyroBias = 0.0; // [Y]
float cosInitialLat;
unsigned long previousTime, currentTime; // milliseconds
float deltaTime; // seconds
const float radiusEarth = 6378100; // metres

// Unit Conversions
const float DEG2RAD = 3.14159 / 180;
const float RAD2DEG = 180 / 3.14159;
const float KNOTS2MPS = 0.514444;

// Initialize Objects
TinyGPS gps;
MPU9250_asukiaaa IMU;
Adafruit_BMP280 Baro;
elapsedMillis currentTime;

void setup() {
  Serial.begin(115200); // For debugging
  HWSERIAL.begin(9600); // For GPS
  Wire.begin(); // For GY-91
  Wire.setSCL(16);
  Wire.setSDA(17);
  
  IMU.setWire(&Wire);
  IMU.beginAccel();
  IMU.beginGyro();
  IMU.beginMag();

  // biasCorrection();
  // getInitialStates();
  previousTime = currentTime;
}

void loop() {
  // Get GPS data
  bool newGPSData = false;
  while( HWSERIAL.available() ){
    char c = HWSERIAL.read();
    if(gps.encode(c)){
      newdata = true;
    }
  }

  // Estimate State
  // estimateAngularPosition();
  // estimateNEDAcceleration();
  deltaTime = (currentTime - previousTime);
  // estimateNEDVelocity();
  // estimateNEDPosition();
  previousTime = currentTime;

  // Kalman Update
  // updateGPS();
  // updateBaro();
  // updateMag();
}
