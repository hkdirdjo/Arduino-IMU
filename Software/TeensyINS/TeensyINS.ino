/*
 * Teensy-based INS by Husayn Kartodirdjo
 * Sensors: GY-91 (MPU9250 and BMP280) and Adafruit Ultimate GPS Board v3
 * North-East-Down reference frame for Inertial Reference Frame
 * Roll-Pitch-Yaw reference frame for body motions
 */

#include <MPU9250_asukiaaa.h> // IMU
#include <MadgwickAHRS.h>

// Definitions
#define NUM_STATES 5
#define HWSERIAL Serial3

// Global Variables
float accelBias[2] = {0.0}; // [R,P]
float gyroBias = 0.0; // [Y]
float cosInitialLat;
unsigned long previousTime; // milliseconds
float deltaTime; // seconds
const float radiusEarth = 6378100; // metres
const int filterRate = 25; // Hz

// Unit Conversions
const float DEG2RAD = 3.14159 / 180;
const float RAD2DEG = 180 / 3.14159;
const float KNOTS2MPS = 0.514444;

// Initialize Objects
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
MPU9250_asukiaaa MPU;
elapsedMillis currentTime;

void setup() {
  // Initiate Connections
  Serial.begin(115200); // For debugging
  // HWSERIAL.begin(9600); // For GPS
  Wire.begin(); // For GY-91


  // Initialize Objects
  MPU.beginAccel();
  MPU.beginGyro();
  MPU.beginMag();  

  filter.begin(filterRate);
  
  microsPerReading = 1000000 / filterRate;
  microsPrevious = micros();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

    // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from IMU
    MPU.accelUpdate();
    ax = MPU.accelX();
    ay = MPU.accelY();
    az = MPU.accelZ();
    MPU.gyroUpdate();
    gx = MPU.gyroX();
    gy = MPU.gyroY();
    gz = MPU.gyroZ();

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }

}
