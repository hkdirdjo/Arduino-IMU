/*
  Husayn Kartodirdjo
  Units: m/s
  NED : Inertial Reference Frame, XYZ : Body
*/
#include <MPU9250_asukiaaa.h>
#include <BasicLinearAlgebra.h> 
using namespace BLA;
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>

SoftwareSerial mySerial(28, 29);
Adafruit_GPS GPS(&mySerial);
MPU9250_asukiaaa mySensor;
Adafruit_BMP280 bmp;

#define GPSECHO false
#define NUM_STATES 5

float accelBias[2] = {0.0}; // [R,P]
float gyroBias = 0.0; // [Y]
float cosInitialLat;
unsigned long previousTime, currentTime; // milliseconds
float deltaTime; // seconds
const float radiusEarth = 6378100; // metres
char c; // for reading GPS NMEA

const float DEG2RAD = 3.14159 / 180;
const float RAD2DEG = 180 / 3.14159;
const float KNOTS2MPS = 0.514444;

BLA::Matrix<NUM_STATES, 1> x; // posN posE velN velE thetaY
auto posNE  = x.Submatrix<2, 1>(0, 0);
auto velNE  = x.Submatrix<2, 1>(2, 0);
auto thetaY = x.Submatrix<1, 1>(4, 0);
BLA::Matrix<6, 1> u; // accN accE omegaY
auto accNE  = u.Submatrix<2, 1>(0, 0);
auto omegaY = u.Submatrix<1, 1>(2, 0);
BLA::Matrix<2, 1> accRP;
BLA::Matrix<5, 1> zGPS; // posN, posE, velN, velE, thetaY
BLA::Matrix<1, 1> zMag; // thetaY
BLA::Matrix<NUM_STATES, NUM_STATES> Q;
BLA::Matrix<NUM_STATES, NUM_STATES> P;
BLA::Matrix<NUM_STATES, NUM_STATES> I;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  Wire.begin();
  mySensor.setWire(&Wire);

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // biasCorrection();
  //getInitialState();
  // Set up covariance matrix Q
  for (int i = 0; i < NUM_STATES; i++) {
    Q(i,i) = 0.1;
  }
  Serial.println("Hello?");
  previousTime = millis();
}

void loop() {
  currentTime = millis();
  deltaTime = (currentTime -  previousTime) / 1000.0; // seconds
  c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // State Prediction
  estimateAngularPosition();
  estimateNEDAcceleration();
  estimateNEDVelocity();
  estimateNEDPosition();

  // Kalman Update
  // updateGPS();
  updateMag();
  
  // TEST AREA
  // Serial.println(String(accelBias[0]) + ", " + String(accelBias[1]) );
  // Serial.println( String(accRP(0)) + ", " + String(accRP(1)) );
  // Serial.println( String(accNE(0)) + ", " + String(accNE(1)) );
  // Serial.println(String(velNE(0)) + ", " + String(velNE(1)) );
  // Serial.println(String(posNE(0)) + ", " + String(posNE(1)) );
  // Serial.println(String(zGPS(0)) + ", " + String(zGPS(1)) );
  Serial.println("Hi!");
  Serial.println(String(thetaY(0)));

  previousTime = currentTime;
}

void biasCorrection() {
  for (int i = 0; i < 1000; i++) {
    mySensor.accelUpdate();
    accelBias[0] += -mySensor.accelY(); // Roll, raw value
    accelBias[1] += -mySensor.accelX(); // Pitch, raw value
    mySensor.gyroUpdate();
    gyroBias += -mySensor.gyroZ(); // Yaw, raw value
  }
  accelBias[0] /= 1000.0;
  accelBias[1] /= 1000.0;
  gyroBias /= 1000.0;
}

void defineIdentityMatrix() {
  for (int i = 0; i < I.Rows; i++) {
    I(i, i) = 1;
  }
}

void getInitialState() {
  char c;
  Serial.println("Waiting for a GPS lock...");
  // Wait to receive a correctly parsed NMEA message
  while (!GPS.fix) {
    c = GPS.read();
    Serial.println(GPS.fix);
    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());
    }
  }
  Serial.println("GPS locked");
  //Serial.println(GPS.latitudeDegrees,10);
  //Serial.println(GPS.longitudeDegrees,10);
  cosInitialLat = cos(GPS.latitudeDegrees * DEG2RAD);
  getGPSObservations();
  x(0) = zGPS(0);
  x(1) = zGPS(1);
  // Assume initial velocity is zero
  x(3) = 0.0;
  x(4) = 0.0;
  Serial.println("Determining magnetometer-based orientation...");
  getMagObservations();
  x(5) = zMag(0);
  Serial.println(x(5));
}

void estimateAngularPosition() {
  mySensor.gyroUpdate();
  thetaY(0)  += ( mySensor.gyroZ() + gyroBias) * DEG2RAD * deltaTime; // Yaw
}

void estimateNEDAcceleration() {
  getAccelRP();

  float sY = sin(thetaY(0));
  float cY = cos(thetaY(0));

  accNE(0) = sY*accRP(0)  + cY*accRP(1);
  accNE(1) = cY*accRP(0)  - sY*accRP(1);

}

void estimateNEDVelocity() {
  velNE += accNE * deltaTime;
}

void estimateNEDPosition() {
  posNE += velNE * deltaTime + accNE * pow(deltaTime, 2);
}

void getAccelRP() {
  mySensor.accelUpdate();
  accRP(0) = (-mySensor.accelY() + accelBias[0])*9.81; // Along the Roll axis
  accRP(1) = (-mySensor.accelX() + accelBias[1])*9.81; // Along the Pitch axis
}

void getGPSObservations() {
  // we poll for new data inside of loop()
  // Uses equirectangular projection based off of https://stackoverflow.com/a/16271669 
  zGPS(0) = radiusEarth * (GPS.latitudeDegrees * DEG2RAD); // N in NED
  zGPS(1) = radiusEarth * (GPS.longitudeDegrees * DEG2RAD) * cosInitialLat; // E in NED
  zGPS(2) = (GPS.speed * KNOTS2MPS) * cos(GPS.angle * DEG2RAD); // velN
  zGPS(3) = (GPS.speed * KNOTS2MPS) * sin(GPS.angle * DEG2RAD); // velE
  zGPS(4) = GPS.angle; // Yaw angle with respect to N in NED
}

void getMagObservations() {
  // using https://www.best-microcontroller-projects.com/magnetometer-tilt-compensation.html
  mySensor.magUpdate();
  zMag(0) = -atan2(mySensor.magY(), mySensor.magX());
}

void updateGPS() {
  getGPSObservations();
  defineIdentityMatrix();
  const int NUM_OBS = 5;
  BLA::Matrix<NUM_STATES,NUM_OBS> K;
  BLA::Matrix<NUM_OBS, NUM_STATES> H;
  for (int i = 0; i < NUM_OBS; i++) {
    H(i,i) = 1.0;
  }
  BLA::Matrix<NUM_OBS, NUM_OBS> S;
  BLA::Matrix<NUM_OBS, NUM_OBS> R;
  for (int i = 0; i < NUM_OBS; i++) {
    R(i,i) = 0.1;
  }

  S = H * P * ~H + R;
  K = P * ~H * Invert(S);
  P = (I - K * H) * P * ~(I - K * H) + K * R * ~K; // Use the Joseph stabilized version instead
  x += K * (zGPS - H * x);
}

void updateMag() {
  getMagObservations();
  defineIdentityMatrix();
  const int NUM_OBS = 1;
  BLA::Matrix<NUM_STATES,NUM_OBS> K;
  BLA::Matrix<NUM_OBS, NUM_STATES> H = {0.0,0.0,0.0,0.0,1.0};
  BLA::Matrix<NUM_OBS, NUM_OBS> S;
  BLA::Matrix<NUM_OBS, NUM_OBS> R;
  for (int i = 0; i < NUM_OBS; i++) {
    R(i,i) = 0.1;
  }
  S = H * P * ~H + R;
  K = P * ~H * Invert(S);
  P = (I - K * H) * P * ~(I - K * H) + K * R * ~K; // Use the Joseph stabilized version instead
  x += K * (zMag - H * x);
}
