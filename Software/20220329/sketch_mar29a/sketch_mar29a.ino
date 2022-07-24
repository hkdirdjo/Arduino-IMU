/*
  Husayn Kartodirdjo
  Units: m/s
  NED : Inertial Reference Frame, XYZ : Body
*/
#include <MPU9250_asukiaaa.h>
#include <BasicLinearAlgebra.h> using namespace BLA;
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
MPU9250_asukiaaa mySensor;
Adafruit_BMP280 bmp;

#define GPSECHO false
#define NUM_STATES 9

float accelBias[3] = {0.0}; // [R,P,Y]
float gyroBias[3] = {0.0}; // [R,P,Y]
bool initialGPSSuccess = false;
float cosInitialLat;
unsigned long previousTime, currentTime; // milliseconds
float deltaTime; // seconds
const float radiusEarth = 6378100; // metres
char c; // for reading GPS NMEA

const float DEG2RAD = 3.14159 / 180;
const float RAD2DEG = 180 / 3.14159;
const float KNOTS2MPS = 0.514444;

BLA::Matrix<NUM_STATES, 1> x; // posN posE posD velN velE velD thetaR thetaP thetaY
auto posNED = x.Submatrix<3, 1>(0, 0);
auto velNED = x.Submatrix<3, 1>(3, 0);
auto thetaRPY = x.Submatrix<3, 1>(6, 0);
BLA::Matrix<6, 1> u; // accN accE accD omegaR omegaP omegaY
auto accNED = u.Submatrix<3, 1>(0, 0);
auto omegaRPY = u.Submatrix<3, 1>(3, 0);
BLA::Matrix<3, 1> accRPY;
BLA::Matrix<6, 1> zGPS; // posN, posE, posD, thetaY, velP, velR
BLA::Matrix<1, 1> zMag; // thetaY
BLA::Matrix<2, 1> zAcc; // thetaR thetaP
BLA::Matrix<1, 1> zBMP; // posD
BLA::Matrix<3, 1> g = {0, 0, 9.81}; // in NED coordinate frame
BLA::Matrix<NUM_STATES, NUM_STATES> Q;
BLA::Matrix<NUM_STATES, NUM_STATES> P;
BLA::Matrix<NUM_STATES, 1> K;
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

  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  biasCorrection();
  //defineIdentityMatrix();
  //getInitialState();

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
  //accNED += g;
  // estimateNEDVelocity();
  // estimateNEDPosition();

  // Kalman Update
  // updateGPS();
  // updateMag();
  // updateAcc();
  // updateBMP();

  // TEST AREA
  //Serial.println(String(accelBias[0]) + ", " + String(accelBias[1]) + ", " + String(accelBias[2]));
  Serial.println(String(accRPY(0)) + ", " + String(accRPY(1)) + ", " + String(accRPY(2)));
  //Serial.println(String(accNED(0)) + ", " + String(accNED(1)) + ", " + String(accNED(2)));
  //Serial.println(String(velNED(0)) + ", " + String(velNED(1)) + ", " + String(velNED(2)));
  //Serial.println(String(accNED(0)) + ", " + String(accNED(1)) + ", " + String(accNED(2)) + ", " + String(velNED(0)) + ", " + String(velNED(1)) + ", " + String(velNED(2)));
  //Serial.println(String(posNED(0)) + ", " + String(posNED(1)) + ", " + String(posNED(2)));
  //Serial.println(String(thetaRPY(0)) + ", " + String(thetaRPY(1)) + ", " + String(thetaRPY(2)));

  previousTime = currentTime;
}

void biasCorrection() {
  for (int i = 0; i < 1000; i++) {
    mySensor.accelUpdate();
    accelBias[0] += -mySensor.accelY(); // Roll, raw value
    accelBias[1] += -mySensor.accelX(); // Pitch, raw value
    accelBias[2] += -mySensor.accelZ(); // Yaw, raw value
    mySensor.gyroUpdate();
    gyroBias[0] +=  mySensor.gyroY(); // Roll, raw value
    gyroBias[1] +=  mySensor.gyroX(); // Pitch, raw value
    gyroBias[2] += -mySensor.gyroZ(); // Yaw, raw value
  }
  for (int i = 0; i < 3; i++) {
    accelBias[i] = accelBias[i] / 1000.0;
    gyroBias[i] /= 1000.0;
  }
  accelBias[2] -=  1;
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
  Serial.println("Determining barometer-based altitude...");
  getBMPObservations();
  x(2) = zBMP(0);
  Serial.println(zBMP(0));
  // Assume initial velocity is zero
  x(3) = 0.0;
  x(4) = 0.0;
  x(5) = 0.0;
  Serial.println("Determining accelerometer-based orientation...");
  getAccObservations();
  x(6) = zAcc(0);
  x(7) = zAcc(1);
  Serial.println(x(6));
  Serial.println(x(7));
  Serial.println("Determining magnetometer-based orientation...");
  getMagObservations();
  x(8) = zMag(0);
  Serial.println(x(8));
}

void estimateAngularPosition() {
  mySensor.gyroUpdate();

  thetaRPY(0)  += (-mySensor.gyroY() + gyroBias[0]) * DEG2RAD * deltaTime; // Roll
  thetaRPY(1)  += (-mySensor.gyroX() + gyroBias[1]) * DEG2RAD * deltaTime; // Pitch
  thetaRPY(2)  += ( mySensor.gyroZ() + gyroBias[2]) * DEG2RAD * deltaTime; // Yaw
}

void estimateNEDAcceleration() {
  getAccelRPY();

  float sR = sin(thetaRPY(0));
  float cR = cos(thetaRPY(0));
  float sP = sin(thetaRPY(1));
  float cP = cos(thetaRPY(1));
  float sY = sin(thetaRPY(2));
  float cY = cos(thetaRPY(2));

  BLA::Matrix<3, 3> RPYtoNED = {cY * cP, -sY*cR + cY*sP * sR,  sY*sR + cY*sP * cR,
                                sY * cP, cY*cR + sY*sP * sR , -cY*sR + sY*sP*cR
                                 - sP  , cP * sR            ,  cP*cR
                               };

  accNED = RPYtoNED * accRPY; // NED frame
}

void estimateNEDVelocity() {
  velNED += accNED * deltaTime;
}

void estimateNEDPosition() {
  posNED += velNED * deltaTime + accNED * pow(deltaTime, 2);
}

void getAccelRPY() {
  mySensor.accelUpdate();

  accRPY(0) = (mySensor.accelY() - accelBias[0])*9.81; // Along the Roll axis
  accRPY(1) = (mySensor.accelX() - accelBias[1])*9.81; // Along the Pitch axis
  accRPY(2) = (-mySensor.accelZ() - accelBias[2])*9.81; // Along the Yaw axis
}

void getGPSObservations() {

  // Uses equirectangular projection based off of https://stackoverflow.com/a/16271669
  zGPS(0) = radiusEarth * (GPS.latitudeDegrees * DEG2RAD); // N in NED
  zGPS(1) = radiusEarth * (GPS.longitudeDegrees * DEG2RAD) * cosInitialLat; // E in NED
  zGPS(2) = GPS.altitude; // z in NED
  zGPS(3) = GPS.angle; // Yaw angle with respect to N in NED
  zGPS(4) = (GPS.speed * KNOTS2MPS) * sin(GPS.angle * DEG2RAD);
  zGPS(5) = (GPS.speed * KNOTS2MPS) * cos(GPS.angle * DEG2RAD);
}

void getBMPObservations() {
  zBMP(0) = -bmp.readAltitude(1019.25); // z in NED
}

void getAccObservations() {
  // based on https://forum.arduino.cc/t/getting-pitch-and-roll-from-acceleromter-data/694148
  getAccelRPY();
  zAcc(0) = atan2(accRPY(1) , accRPY(2));
  zAcc(1) = atan2( (-accRPY(0)) , sqrt(accRPY(1) * accRPY(1) + accRPY(2) * accRPY(2)) );
}

void getMagObservations() {
  // using https://www.best-microcontroller-projects.com/magnetometer-tilt-compensation.html
  mySensor.magUpdate();
  float sR = sin(thetaRPY(0));
  float cR = cos(thetaRPY(0));
  float sP = sin(thetaRPY(1));
  float cP = cos(thetaRPY(1));

  double Mx = mySensor.magX() * cP + mySensor.magY() * sR * sP - mySensor.magZ() * cR * sP;
  double My = mySensor.magY() * cR + mySensor.magZ() * sR ;

  zMag(0) = -atan2(My, Mx);
}

void updateGPS() {
  getGPSObservations();
  BLA::Matrix<1, NUM_STATES> H = {};
  BLA::Matrix<1, 1> S;
  double r[zGPS.Rows];
  BLA::Matrix<1, 1> Z;
  BLA::Matrix<1, 1> R;

  for (int i = 0; i < zGPS.Rows; i++)
  {
    Z = zGPS(i);
    R = r[i];
    S = H * P * ~H + R;
    K = P * ~H * Invert(S);
    // P *= (I - K*H);
    P = (I - K * H) * P * ~(I - K * H) + K * R * ~K; // Use the Joseph stabilized version instead
    x += K * (Z - H * x);
  }
}
