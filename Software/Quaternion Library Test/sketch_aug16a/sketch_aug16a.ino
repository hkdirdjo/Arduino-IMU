#include <QuaternionForINS.h>

#define HWSERIAL Serial1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  BLA::Matrix<4,1,Array<4,1,double>> test = {1.0,1.1,1.2,1.3};
  BLA::Matrix<3,1,Array<3,1,double>> vector = {2.0,2.1,2.2};  
  Quaternion q(test);
  // test = q.Conjugate();
  // test = q.Inverse();
  vector = q.Rotate(vector);
  // Serial.println(String(q.Norm2(),5));
  // Serial.print(test(0)); Serial.print(test(1)); Serial.print(test(2)); Serial.print(test(3));
  Serial.print(vector(0)); Serial.print(vector(1)); Serial.print(vector(2));
}

void loop() {
  // put your main code here, to run repeatedly:

}
