


!!!!DRAFTTTTTTTT!!!!!!!!




#include <Wire.h>
#include <math.h>


const int MPU = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal, tcal;
double t, tx, tf, pitch, roll;




void loop() {
 Wire.beginTransmission(MPU);
 Wire.write(0x3B);
 Wire.endTransmission(false);
 Wire.requestFrom(MPU, 14, true);


 AcX = Wire.read() << 8 | Wire.read();
 AcY = Wire.read() << 8 | Wire.read();
 AcZ = Wire.read() << 8 | Wire.read();

 GyX = Wire.read() << 8 | Wire.read();
 GyY = Wire.read() << 8 | Wire.read();
 GyZ = Wire.read() << 8 | Wire.read();

 getAngle(AcX, AcY, AcZ);



void getAngle(int Ax, int Ay, int Az) {
 double x = Ax;
 double y = Ay;
 double z = Az;
 pitch = atan(x / sqrt((y * y) + (z * z)));
 roll = atan(y / sqrt((x * x) + (z * z)));
 pitch = pitch * (180.0 / 3.14);
 roll = roll * (180.0 / 3.14) ;
}

