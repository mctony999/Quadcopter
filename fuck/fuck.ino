#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

double accX, accY, accZ;
double gx, gy, gz;
double R_D = 180/3.1415;

#define RESTRICT_PITCH


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  
  Serial.begin(38400);
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

void loop() {
  int16_t aX, aY, aZ;
   accelgyro.getAcceleration(&aX, &aY, &aZ);
   accX = (double) aX;
   accY = (double) aY;
   accZ = (double) aZ;
  
  #ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * R_D;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * R_D;
  #else
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * R_D;
    double pitch = atan2(-accX, accZ) * R_D;
  #endif

  //print Roll and Pitch value
  //Serial.print(accX); Serial.print("\t");
  //Serial.print(accY); Serial.print("\t");
  //Serial.print(accZ); Serial.println("\t");
  Serial.print(roll); Serial.print("\t");
  Serial.print(pitch); Serial.println("\t");

  delay(500);
}
