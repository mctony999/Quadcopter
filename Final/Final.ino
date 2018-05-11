#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"
#include <Kalman.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t aX, aY, aZ;
int16_t gX, gY, gZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

unsigned int t;
uint32_t timer;
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

  t = millis();
  delay(100);
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
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
  Serial.print(roll); Serial.print("\t");
  Serial.print(pitch); Serial.println("---------------------------\t");
}

void loop() {
//------------------------Kalman Start--------------------------------------
timer = micros();
if( (millis() - t) > 10 )
        {
          //Get value
          accelgyro.getAcceleration(&aX, &aY, &aZ);
          accelgyro.getRotation(&gX, &gY, &gZ);
          
          // Update all the values
          accX = (double) aX;
          accY = (double) aY;
          accZ = (double) aY;
        
            
          gyroX = (double) gX;
          gyroY = (double) gY;
          gyroZ = (double) gZ;
            
          double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
          timer = micros();
            
#ifdef RESTRICT_PITCH
            double roll  = atan2(accY, accZ) * R_D;
            double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * R_D;
#else
            double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * R_D;
            double pitch = atan2(-accX, accZ) * R_D;
#endif
            
            double gyroXrate = gyroX / 131.0; // Convert to deg/s
            double gyroYrate = gyroY / 131.0; // Convert to deg/s
          //Serial.println("fuck3");  
#ifdef RESTRICT_PITCH
            // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
            if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
                kalmanX.setAngle(roll);
                compAngleX = roll;
                kalAngleX = roll;
                gyroXangle = roll;
            } else
                kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
            
            if (abs(kalAngleX) > 90)
                gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
            kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
            // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
            if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
                kalmanY.setAngle(pitch);
                compAngleY = pitch;
                kalAngleY = pitch;
                gyroYangle = pitch;
            } else
                kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
            
            if (abs(kalAngleY) > 90)
                gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
            kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

            //Serial.println("fuck4"); 
            gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
            gyroYangle += gyroYrate * dt;
            
            //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
            //gyroYangle += kalmanY.getRate() * dt;
            
            compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
            compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

            //Reset the gyro angle when it has drifted too much
            if (gyroXangle < -180 || gyroXangle > 180)
                gyroXangle = kalAngleX;
            if (gyroYangle < -180 || gyroYangle > 180)
                gyroYangle = kalAngleY;
            
//------------------kalman end--------------------------------
  //print Roll and Pitch value
  //Serial.println("--------fuck----------"); 
  //Serial.print(roll); Serial.print("\t");
  //Serial.print(pitch); Serial.println("\t");
  //Serial.println("------------------"); 
  Serial.print(kalAngleX); Serial.print("\t");
  Serial.print(kalAngleY); Serial.println("\t");
  //Serial.println("--------ENDEDN----------"); 
  
  t = millis();
}
}
