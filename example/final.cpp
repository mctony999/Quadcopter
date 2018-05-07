
#define G 0
#define A 1
#define GSENS 500
#define ASENS 2

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <thread>
#include "./library/spi.cpp"
#include "./library/lsm330.cpp"
#include "./library/Kalman.cpp"
#include "./library/server.cpp"

const int EN = 0;
const int MP = 2;
const int MN = 3;

LSM330 lsm_ctrl;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

double RAD_TO_DEG = 180/3.1415;

class PWM
{
public:
    PWM(int en, int p, int n) : en(en), p(p), n(n)
    {
        pinMode(en, OUTPUT);
        digitalWrite(en, LOW);
        pinMode(p, OUTPUT);
        pinMode(n, OUTPUT);
    }
    int work(float v, unsigned int d) //velocity(-1000-1000), duration(micro sec.)
    {
        int counter = micros();
        if(v != 0){
            v = v > 1000 ? 1000 : (v < -1000 ? -1000 : v);
            
            digitalWrite(en, LOW);
            digitalWrite(p, v > 0 ? HIGH : LOW);
            digitalWrite(n, v < 0 ? HIGH : LOW);
            
            v = v > 0 ? v : -v;
            v = v * 0.5 + 500;
            
            unsigned int duty = 60 * (v / 1000);
            
            unsigned int ta = micros();
            while (micros() - ta < d)
            {
                unsigned int t = micros();
                digitalWrite(en, HIGH);
                while (micros() - t < duty);
                digitalWrite(en, LOW);
                while (micros() - t < 60);
            }
            digitalWrite(en, LOW);
            digitalWrite(p, LOW);
            digitalWrite(n, LOW);
        }
        return micros() - counter;
    }
private:
    int en;
    int p;
    int n;
};

class PID
{
public:
    float Kp;
    float Ki;
    float Kd;
    
    PID(float p, float i, float d, double target) : Kp(p), Ki(i), Kd(d), TARGET(target)
    {}
    double remap(double, double);       //input, remap_fac
    double calc(double);
    void reset();
private:
    //PWM pwm(M_EN, M_P, M_N);
    double TARGET;
    double current = 0;
    double err = 0;
    double p_err = 0;
    double ig = 0;
    double dif = 0;
    double dt = 6e-3;
};
/*
class BALANCE
{
public:
    BALANCE(LSM330* lsm, int servo) : lsm(lsm), servo(servo)
    {
        pinMode(servo, OUTPUT);
        digitalWrite(servo, LOW);
    }
    void keep_balance()
    {
        
    }
private:
    //PID pid(1, 1, 1, 0);
    //LSM330* lsm;
    int servo;
};

*/
int main(void)
{
    unsigned int t = millis();
    
    wiringPiSetup();
    
    delay(10);
    
    lsm_ctrl.setup(G, GSENS, A, ASENS);
    
    
    delay(100);
    // Wait for sensor to stabilize
    // Set kalman and gyro starting angle
    lsm_ctrl.getAccelValues(A);
    accX = lsm_ctrl.scaledAccelX_;
    accY = lsm_ctrl.scaledAccelY_;
    accZ = lsm_ctrl.scaledAccelZ_;
    
#ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
    
    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;
    
    timer = micros();
    
    while(1)
    {
        if( (millis() - t) > 10 )
        {
            lsm_ctrl.getAccelValues(A);
            delay(20);
            printf("%9.6f %9.6f %9.6f", lsm_ctrl.scaledAccelX_, lsm_ctrl.scaledAccelY_, lsm_ctrl.scaledAccelZ_);
            lsm_ctrl.getGyroValues(G);
            delay(20);
            printf("   ===%11.6f %11.6f %11.6f", lsm_ctrl.scaledGyroX_, lsm_ctrl.scaledGyroY_, lsm_ctrl.scaledGyroZ_);
            
            
            
            
            
            // Update all the values
            accX = lsm_ctrl.scaledAccelX_;
            accY = lsm_ctrl.scaledAccelY_;
            accZ = lsm_ctrl.scaledAccelZ_;
            
            
            gyroX = lsm_ctrl.scaledGyroX_;
            gyroY = lsm_ctrl.scaledGyroY_;
            gyroZ = lsm_ctrl.scaledGyroZ_;
            
            double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
            timer = micros();
            
#ifdef RESTRICT_PITCH
            double roll  = atan2(accY, accZ) * RAD_TO_DEG;
            double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
            double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
            double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
            
            double gyroXrate = gyroX / 131.0; // Convert to deg/s
            double gyroYrate = gyroY / 131.0; // Convert to deg/s
            
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
            
            gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
            gyroYangle += gyroYrate * dt;
            
            //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
            //gyroYangle += kalmanY.getRate() * dt;
            
            compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
            compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
            
            // Reset the gyro angle when it has drifted too much
            if (gyroXangle < -180 || gyroXangle > 180)
                gyroXangle = kalAngleX;
            if (gyroYangle < -180 || gyroYangle > 180)
                gyroYangle = kalAngleY;
            
            
            
            
            
            
            printf("   ===%11.6f %11.6f\n", kalAngleX, kalAngleY);
            t = millis();
        }
    }
    
    return 0;
}


double PID::remap(double input, double fac)
{
    return input / fac;
}

double PID::calc(double input)
{
    double output = 0;
    current = input;
    err = TARGET - current;
    dif = (err - p_err) / dt;
    ig += err * dt;
    ig = (ig > 10) ? 10 : ((ig < -10) ? -10 : ig);
    p_err = err;
    output = Kp * err + Ki * ig + Kd * dif;
    dt = 6e-3;
    return output;
}

void PID::reset()
{
    current = 0;
    err     = 0;
    p_err   = 0;
    ig      = 0;
    dif     = 0;
}
