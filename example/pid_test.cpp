#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include<signal.h>
#include<unistd.h>

#define TRIG 1
#define ECHO 4
#define M_EN 27
#define M_P  28
#define M_N  29
#define TARGET      20
#define REMAP_FAC   102.4

double KP = 0;
double KI = 0;
double KD =  0;

double us(void);
unsigned int pulseIn(int, int);


void sig_handler(int signo)
{
    if (signo == SIGINT){
        digitalWrite(M_EN, LOW);
        printf("\n\nProcess Ended!\n\n");
        exit(0);
    }
}

class PWM
{
public:
    PWM(int en, int p, int n);
    int work(float v, unsigned int d) //velocity(-1000-1000), duration(micro sec.)
    {
        int counter = micros();
        if(v != 0){
            v = v > 1000 ? 1000 : (v < -1000 ? -1000 : v);
            
            digitalWrite(M_EN, LOW);
            digitalWrite(M_P, v > 0 ? HIGH : LOW);
            digitalWrite(M_N, v < 0 ? HIGH : LOW);
            
            v = v > 0 ? v : -v;
            v = v * 0.5 + 500;
            
            unsigned int duty = 60 * (v / 1000);
            
            unsigned int ta = micros();
            while (micros() - ta < d)
            {
                unsigned int t = micros();
                digitalWrite(M_EN, HIGH);
                while (micros() - t < duty);
                digitalWrite(M_EN, LOW);
                while (micros() - t < 60);
            }
            digitalWrite(M_EN, LOW);
            digitalWrite(M_P, LOW);
            digitalWrite(M_N, LOW);
        }
        return micros() - counter;
    }
};

class PID
{
public:
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    
    PID(float, float, float);           //kp,ki,kd
    double remap(double, double);       //input, remap_fac
    double calc(void);
    void reset();
private:
    //PWM pwm(M_EN, M_P, M_N);
    double current = 0;
    double err     = 0;
    double p_err   = 0;
    double ig      = 0;
    double dif     = 0;
    double dt      = 6e-3;
};


int main(void)
{
    wiringPiSetup();
    //printf("test");
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    PWM pwm(M_EN, M_P, M_N);
    scanf("%lf %lf %lf", &KP, &KI, &KD);
    PID pid(KP, KI, KD);
    int count = 0;
    if (signal(SIGINT, sig_handler) == SIG_ERR)
        printf("\ncan't catch SIGINT\n");
    
    
    pid.reset();
    delay(50);
    while(1)
    {
        if (count > 100)
        {
            count = 0;
            pid.reset();
        }else{
            count++;
        }
        double out = 0;
        out = pid.calc();
        printf("%lf\n", out);
        pwm.work(out * 100, 6000);
    }
    return 0;
}



PID::PID(float p, float i, float d)
{
    dt = 0;
    Kp = p;
    Ki = i;
    Kd = d;
}

double PID::remap(double input, double fac)
{
    return input / fac;
}

double PID::calc(void)
{
    double output = 0;
    current = us();
    err = TARGET - current;
    dif = (err - p_err) / dt;
    ig += err * dt;
    ig = (ig > 10) ? 10 : ((ig < -10) ? -10 : ig);
    p_err = err;
    output = Kp * err + Ki * ig + Kd * dif;
    dt = 6e-3;//pwm.work(output * 100, 6000);
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

double us(void)
{
    double distance;
    unsigned int duration;
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(1000);
    digitalWrite(TRIG, LOW);
    duration = pulseIn(ECHO, HIGH);
    distance = (duration / 2.0) / 29;
    printf("dist.: %10.5lf\n", distance);
    return distance;
}

unsigned int pulseIn(int pin, int value)
{
    unsigned int t = 0;
    if(digitalRead(pin) != value)
        while(digitalRead(pin) != value);
    t = micros();
    while(digitalRead(pin) == value);
    t = micros() - t;
    return t;
}

PWM::PWM(int en, int p, int n)
{
    pinMode(M_EN, OUTPUT);
    digitalWrite(M_EN, LOW);
    pinMode(M_P, OUTPUT);
    pinMode(M_N, OUTPUT);
}

