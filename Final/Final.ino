#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"
#include <Kalman.h>
#include <SoftwareSerial.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

double accX, accY, accZ;
//double gyroX, gyroY, gyroZ;
int16_t aX, aY, aZ;
//int16_t gX, gY, gZ;



//double gyroXangle, gyroYangle; // Angle calculate using the gyro only
//double compAngleX, compAngleY; // Calculated angle using a complementary filter
//double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

//Kalman kalmanX; // Create the Kalman instances
//Kalman kalmanY;
//--------------PID coefficient------------- 
int kp = 10;
int ki = 0;
int kd = 0;

double current_pitch;
double current_roll;
double Pit_I_ERR = 0; //PID Pitch I Error
double Pit_DL_ERR = 0; //PID Pitch D last time Error
unsigned int Pit_dt; 
double Rol_I_ERR = 0; //PID Roll I Error
double Rol_DL_ERR = 0; //PID Roll D last time Error
unsigned int Rol_dt;
int Pit_offset = 0;
int Rol_offset = 0;

double acZ_I_ERR = 0; //PID accZ I Error !!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
double acZ_DL_ERR = 0; //PID accZ D last time Error !!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
unsigned int acZ_dt; //!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
int current_acZ; //!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
int initial_acZ; //!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
//--------------------------------------------

//-------------PWM coefficient---------------
int idle_duty = 50;

unsigned int t;
int period = 1000;
int Pit_duty_change;
int Rol_duty_change;
int acZ_duty_change; //!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!

int Big_cycle = 5;

uint32_t timer;
double R_D = 180/3.1415;

//--------blue tooth--------------
SoftwareSerial I2CBT(0,1);// define 0 as Rx0 1 as TX1
int command; //buletooth command

#define RESTRICT_PITCH

//----------Function Define-------------

int PIDPitch (int Pit_offset, double current_pitch) {
	double error;
	double Pit_D_ERR;
	double dt;
	dt = millis()-Pit_dt;
	error = current_pitch - Pit_offset; // calculate the current error
	Pit_I_ERR = (2/3)*Pit_I_ERR + error*dt; // calculate the Integral
	Pit_D_ERR = (error - Pit_DL_ERR)/dt; // calculate the Derivative
	Pit_duty_change = kp*error + ki*Pit_I_ERR + kd* Pit_D_ERR; // calculate the duty cycle change
	Pit_DL_ERR = error;
	Pit_dt = millis();
  return Pit_duty_change;
}
int PIDRoll (int Rol_offset,double current_roll) {
	double error;
	double Rol_D_ERR;
	double dt;
	dt = millis()-Rol_dt;
	error = current_roll - Rol_offset; // calculate the current error
	Rol_I_ERR = (2/3)*Rol_I_ERR + error*dt; // calculate the Integral
	Rol_D_ERR = (error - Rol_DL_ERR)/dt; // calculate the Derivative
	Rol_duty_change = kp*error + ki*Rol_I_ERR + kd* Rol_D_ERR; // calculate the duty cycle change
	Rol_DL_ERR = error;
	Rol_dt = millis();
  return Rol_duty_change;
}


//!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!!!!
int PIDaccZ (int initial_acZ, int current_acZ) {
	double error;
	double acZ_D_ERR;
	double dt;
	dt = millis()-acZ_dt;
	error = initial_acZ - current_acZ; // calculate the current error
	acZ_I_ERR = (2/3)*acZ_I_ERR + error*dt; // calculate the Integral
	acZ_D_ERR = (error - acZ_DL_ERR)/dt; // calculate the Derivative
	acZ_duty_change = kp*error + ki*acZ_I_ERR + kd* acZ_D_ERR; // calculate the duty cycle change
	acZ_DL_ERR = error;
	acZ_dt = millis();
  return acZ_duty_change;
}
//!!!!!!!!!!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!!!!!
//!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!

//-----------motor number--------------
//			2			  3
//			      roll
//					^
//				    |
//				    x
// pitch	<- 	y 
//			4			  5
//-------------------------------------
void start_PWM() {
    digitalWrite(2,HIGH); //motor1 start
	digitalWrite(3,HIGH); //motor2 start
	digitalWrite(4,HIGH); //motor3 start
	digitalWrite(5,HIGH); //motor4 start
    //Can do other something
    delay(2000);
	digitalWrite(2,LOW); //motor1 start
	digitalWrite(3,LOW); //motor2 start
	digitalWrite(4,LOW); //motor3 start
	digitalWrite(5,LOW); //motor4 start
  }
  

void PWM (int period, int idle_duty, int Pit_duty_change, int Rol_duty_change, int acZ_duty_change) { //!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
	
	int motor2_duty, motor3_duty, motor4_duty, motor5_duty;
	int count =100; //2 SECONDS!!!!!!!!!!!!!!!!NOT SURE!!!!!!!!!!!!!!!!!!!!!
	
	if (Rol_duty_change > 0) { //calculate duty cycle change
		motor2_duty = idle_duty - Rol_duty_change; // +idle_duty/-Rol_duty_change
		motor3_duty = idle_duty + Rol_duty_change; 
		motor4_duty = idle_duty - Rol_duty_change;
		motor5_duty = idle_duty + Rol_duty_change;
		}
	else {
		motor2_duty = idle_duty + Rol_duty_change;
		motor3_duty = idle_duty - Rol_duty_change;
		motor4_duty = idle_duty + Rol_duty_change;	
		motor5_duty = idle_duty - Rol_duty_change;
	}
	
	if (Pit_duty_change > 0) {
		motor2_duty = motor2_duty + Pit_duty_change;
		motor3_duty = motor3_duty + Pit_duty_change;
		motor4_duty = motor4_duty - Pit_duty_change;
		motor5_duty = motor5_duty - Pit_duty_change;
		}
	else {
		motor2_duty = motor2_duty - Pit_duty_change;
		motor3_duty = motor3_duty - Pit_duty_change;
		motor4_duty = motor4_duty + Pit_duty_change;
		motor5_duty = motor5_duty + Pit_duty_change;
	}
	
	//!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
	motor2_duty = motor2_duty + acZ_duty_change;
	motor3_duty = motor3_duty + acZ_duty_change;
	motor4_duty = motor4_duty + acZ_duty_change;
	motor5_duty = motor5_duty + acZ_duty_change;
	//!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
	
	// make sure duty cycle wont > 100
	if (motor2_duty > 100){
	motor2_duty = 100;
	}
	if (motor3_duty > 100){
	motor3_duty = 100;
	}	
	if (motor4_duty > 100){
	motor4_duty = 100;
	}	
	if (motor5_duty > 100){
	motor5_duty = 100;
	}
	// make sure duyy wont < 0
	if (motor2_duty < 0){
	motor2_duty = 0;
	}
	if (motor3_duty < 0){
	motor3_duty = 0;
	}	
	if (motor4_duty < 0){
	motor4_duty = 0;
	}	
	if (motor5_duty < 0){
	motor5_duty = 0;
	}
	
	
  for ( int i = 0; i < count; i++){
    int t = micros();
	digitalWrite(2,HIGH); //motor1 start
	digitalWrite(3,HIGH); //motor2 start
	digitalWrite(4,HIGH); //motor3 start
	digitalWrite(5,HIGH); //motor4 
	//Can do other something
	int triger_a = 0;
	while (triger_a != 15){
		if (micros()-t<motor2_duty){
			digitalWrite(2,LOW);
			triger_a |= 1;
		}
		if (micros()-t<motor3_duty){
			digitalWrite(3,LOW);
			triger_a |= 2;
		}
		if (micros()-t<motor4_duty){
			digitalWrite(4,LOW);
			triger_a |= 4;
		}
		if (micros()-t<motor5_duty){
			digitalWrite(5,LOW);
			triger_a |= 8;
		}
	}

    while(micros()-t<period);
  }
}
//-----------------function define end----------------------





void setup() {
	// put your setup code here, to run once:
	Wire.begin();
	
	Serial.begin(38400);
	I2CBT.begin(38400); //bluetooth baud rate
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
	accZ = (double) aZ*-1;
  
  initial_acZ = (int) accZ/1000; //!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
  
  start_PWM();
  
/*  
#ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * R_D;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * R_D;
  #else
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * R_D;
    double pitch = atan2(-accX, accZ) * R_D;
  #endif
*/
}

void loop() {

for ( int i = 0; i < Big_cycle; i++){
	
	//Get value
	accelgyro.getAcceleration(&aX, &aY, &aZ);
	//accelgyro.getRotation(&gX, &gY, &gZ);
        
	// Update all the values
	accX = (double) aX;
	accY = (double) aY*-1;
	accZ = (double) aY*-1;
            
	current_acZ = (int) accZ/1000; //!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
	
#ifdef RESTRICT_PITCH
	double roll  = atan2(accY, accZ) * R_D;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * R_D;
#else
	double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * R_D;
	double pitch = atan2(-accX, accZ) * R_D;
#endif

//--------PID-------------
Pit_duty_change = PIDPitch(Pit_offset,pitch);
Rol_duty_change = PIDRoll(Rol_offset, roll);
acZ_duty_change = PIDaccZ(initial_acZ,current_acZ);
//------------------------

PWM(period, idle_duty, Pit_duty_change, Rol_duty_change, acZ_duty_change); //!!!!!!!!!!!UP DOWN TESTING CODE!!!!!!!!!!!
}
//----------bule tooth-----------
//			  a
//		2			3
//			  ^
//   b		< e >		c
//			  v
//		4			5
//			  d
//-------------------------------

byte cmmd[20];//宣告含有20個byte型態元素之陣列
if (command=(I2CBT.available())>0){ // check DO I have message?
	for (int i=0; i<command; i++){
		cmmd[i] = char(I2CBT.read());
	}
	switch (cmmd[0]){
		case 97: // a
		Pit_offset = 1;
		Rol_offset = 0;
		case 98: // b
		Pit_offset = 0;
		Rol_offset = -1;
		case 99: // c
		Pit_offset = 0;
		Rol_offset = 1;
		case 100: // d
		Pit_offset = 1;
		Rol_offset = 0;
		case 101: // e
		Pit_offset = 0;
		Rol_offset = 0;
	}
}
}


