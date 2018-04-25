const int pin = 34;
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(34,36);
void setup() {
  // put your setup code here, to run once:
  pinMode(pin,OUTPUT);
  //Serial.begin(9600);
  //mySerial.println("AT");
  //Serial.write("AT");
}

void loop() {

  // put your main code here, to run repeatedly:
  //while (mySerial.available()) {
  //Serial.write(mySerial.read());}
  
  
  
  digitalWrite(pin, HIGH);
  delayMicroseconds(700); // Approximately 10% duty cycle @ 1KHz
  digitalWrite(pin, LOW);
  delayMicroseconds(300);


}
