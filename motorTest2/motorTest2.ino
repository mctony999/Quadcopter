#define pwmPin3
const int peroid = 1000;
void setup() {
  pinMode(pemPin , OUTPUT);
  digitalWrite(pwmPin, LOW);
}

void loop() {
  int duty = 50;
  pwm(pwmPin, 10, peroid, duty);
}

void pwm (int pin, int count, int peroid , int duty){
  for ( int i = 0; i < count; i++){
    int t = micros();
    digitalWrite(pin,HIGH);
    //Can do other something
    while(micros()-t<duty);
    digitalWrite(pemPin, LOW);
    while(micros()-t<peroid);
  }
}


