#include <util/atomic.h>
#include <SD.h>
#include <SPI.h>




#define OPTENCA 4
#define OPTENCB 5
#define ENCA 2
#define ENCB 3
#define PWM 11
//#define IN2 6
//#define IN1 7
#define DIRA 12
#define BREAKA 8


Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = BUILTIN_SDCARD;
volatile long counter = 0;
const float conv_encoder = 360.0/(1024.0);
volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float eeintegral = 0;
const int dirPinA = 12;
const int pwmPinA = 11;
const int dirPinB = 12;
const int pwmPinB = 11;
const float kp = 2.50; // best so far is 1, 0.025, 0
const float kd = 0.03; // 0.03
const float ki = 0.01; // 0.01
const float kii = 0.01;
const float gear_ratio = 187.0/3591.0;
const float encoder_ratio =  	921.744; // countable events per rev or 12  Cycles Per Revolution
 
void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
//  attachInterrupt(0, A,RISING);
//  attachInterrupt(1, B, RISING);
  pinMode(dirPinA, OUTPUT);
  // digitalWrite(dirPinA,LOW);
  pinMode(pwmPinA, OUTPUT);
  // digitalWrite(pwmPinA,LOW);
  pinMode(dirPinB, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

  // OPT ENCODER PARAM SETUP
  pinMode(4, INPUT_PULLUP); 
  
  pinMode(5, INPUT_PULLUP); 
   
  attachInterrupt(digitalPinToInterrupt(OPTENCA), A, RISING);
   
  attachInterrupt(digitalPinToInterrupt(OPTENCB), B, RISING);  

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
}
 
// void loop() {
//  setMotorB(false, 70, DIRA, PWM, BREAKA);
//  delay(1000);
// //  if( counter != x ){
// //  x = counter;
// //  }
// //  Serial.println(counter);
//  setMotorB(true, 70, DIRA, PWM, BREAKA);
//  // delay(1000);
// //  if( counter != x ){
// //  x = counter;
// //  }
// //  Serial.println(counter);
//  Serial.println("I got past the second command");
//  // setMotorB(false, 70, DIRA, PWM, BREAKA);
//  // delay(1000);
// //  if( counter != x ){
// //  x = counter;
// //  }
// //  Serial.println(counter);
//  // delay(1000);
// }
void loop() {
    // PID constants
  // if (Serial.available() > 0) {
  //   String msg = Serial.readString();
  //   String kp1 = msg.substring(0, 4);
  //   kp = kp1.toFloat();
  //   String kd1 = msg.substring(4, 8);
  //   kd = kd1.toFloat();
  //   String ki1 = msg.substring(8, 12);
  //   ki = ki1.toFloat();
  //   String kii1 = msg.substring(12, 16);
  //   kii = kii1.toFloat();
  //   Serial.println("*****************************************");
  //   Serial.print("changed kp to:  ");
  //   Serial.println(kp);
  //   Serial.print("changed kd to:  ");
  //   Serial.println(kd);
  //   Serial.print("changed ki to:  ");
  //   Serial.println(ki);
  //   Serial.print("changed kii to:  ");
  //   Serial.println(kii);
  // }
  // set target position
  int target = 1200;
  target = 1.5*75*sin(2*prevT/1e6);
 
 
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  float integrand = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    integrand = counter*conv_encoder;

  }
  // error
  int e = pos-target;
 
  // derivative
  float dedt = (e-eprev)/(deltaT);
 
  // integral
  eintegral = eintegral + e*deltaT;

  // DOUBLE INTEGRAL
  eeintegral = eeintegral + eintegral*deltaT;
 
  // control signal
  float u = kp*e + kd*dedt + ki*eintegral + kii*eeintegral;
 
  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
 
  // motor direction
  bool dir = u>0;
  // pwr = 255*sin(prevT/1e6);
 
  // signal the motor
  setMotorB(dir,pwr,DIRA, PWM,BREAKA);
 
  // store previous error
  eprev = e;
  // make a string for assembling the data to log:
  String dataString = "";
  dataString += String(target);
  dataString += ",";
  dataString += String(pos);
  dataString += ",";
  dataString += String(integrand);
 
  File dataFile = SD.open("integral_trail_4.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
 

void setMotorB(bool dir, int pwmVal,int dirPin, int pwmPin, int brakePin) {
  if(dir == false){
    digitalWrite(dirPin, LOW);
  }
  else{
    digitalWrite(dirPin, HIGH);
  }
  analogWrite(pwmPin, pwmVal);
}
 

//void A() {
//  if(digitalRead(3)==LOW) {
//  pos++;
//  }else{
//  pos--;
//  }
//}
//   
//void B() {
//  if(digitalRead(2)==LOW) {
//  pos--;
//  }else{
//  pos++;
//  }
//}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

void A() {
  if(digitalRead(5)==LOW) {
  counter++;
  }else{
  counter--;
  }
}
   
void B() {
  if(digitalRead(4)==LOW) {
  counter--;
  }else{
  counter++;
  }
}
