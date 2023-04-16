// Include the AccelStepper library:
#include <AccelStepper.h>
#include <util/atomic.h>
#include <SD.h>
#include <SPI.h>
// define objects for sd card saving 
Sd2Card card;
SdVolume volume;
SdFile root;


long prevT = 0;
const int chipSelect = BUILTIN_SDCARD;
// this counter is for the encoder. Ignore
volatile long counter = 0;
// Define pin connections
const int dirPin = 11;
const int stepPin = 12;
// Define the AccelStepper interface type:
#define MotorInterfaceType 2

// define the pins that control the microstepping
#define MS1 4
#define MS2 5
#define MS3 6


// Create a new instance of the AccelStepper class:
AccelStepper testStepper = AccelStepper(MotorInterfaceType, stepPin, dirPin);

void setup() {
  Serial.begin(115200);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  // one eigth microsteps (1600 steps/rev)
  digitalWrite(MS1,HIGH);
  digitalWrite(MS2,HIGH);
  digitalWrite(MS3,LOW);
  // Set the max speed and acceleration
  testStepper.setMaxSpeed(3000);
  testStepper.setAcceleration(4000);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
}

void loop() {
  int target = 1200;
  target = exp(-0.1*prevT/1e6)*1600.0*sin(2*prevT/1e6);
  // Set the target position/velocity:
  testStepper.moveTo(target);
  // testStepper.setSpeed(target);
  // Run to target position /velocity with set speed and acceleration/deceleration:
  testStepper.runToPosition();
  // testStepper.runSpeed();
  // long pos = 0; 
  // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  //   pos = counter;
  // }
  // Print the target value to serial
  Serial.print(target);
  Serial.println();
  prevT = micros();
 
  // File dataFile = SD.open("encoder_input_test13.txt", FILE_WRITE);

  // // if the file is available, write to it:
  // if (dataFile) {
  //   dataFile.println(dataString);
  //   dataFile.close();
  //   // print to the serial port too:
  //   // Serial.println(dataString);
  // }
  // // if the file isn't open, pop up an error:
  // else {
  //   Serial.println("error opening datalog.txt");
  // }
}

// encoder counting functions. If A is triggered check if B is low or high and add/substract(quaderture)
void A() {
  if(digitalRead(3)==LOW) {
  counter++;
  }else{
  counter--;
  }
}
// encoder counting functions. If B is triggered check if A is low or high (quaderture)
void B() {
  if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++;
  }
}
