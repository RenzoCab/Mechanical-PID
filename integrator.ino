// Include the AccelStepper library:
#include <AccelStepper.h>

// Define number of steps per revolution:
const int stepsPerRevolution = 200;
volatile long x , counter = 0;


// Give the motor control pins names:
#define pwmA 3
#define pwmB 11
#define brakeA 9
#define brakeB 8
#define dirA 12
#define dirB 13

// Define the AccelStepper interface type:
#define MotorInterfaceType 2

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(MotorInterfaceType, dirA, dirB);

void setup() {
  // Set the PWM and brake pins so that the direction pins can be used to control the motor:
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);

  digitalWrite(pwmA, HIGH);
  digitalWrite(pwmB, HIGH);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);

  // Set the maximum steps per second:
  stepper.setMaxSpeed(600);

  // define the encoder pins
  Serial.begin (9600);
  pinMode(2, INPUT_PULLUP); 
  
  pinMode(3, INPUT_PULLUP); 
   
  attachInterrupt(0, A, RISING);
   
  attachInterrupt(1, B, RISING);
}

void loop() {
  if( counter != x ){
  Serial.println (counter);
  x = counter;
  }
  // Set the speed of the motor in steps per second:
  stepper.setSpeed(600);
  // Step the motor with constant speed as set by setSpeed():
  stepper.runSpeed();
}

void A() {
  if(digitalRead(3)==LOW) {
  counter++;
  }else{
  counter--;
  }
}
   
void B() {
  if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++;
  }
}
