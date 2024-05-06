#include <Arduino.h>
#include <Stepper.h>

// Define the number of steps per revolution
const int stepsPerRevolution = 200;

// Define the pin connections for the stepper motor
const int motorPin1 = 2;
const int motorPin2 = 3;
const int motorPin3 = 4;
const int motorPin4 = 5;

// limit switch
const int limitSwitchPin = 7;

// Initialize the stepper motor library
// Stepper myStepper(stepsPerRevolution, motorPin1, motorPin2);

void setup() {
  // Limit switch pin as input
  pinMode(limitSwitchPin, INPUT_PULLUP);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  // Speed of the stepper motor 
  // myStepper.setSpeed(300); // RPM

  // Initialize serial communication
  Serial.begin(9600);
  delay(1000);
}

void step(int val)
{
  if(val < 0)
    digitalWrite(motorPin3, HIGH);
  else
    digitalWrite(motorPin3, LOW);

  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin4, LOW);
  for (int i=0; i<abs(val); i++){
    digitalWrite(motorPin1, HIGH);
    delay(3);
    digitalWrite(motorPin1, LOW);
    delay(3);
  }
}

void loop() {
  // Check if the limit switch is pressed or serial data is available
    // If limit switch is pressed
  if (digitalRead(limitSwitchPin) == HIGH) {
    step(290);
    step(-250);
  }

  // If serial data is available
  if (Serial.available() > 0) {
    // Read the serial command
    char command = Serial.read();

    // Perform action based on the command
    switch (command) {
      case 'F':
        step(290);
        break;
      case 'B': // Backward
        step(-250);
        break;
      default:
        // Invalid command
        //Not Readable
        break;
    }
  }
  
}