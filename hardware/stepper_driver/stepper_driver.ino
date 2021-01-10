#include <Wire.h>
#include "FastAccelStepper.h"

const byte ADRESS = 0x69;

#define ENABLE_PIN  7
#define DIR_PIN     8
#define STEP_PIN    9

volatile byte read_buffer[100];
volatile byte write_buffer[100];
volatile int read_size  = 0;
volatile int write_size = 0;
volatile long stepsToMove;

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::FULL2WIRE, STEP_PIN, DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5



void setup() {
  // put your setup code here, to run once:
  Wire.begin(ADRESS);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.begin(115200);           // start serial for output

  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  Serial.println("What is my purpose?");
  Serial.println("you move the A");
  Serial.println("Oh my god");
}

void loop() {
  // put your main code here, to run repeatedly:

  stepper.run();
 

}

void receiveEvent(int howMany)
{
  while (0 < Wire.available()) {
    byte c = Wire.read(); // receive byte as a character
    read_buffer[read_size] = c;
    read_size++;
  }
  if (read_buffer[0] == 0) {
    //move to
    for (int i;i<5;i++){
      stepsToMove+= (read_buffer[i]>>(8*(4-i))) ;
    }
    stepper.moveTo(stepsToMove);
    Serial.println(stepsToMove, BIN);
  }
  else if (read_buffer[0] == 1) {
    //send back the current angle

  }
  read_size = 0;
}

void requestEvent(void) {

}
