/**
Motor control demo for window in Aurai system
By Stuart Sonatina and Jacquie Nguyen
IDD - 209U, UC Berkeley, November 2017
Details:
  Initialize window position by closing it until the reed switch is triggered
  Move window to setpoint position by driving NEMA17 Stepper Motor

  Uses DRV8825 pololu stepper driver
 */
#include <Arduino.h>
#include "DRV8825.h"

// Pin Definitions
#define STEP_PIN    5
#define DIR_PIN     6
#define SLEEP_PIN   9  // HIGH enables driver and LOW puts it to sleep
#define POT_PIN     A0 // Sense voltage on Potentiometer wiper
#define REED_PIN    A1 // Sense reed switch (LOW is open and HIGH is closed)

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 16
#define MICROSTEPS 1  // 1=full step, 2=half step etc.

DRV8825 stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN);

void setup() {
  // Initialize stepper
  stepper.begin(RPM, MICROSTEPS);

  // Initialize pins
  pinMode(SLEEP_PIN,OUTPUT);
}

void loop() {

  // Enable stepper driver
  digitalWrite(SLEEP_PIN,HIGH);

  /** Move motor using steps since each step is 1.8
  * Positive to move forward, negative to reverse
  */
  stepper.move(200);

  // Disable stepper driver to allow for manual movement
  digitalWrite(SLEEP_PIN,LOW);

  delay(3000);
}
