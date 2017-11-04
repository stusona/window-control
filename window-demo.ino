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
#define DIR_PIN     8
#define STEP_PIN    9
#define POT_PIN     A0 // Sense voltage on Potentiometer wiper
#define SWITCH_PIN  A1 // Sense reed switch (LOW is open and HIGH is closed)
#define ENABLE_PIN
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 60
#define MICROSTEPS 1  // 1=full step, 2=half step etc.

DRV8825 stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN);

void setup() {
    stepper.begin(RPM, MICROSTEPS);
}

void loop() {

    // Move motor using steps since each step is 1.8 degrees
    stepper.move(200*MICROSTEPS);

    delay(3000);
}
