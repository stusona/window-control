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
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 60

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
#define DIR_PIN   8
#define STEP_PIN  9
#define POT_PIN   A0

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE);

void setup() {
    stepper.begin(RPM, MICROSTEPS);
}

void loop() {

    // energize coils - the motor will hold position
    // stepper.enable();

    /*
     * Moving motor one full revolution using the degree notation
     */
    stepper.rotate(360);

    /*
     * Moving motor to original position using steps
     */
    stepper.move(-200*MICROSTEPS);

    // pause and allow the motor to be moved by hand
    // stepper.disable();

    delay(5000);
}
