/**
Motor control demo for window in Aurai system
By Stuart Sonatina and Jacquie Nguyen
IDD - 209U, UC Berkeley, November 2017
Details:
  To be used with Feather Board
  Waits for input from bluetooth, then opens or closes depending on input.

  Uses DRV8825 pololu stepper driver
 */

// Pin Definitions
#define STEP_PIN    5
#define DIR_PIN     6
#define SLEEP_PIN   9  // HIGH enables driver and LOW puts it to sleep
//#define POT_PIN     A0 // Sense voltage on Potentiometer wiper
//#define REED_PIN    A1 // Sense reed switch (LOW is open and HIGH is closed)

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define STEPS_PER_REV 200
#define RPM 10
#define PERIOD 3000; // microseconds

bool characteristic;

void setup() {
  // Initialize pis
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SLEEP_PIN,OUTPUT);

  // Make sure stepper driver is off
  digitalWrite(SLEEP_PIN, LOW);
}

void loop() {

  switch (characteristic) {
    case 0: // open window
    {
      // Enable stepper driver and set direction
      digitalWrite(SLEEP_PIN,HIGH);
      digitalWrite(dirPin,HIGH);

      // turn 10 times
      for(int x = 0; x < STEPS_PER_REV*10; x++) {
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(PERIOD/2);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(PERIOD/2);
      }

      // Turn stepper driver off
      digitalWrite(SLEEP_PIN, LOW);

      break;
    }

    case 1: // close window
    {
      // Enable stepper driver and set direction
      digitalWrite(SLEEP_PIN,HIGH);
      digitalWrite(dirPin,LOW);

      // turn 10 times
      for(int x = 0; x < STEPS_PER_REV*10; x++) {
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(PERIOD/2);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(PERIOD/2);
      }

      // Turn stepper driver off
      digitalWrite(SLEEP_PIN, LOW);

      break;
    }

    default:
    {
      // Make sure stepper driver is off
      digitalWrite(SLEEP_PIN, LOW);
    }
  }

  // Disable stepper driver to allow for manual movement
  digitalWrite(SLEEP_PIN,LOW);
}
