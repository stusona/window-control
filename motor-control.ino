/*
Motor control for window in Aurai system
By Stuart Sonatina and Jacquie Nguyen
IDD - 209U, UC Berkeley, November 2017
Details:
  Initialize window position by closing it until the reed switch is triggered
  Move window to setpoint position by driving NEMA17 Stepper Motor

  Uses DRV8825 pololu stepper driver
*/
// Serial write, or serial plot?
#define serWrite 1

// Pin Definitions
#define POT_PIN     0  // Potentiometer for window position feedback
#define DIR_PIN     8  // direction pin on DRV8825
#define STP_PIN     9  // step pin on DRV8825
#define SWT_PIN     A0 // Reed switch on window

// Potentiometer Definitions
//#define DEG_PER_CNT NEED TO DEFINE

// Stepper Definitions
#define MOTOR_STEPS 200  //Steps per full motor revolution
#define RPM 120          //Motor speed
#define MICROSTEPS 1     //Low resolution (full step)
#define OPEN_POS 200     //Absolute step position of open window
#define CLOSED_POS 0     //Absolute step position of closed window

// Reed switch Definitions
#define CLOSED_FLAG False

// Stop motor after a period of time (in case of mishaps)
#define CUTOFF_TIME 10000 // (ms)

#include <Arduino.h>
#include "DRV8825.h"

// Using a 200-step motor
DRV8825 stepper (200, 8, 9, ms1, ms2, ms3);

void setup() {
  //Set target motor RPM to 1RPM and microstepping at resolution 1 (full)
  stepper.begin(RPM, MICROSTEPS);
}

void loop() {
  stepper.move (); //
}


/*****************************************************************************/



// Delays and periods
#define SERIAL_WRITE_PERIOD 40   // ms
#define TA_DELAY            2000  // ms Wait for TA arduino
#define CONTROLLER_PERIOD   10    // ms
#define POSITION_HOLD_TIME  1000  // ms

// PID Constants
#define KP   300
#define KI   1800
#define KD   20

// Variable Declarations

// Transition angles
float thetaTop;    // degrees
float thetaImpact; // degrees
float thetaStop;    // degrees

// Holding at angle timer and counter;
unsigned long timeatpos = 0;
int posholdcount = 0;

long encoderCount = 0;        // Current encoder position
unsigned long beginTime = 0;  // Time when loop() starts
int motorSpeed  = 0;          // Percent

// Runtime (timer) Variables
unsigned long serialWriteRunTime = 0;
unsigned long controllerRunTime = 0;

// Define state struct type
// add flag for manual change
typedef struct S_t
{
  float theta[2];     // radians
  float theta_dot[2]; // rad/s
  float t[2];         // seconds
  float u[2];         // output to motor
  int   state;        // state of wheel
} S_t;

// Define PID controller struct type
typedef struct PID_t
{
  // Constants
  float kp = 0;
  float ki = 0;
  float kd = 0;

  // Setpoint/Target
  float setpoint = 0;         // radians

  // Saturation value
  float errorsat = 0;

  // Error storage
  float error[2] = {0,0};     // radians
  float errorsum = 0;
  float t[2] = {0,0};         // seconds

  // Enabled?
  int enabled = 0;
} PID_t;

// Create state structure
S_t S;

// Create PID controller structure
PID_t PID;

///////////
// Setup //
///////////
void setup() {
  // Transition angles
  thetaTop    = deg2rad(-172);  // degrees
  thetaImpact = deg2rad(-180); // degrees
  thetaStop   = deg2rad(0); // degrees

  // Initialize pins
  pinMode(POT_PIN,INPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT);
  pinMode(TA_PIN, OUTPUT);

  // Start serial connection
  Serial.begin(250000);

  // Set state to 0 to begin program
  S.state = 0;
}

//////////
// Loop //
//////////
void loop()
{
  // Depending on state, perform action
  switch(S.state)
  {
    case 0: // 0. Wait for button press.
    {
      // Make sure motor is stopped
      stopMotor();

      // Wait for input from user
      if(serWrite) Serial.println();
      if(serWrite) Serial.println("----- Press start button -----");
      while (digitalRead(START_PIN)) delay(10);

//        Serial.println("Tell the TAs that we're ready...");
//        digitalWrite(TA_PIN,LOW);
//        delay(TA_DELAY);
//        digitalWrite(TA_PIN,HIGH);

      // First state: get to -175 degrees
      S.state = 1;
      if(serWrite) Serial.println("Moving to State 1");

      // Reset encoder value
      encoderCount = 0;

      // Set zero time to use as offset
      beginTime = millis();

      break;
    }
    case 1: // 1. Initialize controller for lift.
    {
      // Initialize PID controller for lift
      PID.kp = KP_LIFT;
      PID.ki = KI_LIFT;
      PID.kd = KD_LIFT;
      PID.errorsat = 100/PID.kp;
      PID.enabled = 1;

      PID.setpoint = thetaTop;

      // Initialization is done, so immediately go to state 2
      S.state = 2;
      if(serWrite) Serial.println("Moving to State 2");
      break;
    }
    case 2: // 2. Raise mass from bottom to top (-172 degrees).
    {
      if(abs(PID.error[0]) <= deg2rad(2))
      {
        // You've reached the top!
        timeatpos = millis();
        if(serWrite) Serial.println("thetaTop reached");
        if(serWrite) Serial.println("Moving to State 3");
        S.state = 3;
      }
      break;
    }
    case 3: // 3. Wait at top for prescribed amount of time
    {
      // Make sure we're still at the top
      if(S.theta[0] <= -170 && S.theta[0] >= -180)
      {
        // if not, go back to state 2
        S.state = 2;
        if(serWrite) Serial.println("Moving back to State 2");
      }
      // If we've been here long enough, then we can proceed.
      else if( millis() - timeatpos > POSITION_HOLD_TIME )
      {
        S.state = 4;
        if(serWrite) Serial.println("Moving to State 4");
      }
      break;
    }
    case 4: // 4. Start to swing mass around +355 degrees to strike pendulum at top.
    {
      // Run free for a while.  100% ALL THE WAY!  But make sure PID is disabled first
      PID.enabled = 0;
      setMotor(100);

      // Wait a little to let the wheel go past the strike zone
      delay(100);

      // Prep work done - continue to next state
      S.state = 5;
      if(serWrite) Serial.println("Moving to State 5");
      break;
    }
    case 5: // 5. Wait for mass to strike pendulum at top (-180 degrees).
    {
      if(S.theta[0] <= thetaImpact)
      {
        if(serWrite) Serial.println("Impact");
        S.state = 6;
        if(serWrite) Serial.println("Moving to State 6");
      }
      break;
    }
    case 6: // 6. Initialize controller for follow-through
    {
      // Initialize PID controller for Stopping
      PID.kp = KP_STOP;
      PID.ki = KI_STOP;
      PID.kd = KD_STOP;
      PID.errorsat = 100/PID.kp;
      PID.enabled = 1;

      PID.setpoint = thetaStop;

      // Initialization is done, so immediately go to state 7
      S.state = 7;
      if(serWrite) Serial.println("Moving to State 7");
      break;
    }
    case 7: // 7. Wait for controller to reach bottom (0 degrees)
    {
      if(abs(PID.error[0]) <= deg2rad(1.5))
      {
        // You've reached the end!
        if(serWrite) Serial.println("thetaStop reached");
        timeatpos = millis();
        S.state = 8;
        if(serWrite) Serial.println("Moving to State 8");
      }
      break;
    }
    case 8: // 8. Wait for controller to stabilize at bottom (0 degrees)
    {
      // Make sure we're hanging out at the bottom
      if(abs(PID.error[0]) > deg2rad(1.5))
      {
        // if not, go back to state 7
        S.state = 7;
        if(serWrite) Serial.println("Moving back to State 7");
      }
      // If we've been here long enough, then we can proceed.
      else if( millis() - timeatpos > POSITION_HOLD_TIME )
      {
        // You've reached the end!
        if(serWrite) Serial.println("Stopping Motor");
        stopMotor();
        if(serWrite) Serial.print("Execution time: ");
        if(serWrite) Serial.print(timeatpos - beginTime);
        if(serWrite) Serial.println(" ms");

        // Restart
        S.state = 0;
        if(serWrite) Serial.println("Moving to State 0");
      }
      break;
    }
    default:
    {
      S.state = 0;
      if(serWrite) Serial.println("Moving to State 0");
    }
  }

  // Run controller every CONTROLLER_PERIOD
  if(millis() >= controllerRunTime)
  {
    // Set time to run controller the next time
    controllerRunTime = millis() + CONTROLLER_PERIOD;

    // Start running controller
    PID.t[1]  = PID.t[0];
    PID.t[0]  = float(micros())/1000000.0;
    float dt = PID.t[0]-PID.t[1];

    // Save old angle value, read in new one.
    S.theta[1]      = S.theta[0];
    S.theta[0]      = readPotRadians(POT_PIN);

    // Get difference of position and setpoint
    PID.error[1] = PID.error[0];
    PID.error[0] = PID.setpoint - S.theta[0];
    float de = PID.error[0] - PID.error[1];

    // Calculate integral gain
    if (abs(PID.kp * PID.error[0]) >= 110) // Windup protection
      PID.errorsum = 0;
    else if ( PID.errorsat > 0 )
      PID.errorsum = constrain(PID.errorsum + PID.error[0] * dt,
                              -1*PID.errorsat,PID.errorsat);
    else
      PID.errorsum += PID.error[0];

    // Only run if enabled
    if(PID.enabled)
    {
      // Controller
      S.u[0] = PID.kp*PID.error[0] + PID.ki*PID.errorsum + PID.kd*de/dt;
      S.u[1] = setMotor(S.u[0]);
    }
  }

  // Print at defined intervals
  if(millis() >= serialWriteRunTime && !serWrite)
  {
    serialWriteRunTime = millis() + SERIAL_WRITE_PERIOD;

    // Write to serial port
//    Serial.print("\t");
//    Serial.print(rad2deg(S.theta[0]));
    Serial.print(",\t");
    Serial.print(rad2deg(PID.error[0]));
    Serial.print(",\t");
    Serial.println(S.u[1]);
    return;
  }

  // Stop motor if time has gone too long
  if(millis()>=CUTOFF_TIME+beginTime)
  {
    if(serWrite) Serial.println("CUTOFF_TIME reached");
    stopMotor();
    S.state = 0;
    if(serWrite) Serial.println("Moving to State 0");
  }

}

//////////////////////////////
// Custom defined functions //
//////////////////////////////

/*******************************************************************************
* float rad2deg(float radians)
*
* Exactly what you think it does
*******************************************************************************/
float rad2deg(float radians)
{
  return radians * 180.0/PI;
}

/*******************************************************************************
* float deg2rad(float degrees)
*
* Exactly what you think it does
*******************************************************************************/
float deg2rad(float degrees)
{
  return degrees * PI/180.0;
}

/*******************************************************************************
* float readPotRadians(int pin)
*
* Read pot angle in radians
*******************************************************************************/
float readPotRadians(int pin)
{
  // Standard read
  return deg2rad(DEG_PER_CNT*(analogRead(pin) - POT_OFFSET));
}

/*******************************************************************************
* void stopMotor()
*
* Stop the motor, set flag "serWrite"
*******************************************************************************/
void stopMotor()
{
  setMotor(0);
  PID.enabled = 0;
}

/*******************************************************************************
* float setMotor()
*
* Set the motor to a speed and direction
* Has saturation protection
*******************************************************************************/
float setMotor(float motorSpeed)
{
  // Set motor direction
  if (motorSpeed>0){digitalWrite(DIR_PIN,LOW);}
  else{digitalWrite(DIR_PIN,HIGH);}

  // Saturation protection
  if (motorSpeed>100){motorSpeed=100.0;}
  if (motorSpeed<-100){motorSpeed=-100.0;}

  // convert 0-100% to 0-255 duty cycle
  int motorDuty = map(abs(motorSpeed),0,100,0,255);

  // send PWM to motor
  analogWrite(PWM_PIN,motorDuty);

  // Return the actual controller output with saturation protection
  return motorSpeed;
}
