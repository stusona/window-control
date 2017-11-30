/******************************************************************************
 * Takes setpoints over bluetooth, moves window, then sends position data back.
 *
 * by Ashis Ghosh, Stuart Sonatina, Jacquie Nguyen, and Michael Oudenhoven
 *
 * Based on an example built by Adafruit: nRF51822 based Bluefruit LE modules
 *
 *****************************************************************************/

/*
    Please note the long strings of data sent mean the RTS pin is
    required with UART to slow down data sent to the Bluefruit LE!
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif


// Pin Definitions
#define FWD_PIN     5
#define REV_PIN     6
#define PWM_PIN     9  // Enable pin on Motor Driver
#define POT_PIN     A0 // Sense voltage on Potentiometer wiper
#define REED_PIN    A1 // Sense reed switch (LOW is open and HIGH is closed)


// Number of milliseconds before moveWindow times out
#define TIMEOUT   10000 // ms

// Constants
#define KP          -10
#define MAX_ERROR     2   // allowable steady state error (percent)
#define REED_OFFSET   2   // Amount window needs to move past triggering
                          // of reed switch (percent)

/** Create the bluefruit object
 * Hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user-selected
 * CS/IRQ/RST
 */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */
int32_t customServiceID;
int32_t positionCharID;
int32_t setpointCharID;

// Define potentiometer scale (can be between 0 and 1023)
// This needs to be calibrated. Let's start with a small range in the middle.
float pot_min = 0; // adc counts
float pot_max = 1023; // adc counts
float pot_scale  = 100.0/(pot_max-pot_min); // percent / adc counts

//Initialize window control variables
int32_t pos_current = 0;
int32_t pos_setpoint = 0;
boolean valueChanged = false;


/**************************************************************************/

void setup(void)
{
  // Initialize pins
  pinMode(FWD_PIN, OUTPUT);
  pinMode(REV_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(REED_PIN, INPUT);

  // Make sure stepper driver is off
  digitalWrite(PWM_PIN, LOW);

  // Intialize window control variables to starting state
  pos_current = readPosition();
  pos_setpoint = pos_current;
  Serial.print("Starting window position: ");
  Serial.println(pos_current);

  // Let's determine if we want to home the window right here
  // *move window until reed switch triggers, then move additional offset*
  // *call that position "pot_min"


  //////////////////////// BEGIN ADAFRUIT BLUETOOTH CODE ///////////////////////
  // Set up the BLE module
  while (!Serial); // required for Flora & Micro
  delay(500);

  boolean success;

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Custom AURAI Window Module"));
  Serial.println(F("---------------------------------------------------"));

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Aurai Window Module': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Aurai Window Mod")) ) {
    error(F("Could not set device name?"));
  }


  /*Add custom service */
    Serial.println(F("Adding the Custom Service definition "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID128=20-ff-77-b5-75-c2-45-b2-85-1d-b4-2b-04-1d-35-c3"), &customServiceID);
  if (! success) {
    error(F("Could not add custom service"));
  }

  /* Position characteristic */
  Serial.println(F("Adding the position characteristic  "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=57-12-04-BC-B3-96-42-97-A5-13-31-1C-45-71-C0-23, PROPERTIES=0x02, MIN_LEN=1, MAX_LEN=4, VALUE=1000000000"), &positionCharID);
    if (! success) {
    error(F("Could not add position characteristic"));
  }

   /* Setpoint characteristic */
  Serial.println(F("Adding the setpoint characteristic  "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=19-9b-42-78-8c-07-4e-71-bd-0d-7f-4e-1f-d5-76-d2, PROPERTIES=0x08, MIN_LEN=1, MAX_LEN=4, VALUE=1000000000"), &setpointCharID);
    if (! success) {
    error(F("Could not add setpoint characteristic"));
  }

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Custom Service UUID to the advertising payload: "));
  //ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-06-b5-77-0a-18") );
  //ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-b5-77-0a-18") );
  //ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();

////////////////////////// END ADAFRUIT BLUETOOTH CODE /////////////////////////
}


/**************************************************************************/

int incomingByte = 0;

void loop(void)
{
  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */

//  ble.sendCommandWithIntReply(F("AT+GATTCHAR=2"), &window_setpoint);
//  Serial.print("window_setpoint2: ");
//  Serial.println(window_setpoint);

  Serial.println("----------***--------------");
  // Print current position to serial monitor
  pos_current = readPosition();
  Serial.print("Window is at: ");
  Serial.println(pos_current);

  // send position to Pico
  broadcastPosition();

  Serial.println("----------------");

  if (Serial.available() > 0) {
    // read the incoming byte:
     incomingByte = Serial.parseInt();

     // say what you got:
     Serial.print("I received: ");
     Serial.println(incomingByte);

     pos_setpoint = incomingByte;

     Serial.print("pos_setpoint: ");
     Serial.println(pos_setpoint);
     ble.print( F("AT+GATTCHAR=") );
     ble.print( setpointCharID );
     ble.print(F(","));
     ble.print(pos_setpoint, HEX);
     ble.println( F("-00-00-00") );
  }
  else{
    readSetpoint();
  }

  Serial.println("----------------");
  // move the window to setpoint
  moveWindow(pos_setpoint);

  // window should be at setpoint now. Send position to Pico.
  broadcastPosition();

  /* Delay before next measurement update */
  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////
//                          CUSTOM FUNCTIONS                                 //
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * int moveWindow()
 *
 * Takes in a setpoint and moves window until error is below max error.
 * Returns 0 upon success.
******************************************************************************/
int moveWindow(int setpoint) {

  // Start a timer for the timeout
  unsigned long startTime = millis();
  unsigned long runTime = millis() - startTime;

  // Read pot position and print to serial monitor
  pos_current = readPosition();
  Serial.print("Move from ");
  Serial.print(pos_current);
  Serial.print(" to ");
  Serial.println(setpoint);

  // While error is above max and run time is less than timeout, drive motor
  // towards setpoint
  while (abs(pos_current - setpoint) > MAX_ERROR
         && (runTime < TIMEOUT)) {

    pos_current = readPosition(); // percent

    // set motor speed and print to serial monitor
    // Serial.println(setMotor(KP*(pos_current - setpoint)));
    setMotor(KP*(pos_current - setpoint));

    // update runTime
    runTime = millis() - startTime; // ms

    delay(50); // 20 Hz
  }

  // Stop motor
  setMotor(0);

  // If runTime is close to timeout, then it probably timed out.
  // Print whether window finished moving to position or timed out.
  if (runTime > (TIMEOUT - 100)) {
    Serial.println("moveWindow timed out");
    return 1;
  }
  else {
    Serial.print("Finished moving to ");
    Serial.println(setpoint);
    return 0;
  }
}

/******************************************************************************
 * int readPosition()
 *
 * Returns position of window from 0% (closed) to 100% (open).
 * Also checks for out of range.
 *****************************************************************************/
int readPosition() {
  // get position in adc counts 0-1023
  pos_current = analogRead(POT_PIN);

  // If position is below min or above max, something is wrong
  if ((pos_current < pot_min) || (pos_current > pot_max)) {
//    Serial.println("Window out of range");
  }

  pos_current -= pot_min;   // subtract minimum from position
  pos_current = 100 - pos_current*pot_scale; // convert from adc counts to percent
  return pos_current;
}

/******************************************************************************
 * void broadcastPosition()
 *
 * Sends position back to Pico
 *****************************************************************************/
void broadcastPosition() {
  pos_current = readPosition();
  ble.print( F("AT+GATTCHAR=") );
  ble.print( positionCharID );
  ble.print(F(","));
  ble.print(pos_current, HEX);
  ble.println( F("-00-00-00") );
  ble.print( F("AT+GATTCHAR=") );
  ble.println( positionCharID );
  Serial.print("Window position: ");
  Serial.println(pos_current);
  return;
}

/******************************************************************************
 * void readSetpoint()
 *
 * Reads setpoint from Pico
 *****************************************************************************/
void readSetpoint() {
//  ble.print( F("AT+GATTCHAR=") );
//  ble.println( setpointCharID );

  ble.sendCommandWithIntReply(F("AT+GATTCHAR=2"), &pos_setpoint);
  ble.sendCommandWithIntReply(F("AT+GATTCHAR=2"), &pos_setpoint);
  pos_setpoint = (pos_setpoint/10)*16 + pos_setpoint%10;
  Serial.print("pos_setpoint: ");
  Serial.println(pos_setpoint);

  /* Check if command executed OK */
  if ( !ble.waitForOK() ) {
    Serial.println(F("Failed to get response!"));
  }
}

/*******************************************************************************
 * float setMotor()
 *
 * Set the motor to a speed and direction
 * Has saturation protection
 ******************************************************************************/
float setMotor(float motorSpeed)
{
  // Set motor direction
  if (motorSpeed>=0) {
    digitalWrite(FWD_PIN,HIGH);
    digitalWrite(REV_PIN,LOW);
  }
  else {
    digitalWrite(FWD_PIN,LOW);
    digitalWrite(REV_PIN,HIGH);
  }

  // Saturation protection
  if (motorSpeed >  100) {motorSpeed =  100.0;}
  if (motorSpeed < -100) {motorSpeed = -100.0;}

  // convert 0-100% to 0-255 duty cycle
  int motorDuty = map(abs(motorSpeed),0,100,0,255);

  // send PWM to motor
  analogWrite(PWM_PIN,motorDuty);

  // Return the actual controller output with saturation protection
  return motorSpeed;
}
