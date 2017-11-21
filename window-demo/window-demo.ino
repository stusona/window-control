/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

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
#define STEP_PIN    5
#define DIR_PIN     6
#define SLEEP_PIN   9  // HIGH enables driver and LOW puts it to sleep
#define POT_PIN     A0 // Sense voltage on Potentiometer wiper
//#define REED_PIN    A1 // Sense reed switch (LOW is open and HIGH is closed)

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define STEPS_PER_REV 200
#define RPM 10
#define PERIOD 3000 // microseconds


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */
int32_t customServiceID;
int32_t positionCharID;
int32_t setpointCharID;

// Define potentiomete scale
float pot_max = 5;
float pot_min = 0;
float pot_scale  = 100/(pot_max-pot_min);


//Initialize window control variables
int32_t window_pos = 0;
int32_t window_setpoint = 0;
boolean valueChanged = false;


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  // Initialize pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SLEEP_PIN,OUTPUT);

  // Make sure stepper driver is off
  digitalWrite(SLEEP_PIN, LOW);

  
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
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
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

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Aurai Window Module")) ) {
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
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=57-12-04-BC-B3-96-42-97-A5-13-31-1C-45-71-C0-23, PROPERTIES=0x08, MIN_LEN=1, MAX_LEN=4, VALUE=1000000000"), &positionCharID);
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
//  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-06-b5-77-0a-18") );
//ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-b5-77-0a-18") );
  //ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );
  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();

//  intialize window control variables to starting state
  window_pos = readPosition();
  window_setpoint = window_pos;
  Serial.print("Starting window position: ");
  Serial.println(window_pos);
}

void loop(void)
{
  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */

  window_pos = readPosition();
  Serial.print("Window is at: ");
  Serial.println(window_pos);
  broadcastPosition();

  readSetpoint();  

  moveMotor(window_setpoint);

  broadcastPosition();
  
  /* Delay before next measurement update */
  delay(1000);
}

void moveMotor(int setpoint){
  int timeout = 0;
  int currentPos = readPosition();
  Serial.print("Move to ");
  Serial.print(setpoint);
  Serial.print(" from ");
  Serial.println(currentPos);
  
  while (abs(currentPos - setpoint) > 2){
     // Enable stepper driver and set direction
      digitalWrite(SLEEP_PIN,HIGH);

      if ((currentPos - setpoint) > 0){
        digitalWrite(DIR_PIN,LOW);
      }
      else{
        digitalWrite(DIR_PIN,HIGH);
      }

      // turn 10 times
   
      digitalWrite(STEP_PIN,HIGH);
      delayMicroseconds(PERIOD/2);
      digitalWrite(STEP_PIN,LOW);
      delayMicroseconds(PERIOD/2);
     
      currentPos = readPosition();
      
      timeout++;
      if (timeout>10000){
        Serial.println("moveMotor timed out.");
        break;
      }
  }
  // Turn stepper driver off
  digitalWrite(SLEEP_PIN, LOW);  

  Serial.print("Finished moving to ");
  Serial.println(setpoint);
  
  return;
}

int readPosition(){
  window_pos = analogRead(POT_PIN);
  window_pos -= pot_min;
  window_pos *= pot_scale;
  return window_pos;
}

void broadcastPosition(){
  int currentPos = readPosition();
  ble.print( F("AT+GATTCHAR=") );
  ble.print( positionCharID );
  ble.print(F(","));
  ble.print(currentPos, HEX);
  ble.println( F("-00-00-00") );
  return;
}

void readSetpoint(){
//  ble.print( F("AT+GATTCHAR=") );
//  ble.println( setpointCharID );

  ble.sendCommandWithIntReply(F("AT+GATTCHAR=2"), &window_setpoint);
  Serial.print("window_setpoint: ");
  Serial.println(window_setpoint);

  /* Check if command executed OK */
  if ( !ble.waitForOK() )
  {
    Serial.println(F("Failed to get response!"));
  }
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
  if (motorSpeed>0)
  {
    digitalWrite(FWD_PIN,HIGH);
    digitalWrite(REV_PIN,LOW);
  }
  else
  {
    digitalWrite(FWD_PIN,LOW);
    digitalWrite(REV_PIN,HIGH);
  }
  
  // Saturation protection
  if (motorSpeed> 100){motorSpeed= 100.0;}
  if (motorSpeed<-100){motorSpeed=-100.0;}

  // convert 0-100% to 0-255 duty cycle
  int motorDuty = map(abs(motorSpeed),0,100,0,255);

  // send PWM to motor
  analogWrite(PWM_PIN,motorDuty);

  // Return the actual controller output with saturation protection
  return motorSpeed;
}

