const int stepPin = 3; 
const int dirPin = 4; 
const int sleepPin = 5;
const int period = 950; 

void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(sleepPin,OUTPUT);
}

void loop() {
  // enable driver and set direction
  digitalWrite(sleepPin, HIGH);
  digitalWrite(dirPin,HIGH); 
  
  // 200 pulses is one full cycle 
  for(int x = 0; x < 2000; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(period); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(period); 
  }

  digitalWrite(sleepPin, LOW);
  
  delay(3000); // One second delay
}
