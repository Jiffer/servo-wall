#include <Servo.h>

#define FIRST_PIN 2
#define NUM_SERVOS 15

Servo servos[NUM_SERVOS];
int pos[NUM_SERVOS];

const int sensorPin = A0;

int change= 10;

void setup()
{
  // start serial
  Serial.begin(9600);
  
  for( int i = 0; i < NUM_SERVOS; i++)
  servos[i].attach(FIRST_PIN + i);
  
}

void loop()
{
  //pos = analogRead(sensorPin);
  //int change = random(80, 100);
  //delay(500);
  if (Serial.available() > (NUM_SERVOS-1))
  {
    for(int i = 0; i < NUM_SERVOS; i++)
    {
      // read from serial port 
      pos[i] = Serial.read();
      // write angle to current servo
      servos[i].write(pos[i]);
      //servos[i].write(change);
    }
    
  }
  
}
