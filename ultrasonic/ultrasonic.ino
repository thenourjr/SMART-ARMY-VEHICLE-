// Include NewPing Library
#include <NewPing.h>

// Hook up HC-SR04 with Trig to Arduino Pin 9, Echo to Arduino pin 10
#define TRIGGER_PIN 25
#define ECHO_PIN 33
unsigned long dis = 0 ;
// Maximum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 400	

// NewPing setup of pins and maximum distance.
  NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
	Serial.begin(115200);
}

void loop() {
  dis = sonar.ping_cm() ; 
	Serial.print("Distance = ");
	Serial.print(dis);
	Serial.println(" cm");
	delay(500);
}
