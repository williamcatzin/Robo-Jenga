#include <Arduino.h>
#include <ESP32Servo.h>

// create servo object to control a servo
Servo GRIPPER;
int OPEN_POSITION = 0;
int CLOSED_POSITION = 120;
char COMMAND = 'A';    
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
#define SERVO_PIN 15
#define RXD2 16
#define TXD2 17
 
void setup() {
	Serial.begin(9600);
	Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	// standard 50 hz servo
	GRIPPER.setPeriodHertz(50);
	// attaches the servo on pin 18 to the servo object
	GRIPPER.attach(SERVO_PIN, 500, 2400);
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
}
 
void loop() {
	if (Serial2.available()) {
		COMMAND = char(Serial2.read());
		if (COMMAND == 'P') {
			GRIPPER.write(CLOSED_POSITION);
			delay(500);
			GRIPPER.write(30);
			// Serial2.print("Received Command!");
			// Serial2.print("\n");
		} else if (COMMAND == 'D'){
			GRIPPER.write(OPEN_POSITION);
			// Serial2.print("Received Command!");
			// Serial2.print("\n");
		} 
	}
}