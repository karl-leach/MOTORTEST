#include <Arduino.h>
#include "X27Motor.h"

X27Motor motor; // uses default pins

void setup() {
	Serial.begin(115200);
	delay(100);
	Serial.println("X27Motor demo: begin and homing (610 steps anticlockwise)");
	motor.setMinMaxValue(0, 120);
	motor.setFullTravelSteps(600);
	motor.setStepDelay(1); // minimal delay for max speed; adjust if needed
	motor.begin(); // performs homing

}

void loop() {
	Serial.println("Move to 60 (midpoint of 0-120)");
	motor.setPosition(60);
	delay(4000);

	Serial.println("Move to 30");
	motor.setPosition(30);
	delay(4000);

	Serial.println("Move to 120 (end)");
	motor.setPosition(120);
	delay(4000);

	Serial.println("Move to 0 (start)");
	motor.setPosition(0);
	delay(4000);
}
