                                     #include <Arduino.h> 				// Include the Arduino library - platformio needs this
#include "QTRSensors.h" 			// Include the library for controlling the sensor array (https://www.pololu.com/docs/0J19/3)

#define NUM_SENSORS             8  	// number of sensors used

/*
 * As far as I can tell from my experiments with the motor driver, you tell the motor the speed you want it to run at, and then pull another pin high to get the motor to rotate backwards - It is entirely possible that I have this completely wrong, but it is the best I can work out.
 *
 * The constants below set the pins to control each motor (PWM pins), and the pins that set the motor backwards
 */

#define LEFT		11
#define LEFT_BACK	13

#define RIGHT		3
#define RIGHT_BACK	12

// Initialise the sensor array - As far as I can tell it is some form of Pololu QTR-RC array. Documentation of the library found here: https://www.pololu.com/docs/0J19/3
QTRSensorsRC line((unsigned char[]) {2, 4, 5, 6, 7, 8, 9, 10}, NUM_SENSORS);

void setup(){
	pinMode(LEFT, OUTPUT);
	pinMode(LEFT_BACK, OUTPUT);
	pinMode(RIGHT, OUTPUT);
	pinMode(RIGHT_BACK, OUTPUT);


	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
	for (int i = 0; i < 600; i++){
		line.calibrate();       // reads all sensors 10 times
	}
	digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

	// print the calibration minimum values measured when emitters were on
	Serial.begin(9600);
	Serial.println("Minimum Values:");
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(line.calibratedMinimumOn[i]);
		Serial.print(' ');
	}
	Serial.println();

	Serial.println("Maximum Values:");
	// print the calibration maximum values measured when emitters were on
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(line.calibratedMaximumOn[i]);
		Serial.print(' ');
	}
	Serial.println();
	Serial.println();
}

void loop(){
	unsigned int data[NUM_SENSORS]; // Declare an array for storing line data

	int position = line.readLine(data);

	Serial.println(position);
	digitalWrite(LEFT, HIGH);
	digitalWrite(LEFT_BACK, LOW);
	digitalWrite(RIGHT, HIGH);
	digitalWrite(RIGHT_BACK, LOW);
}