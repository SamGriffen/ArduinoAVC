#include <Arduino.h> 				// Include the Arduino library - platformio needs this
#include "QTRSensors.h" 			// Include the library for controlling the sensor array (https://www.pololu.com/docs/0J19/3)
#include "VicMoto.h"				// Include the small utility functions for motor control
#include "PID.h"					// Include the PID controller code

#define NUM_SENSORS             8  	// number of sensors used

#define STATUS_LED				A2	// Pin for the status LED

#define TESTING					true // Flag for testing - Sets serial up

#define FOLLOW_MAX_SPEED 255
#define FOLLOW_MIN_SPEED -255

// Pins for sensor readings
#define LEFT_IR					A5
#define MID_IR					A4
#define RIGHT_IR				A3

// Initialise the sensor array - As far as I can tell it is some form of Pololu QTR-RC array. Documentation of the library found here: https://www.pololu.com/docs/0J19/3

// Definitions for the potentiometers that may be used for tuning
// #define P_PIN A5
// #define D_PIN A4

// Array to store testing states.
/**
 * testing[0] - Stores whether to print motor setting data in the main loop
 * testing[1] - Stores whether to print mid IR sensor readings
 *
 */
bool testing[2] = {0, 1};

QTRSensorsRC line((unsigned char[]) {2, 4, 5, 6, 7, 8, 9, 10}, NUM_SENSORS);

unsigned int sensor_values[NUM_SENSORS]; // Values from the line sensor

// Initialise the motor driver
VicMoto motors;

// Initialise the line following PID
// PID linePID(1.9, 0, 5);
// PID linePID(1.8, 0, 4.8);
PID linePID(0.22, 0, 0.1);
// PID linePID(1.09, 0, 0.2);


// Function declarations for line following control
void calibrate(int motor_speed); // Calibrates the line sensor
void findLine(int speed); // Finds the line
void follow(int speed); // Follows the line

// Functions for obstacle avoidance
bool isBlocked(); // Returns true if the robot has been obstructed, false otherwise
void avoidObstacle(); // Manouvers the object around an obstacle. Looks for the line to return and for there to be nothing in front of the robot.

double mapDouble(double value, double fromLow, double fromHigh, double toLow, double toHigh); // Map a number doing double math

void setup(){

	// Only use for potentiometer tuning
	// pinMode(P_PIN, INPUT);
	// pinMode(D_PIN, INPUT);

	// Setup the IR sensor pins
	pinMode(LEFT_IR, INPUT);
	pinMode(MID_IR, INPUT);
	pinMode(RIGHT_IR, INPUT);

	if(true){
		Serial.begin(9600); // Begin serial if we are testing

		//Used for reading the potentiometers that may be used for tuning
		// Serial.print("KP: ");
		// Serial.println(mapDouble(analogRead(P_PIN), 0.0, 1023.0, 0.0, 5.0));
		// Serial.print("KD: ");
		// Serial.println(mapDouble(analogRead(D_PIN), 0.0, 1023.0, 0.0, 5.0));
	}

	motors.begin(); // Setup the motor driver

	pinMode(STATUS_LED, OUTPUT);



	calibrate(65); // Calibrate the line sensor
	findLine(60); // Find the line
}

void loop(){
	// For potentiometer tuning
	// linePID.setConstants(mapDouble(analogRead(P_PIN), 0.00, 1023.0, 0.0, 0.5), 0.0, mapDouble(analogRead(D_PIN), 0.0, 1023.0, 0.0, 0.5));
	follow(53); // Follow the line
}

/**
 * Function to actually follow the line
 * @param speed Base speed to folow with
 */
void follow(int speed){
	int dv = linePID.process(3500, line.readLine(sensor_values))/10; // Get the result of a PID process - Divide by 5 simply to make the constants larger numbers
	if(TESTING && testing[0]){
		Serial.print("Left: ");
		Serial.print(speed+dv);
		Serial.print(" Right: ");
		Serial.print(speed-dv);
		Serial.print(" DV: ");
		Serial.println(dv);
	}
	if(TESTING && testing[1])Serial.println(analogRead(MID_IR));
	motors.setMotors(constrain(speed+dv, FOLLOW_MIN_SPEED, FOLLOW_MAX_SPEED), constrain(speed-dv, FOLLOW_MIN_SPEED, FOLLOW_MAX_SPEED)); // Set motor speeds

}

/**
 * Function for finding the line that is being followed
 * @param speed The speed to set motors to to spin around to the line
 */
void findLine(int speed){
	motors.setMotors(speed, -1*speed); // Turn the motors on to spin the robot
	int linePos = line.readLine(sensor_values); // Read where the line is
	while(linePos < 2500 || linePos > 4500){ // If the line is not centered, reread the line, and continue moving
		linePos = line.readLine(sensor_values);
	}
	motors.setMotors(0, 0); // The line is found, stop
	linePID.reset(); // Reset the PID controller
}

/**
 * Function to calibrate the line underneath the robot. Status LED will glow while in this phase. Before initialising this phase, ensure the robot is sat centered on the line.
 * @param motor_speed	Max speed to set the motors to during calibration
 */
void calibrate(int motor_speed){
	digitalWrite(STATUS_LED, HIGH);    // turn on status LED to indicate we are in calibration mode

	// Variables for handling sweep movement
	int cur_speed = 0;
	bool going_up = true;

	int threshold = 50; // Threshold where motors stop

	// motors.setMotors(200,-200);

	for (int i = 0; i < 400; i++){
		if(going_up && cur_speed >=motor_speed){ // If we have overshot the max, or are at the max, flip dirtection
			going_up = false;
		}
		else if(!going_up && cur_speed <= -1*motor_speed){ // Same, but with zero.
			going_up = true;
		}

		if(abs(cur_speed) < threshold){ // This should skip the threshold between -40 and 40 where the motors freeze up
			cur_speed = threshold * (going_up?1:-1); // The ternary flips the sign. If the speed is currently going up, it should set current speed to 40, if the speed is going down, it should set it to -40
		}

		cur_speed += (going_up?1:-1); // Modify the speed

		motors.setMotors(cur_speed, -1*cur_speed);

		line.calibrate();       // reads all sensors 10 times
	}
	motors.setMotors(0, 0); // End sweep
	digitalWrite(STATUS_LED, LOW);     // turn off status LED to indicate we are through with calibration

	if(TESTING){ // If in testing, print the values found
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
}


/* Functions for moving around obstacles */

/** Method to determine whether the robot is blocked by an obstacle. Returns true if blocked, false otherwise. Reads from the front sensor
*	As far as my testing shows, the reading increases as objects get closer
*	Max - Seems to be about 620 @ ~2cm away from sensor
* 	Min - 0, however radiant IR seems to float it up to a reading of ~15
*/
bool isBlocked(){
	short threshold = 266; // Store the maximum number for an object that is MORE than 100mm away from the sensor
	return analogRead(MID_IR) > threshold; // Return a boolean representing whether the robot has been obstructed
}

/**
 * Function to move the robot around an obstacle
 */
void avoidObstacle(){
	//
}


/**

/**
 * Acts like the Arduino map function, but does double math
 * @param  value    [description]
 * @param  fromLow  [description]
 * @param  fromHigh [description]
 * @param  toLow    [description]
 * @param  toHigh   [description]
 * @return          [description]
 */
double mapDouble(double value, double fromLow, double fromHigh, double toLow, double toHigh){
	double prop = value/(fromHigh - fromLow);
	return (prop * (toHigh - toLow))+toLow;
}