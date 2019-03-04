#include <Arduino.h> 				// Include the Arduino library - platformio needs this
#include "QTRSensors.h" 			// Include the library for controlling the sensor array (https://www.pololu.com/docs/0J19/3)
#include "VicMoto.h"				// Include the small utility functions for motor control
#include "PID.h"					// Include the PID controller code

#define NUM_SENSORS             6  	// number of sensors used

#define STATUS_LED				A2	// Pin for the status LED

#define TESTING					true // Flag for testing - Sets serial up

#define FOLLOW_MAX_SPEED 255
#define FOLLOW_MIN_SPEED -255

#define IR_THRESHOLD 266 			// Store the maximum number for an object that is MORE than 100mm away from the sensor
#define IR_OBJECT_THRESHOLD 250		// IR reading to target when following an object

#define IR_LEFT_THRESHOLD 310 		// IR Threshold for the left IR sensor - Used for obstacle avoidance
#define IR_RIGHT_THRESHOLD 310 		// IR Threshold for right sensor - Used for obstacle avoidance

// Pins for sensor readings
#define LEFT_IR					A5
#define MID_IR					A4
#define RIGHT_IR				A3

// Pins for DIP Switch
#define DIP_1	A1
#define DIP_2	A0

// Initialise the sensor array - As far as I can tell it is some form of Pololu QTR-RC array. Documentation of the library found here: https://www.pololu.com/docs/0J19/3
QTRSensorsRC line((unsigned char[]) {4, 5, 6, 7, 8, 9}, NUM_SENSORS);

unsigned int sensor_values[NUM_SENSORS]; // Values from the line sensor

bool calibrated = false; // Stores whether or not a calibration sequence has been performed

// Definitions for the potentiometers that may be used for tuning
// #define P_PIN A5
// #define D_PIN A4

// Array to store testing states.
/**
 * testing[0] - Stores whether to print motor setting data in the main loop
 * testing[1] - Stores whether to print mid IR sensor readings
 * testing[2] - Stores whether to print data from the right sensor while rotating to be in line with an obstacle
 * testing[3] - Stores whether to print the speed difference in obstacleAvoider mode
 * testing[4] - Stores whether to print debug data in the lineFollowerAvoid function
 */
bool testing[5] = {0, 0, 0, 0, 1};

// Initialise the motor driver
VicMoto motors;

// Initialise the line following PID
// PID linePID(1.9, 0, 5);
// PID linePID(1.8, 0, 4.8);
PID linePID(0.28, 0, 0.15);
// PID linePID(1.09, 0, 0.2);


// Initialise the wall following PID
PID wallPID(0.25,0,0.15);

// Funtion declarations for different programs
void lineFollower();		// Program for following a line - Program 0
void obstacleAvoider();		// Program for roaming artound avoiding obstacles - Program 1
void lineFollowerAvoid();	// Program for line following, avoiding any obstacles on the line - Program 2

// Function declarations for line following control
void calibrate(int motor_speed); // Calibrates the line sensor
void findLine(int speed); // Finds the line
void followLine(int speed); // Follows the line

// Functions for obstacle avoidance
bool isBlocked(int thres=IR_THRESHOLD); // Returns true if the robot has been obstructed, false otherwise
void avoidObstacle(int baseSpeed); // Manouvers the object around an obstacle. Looks for the line to return and for there to be nothing in front of the robot.

double mapDouble(double value, double fromLow, double fromHigh, double toLow, double toHigh); // Map a number doing double math

void printSensorArray(); // Function to print out the sensor array for debugging

short readDIPSwitch(){ // Small helper function for reading the status of the DIPSwitch
	return !digitalRead(DIP_1) | (!digitalRead(DIP_2) << 1);
}

void setup(){

	// Only use for potentiometer tuning
	// pinMode(P_PIN, INPUT);
	// pinMode(D_PIN, INPUT);

	// Setup the DIP Swtich
	pinMode(DIP_1, INPUT_PULLUP);
	pinMode(DIP_2, INPUT_PULLUP);

	// Setup the IR sensor pins
	pinMode(LEFT_IR, INPUT);
	pinMode(MID_IR, INPUT);
	pinMode(RIGHT_IR, INPUT);

	if(TESTING){
		Serial.begin(9600); // Begin serial if we are testing
	}

	motors.begin(); // Setup the motor driver

	pinMode(STATUS_LED, OUTPUT);

	//calibrate(65); // Calibrate the line sensor
	//findLine(60); // Find the line
}

void loop(){
	// For potentiometer tuning
	// linePID.setConstants(mapDouble(analogRead(P_PIN), 0.00, 1023.0, 0.0, 0.5), 0.0, mapDouble(analogRead(D_PIN), 0.0, 1023.0, 0.0, 0.5));

	// Set the correct program based on the DIP Switch input
	switch(readDIPSwitch()){
		case 0:
			if(!calibrated)calibrate(60);
			lineFollower();
			break;
		case 1:
			obstacleAvoider();
			break;
		case 2:
			if(!calibrated)calibrate(60);
			lineFollowerAvoid();
			break;
	}
}

/**
 * Functions for what are essentially seperate programs on the robot
 *
 * The dipswitch is set as a binary number - Giving 4 different programs
 */

/**
 * Program 0
 * Method for controlling the line follower.
 */
void lineFollower(){
	followLine(53); // Follow the line (Yep, this is really all it uses... This function is probably a little bit invalid. Oh well)
}

/**
 * Program 1
 * Roams around an open space avoiding obstacles
 */
void obstacleAvoider(){
	short baseSpeed = 100;

	if(isBlocked()){ // If the robot is obstructed, dodge the obstacle

		bool leftClear = analogRead(LEFT_IR) < analogRead(RIGHT_IR); // Detect which side is the clearest to move into. High reading means nearby object.

		while(isBlocked()){ // While there is an obstacle, avoid it. This will ignore any program swaps until the object is avoided
			motors.setMotors(baseSpeed * (leftClear?-1:1), baseSpeed * (leftClear?1:-1)); // Set the motors. ternary operation should set rotate direction
		}
	}
	else{
		// Calculate the difference between the readings from the left and right sensors if they are over the threshold - This essentially allows the robot to steer itself away from obstacles. If this value is positive it means that tehre is a bigger reading on the left. If it is negative, there is a bigger reading on the right
		short dv = constrain(analogRead(LEFT_IR) - IR_LEFT_THRESHOLD, 0, 1023) - constrain(analogRead(RIGHT_IR) - IR_RIGHT_THRESHOLD, 0, 1023)/4;

		if(TESTING && testing[3])Serial.println(dv);

		motors.setMotors(baseSpeed + dv, baseSpeed - dv);
	}
}

/**
 * Program 2
 * Follows a line, avoiding obstacles that are on the line
 */
void lineFollowerAvoid(){
	if(TESTING && testing[4])Serial.println(analogRead(MID_IR));
	if(isBlocked(IR_THRESHOLD + 150)){ // If the robot encounters an obstacle, avoid it
		if(TESTING && testing[4]){
			Serial.println("BLOCKED");
		}

		while(analogRead(MID_IR) > (IR_THRESHOLD)){
			motors.setMotors(-53, -53);
		}

		motors.setMotors(0, 0);

		avoidObstacle(53);
	}
	else{
		followLine(53); // Follow the line as usual
	}
}

/**
 * Function to actually follow the line
 * @param speed Base speed to folow with
 */
void followLine(int speed){
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

	calibrated = true; // The robot is now calibrated
}


/* Functions for moving around obstacles */

/** Method to determine whether the robot is blocked by an obstacle. Returns true if blocked, false otherwise. Reads from the front sensorWhile loops throughout the code will cause dela
*	As far as my testing shows, the reading increases as objects get closer
*	Max - Seems to be about 620 @ ~2cm away from sensor
* 	Min - 0, however radiant IR seems to float it up to a reading of ~15
*
* 	@param thres the IR threshold to count as blocked
*/
bool isBlocked(int thres = IR_THRESHOLD){
	return analogRead(MID_IR) > thres; // Return a boolean representing whether the robot has been obstructed
}

/**
 * Function to move the robot around an obstacle
 *
 * @param baseSpeed The base speed to move the robot around the obstacle at
 */
void avoidObstacle(int baseSpeed){
	wallPID.reset(); // Reset the PID controller for the wall follower

	motors.setMotors(-60,0); // Set the robot spinning
	while(sensor_values[NUM_SENSORS-1 -1] < 500){ // Align the leftmost sensor with the line
		line.readCalibrated(sensor_values);
		printSensorArray();
	}

	delay(1000);

	// Align the rightmost sensor to the line
	motors.setMotors(0, 60);
	while(sensor_values[0] < 500){
		line.readCalibrated(sensor_values);
		Serial.println(sensor_values[0]);
	}

	delay(5000);

	// Once the loop ends, we need to move off the line
	while(sensor_values[0] > 500 && sensor_values[NUM_SENSORS-1 -1] > 500){
		motors.setMotors(50, 50);
	}

	// Start following the object

	int pid_reading = 0; // Initialise the PID variable

	while(sensor_values[2] < 500 && sensor_values[3] < 500){ // MAKE THIS NOT WHILE TRUE
		line.readCalibrated(sensor_values); // Read the line sensor

		pid_reading = wallPID.process(IR_OBJECT_THRESHOLD, analogRead(RIGHT_IR)); // Get the first PID reading

		Serial.print("SENSOR: ");
		Serial.print(analogRead(RIGHT_IR));
		Serial.print("  PID: ");
		Serial.println(pid_reading);

		motors.setMotors(baseSpeed + pid_reading, baseSpeed - pid_reading); // Set the motors to account for PID
	}
}

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

// Prints out the sensor array
void printSensorArray(){
	Serial.print("{");
	for(int i = 0; i < NUM_SENSORS-1; i++){
		Serial.print(sensor_values[i]);
		Serial.print(",");
	}
	Serial.print(sensor_values[NUM_SENSORS-1]);
	Serial.println("}");
}