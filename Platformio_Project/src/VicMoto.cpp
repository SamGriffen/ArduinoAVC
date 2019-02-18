/**
 * Implements VicMoto.h
 * Class for controlling robot motors
 */

#include "VicMoto.h"

/**
 * Constructor for VicMoto class. Can be intialised with no parameters, and it should wor. There is no reason I can see for these pins to change, but the option is there.
 * @param left       Pin that controls the left motor speed (Default 11)
 * @param right      Pin that controls the left motor speed (Default 3)
 * @param left_back  Pin that controls whether the left motor moves forward or backward (Default 13)
 * @param right_back Pin that controls whether the left motor moves forward or backward (Default 12)
 */
VicMoto::VicMoto(short int left=11, short int right=3, short int left_back=13, short int right_back=12){
	// Set all the internal private variables
	_left = left;
	_right = right;
	_left_back = left_back;
	_right_back = right_back;
}

/**
 * Method for setting up the motor driver. Sets required pinmodes
 */
void VicMoto::begin(){
	// Set all the motor control pins to outputs
	pinMode(_left, OUTPUT);
	pinMode(_right, OUTPUT);
	pinMode(_left_back, OUTPUT);
	pinMode(_right_back, OUTPUT);
}

/**
 * Method for setting motor speeds
 * @param left  Speed of the left motor. -255 is backwards full speed, 0 is stationary, 255 is forward fullspeed
 * @param right Speed of the right motor. -255 is backwards full speed, 0 is stationary, 255 is forward fullspeed
 */
void VicMoto::setMotors(short int left, short int right){
	// Set the left motor state.
	analogWrite(_left, abs(left)); 			 // Set the control pin to the absolute value of left
	digitalWrite(_left_back, (left <= 0)); 	 // Set the back toggle to on if the value is negative

	// Set the right motor state.
	analogWrite(_right, abs(right)); 		 // Set the control pin to the absolute value of right
	digitalWrite(_right_back, (right <= 0)); // Set the back toggle to on if the value is negative
}