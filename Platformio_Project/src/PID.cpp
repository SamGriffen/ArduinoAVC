/**
 * Implements the PID class. Creates a PID controller that can be used with multiple things
 */
#include "PID.h"

/**
 * PID Constructor
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
PID::PID(double kp, double ki, double kd){
	_kp = kp;
	_ki = ki;
	_kd = kd;

	// Initialise the error to 0
	_error = 0;

	_last_time = millis(); // Set the last time a calculation was done
	_previous_error = 0; // Initialise the previous error
	_total_error = 0;
};

/**
 * Processes the PID algorithm. Takes a target value, a read (read) value, calculates error, and returns and output modifier, that can be used to modify things such as motor speed
 * @param  target The target value
 * @param  read   The actual value
 * @return        The error multiplied by Kp, total error multiplied by Ki, and the rate of change of error multiplied by Kd
 */
double PID::process(int target, int read){
	_error = (double)(target - read); // Calculate the current error value

	_output = _error * _kp; // Set the proportional component of output

	_total_error += _error; // Add the new error to the total
	_output += _total_error*_ki; // Include the integral component of output

	_output += (_error - _previous_error)/(millis() - _last_time); // Include the derivative component of the output

	// Update the variables used for Kd
	_last_time = millis();
	_previous_error = _error;

	return _output;
};

/**
 * Resets all internal values - Used to ensure that the last time is now, and the last error is zero. Call after calibration, but only call once
 */
void PID::reset(){
	_last_time = millis(); // Set the last time to now, so essentially zero
	_previous_error = 0;
	_total_error = 0;
}

/**
 * Method to change the internal constants
 * @param kp [description]
 * @param ki [description]
 * @param kd [description]
 */
void PID::setConstants(double kp, double ki, double kd){
	_kp = kp;
	_ki = ki;
	_kd = kd;
}