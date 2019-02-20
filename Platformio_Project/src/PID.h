/**
 * Class for managing PID control. Implemented in a general fashion, so that it can potentially be used for line following and wall following.
 */

#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID{
	public:
		PID(double kp, double ki, double kd); // Constructor for PID controller. Takes Kp, Ki, and Kd values
		double process(int target, int reading); // Processes the PID algorithm. Takes the target value, and the read value (For example this may be for line following, with target being 500, and the read value being 250). Outputs the output value. Which is essentially the error multiplied by Kp, the error diff multiplied by Kd, and the total error multipled by Ki
		void reset(); // Resets all internal values
		void setConstants(double kp, double ki, double kd); // Resets the internal constants
	private:
		// Store the gains for P, I, and D
		double _kp;
		double _ki;
		double _kd;

		double _error; // Store the current error

		unsigned long _last_time; 	// Store the time of last error calculation
		int _previous_error;		// Store the previous error value
		unsigned long _total_error; // Store the total error over time

		double _output; // Store the output value
};

#endif