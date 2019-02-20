/**
 * Class for controlling the VicMoto motor board - Or as close as I can get to controlling it.
 *
 * As far as I can tell from my experiments with the motor driver, you tell the motor the speed you
 * want it to run at, and then pull another pin high to get the motor to rotate backwards - It is
 * entirely possible that I have this completely wrong, but it is the best I can work out.
 */
#ifndef VicMoto_h
#define VicMoto_h

#define VICMOTO_MAX_SPEED 255
#define VICMOTO_MIN_SPEED 0

#include <Arduino.h>

class VicMoto{
	public:
		VicMoto(short int left=11, short int right=3, short int left_back=13, short int right_back=12); // Initialiser. Sets internal pin numbers
		void begin(); // Starts the motor driver board. Sets pinmodes.
		void setMotors(short int left, short int right); // Function for setting motor values. Takes values between -255 and 255 for each side, -255 is full speed backwards, 255 is full speed forwards.
	private:
		// Variables for storing motor driver pins
		short int _left;
		short int _right;
		short int _left_back;
		short int _right_back;
};

#endif