/*! @file 
 * \addtogroup Motor 
 * \{
*/

#ifndef burt_arm_motor_h
#define burt_arm_motor_h

#include <Arduino.h>

#include "TmcStepper.h"

/// If this is false, all firmware will print to the console instead of using hardware.
#define IS_CONNECTED true

/// The pins that a #StepperMotor.
struct StepperMotorPins {
	/// The SPI chip select pin for this stepper motor.
	int chipSelect;

	/// The enable pin for this stepper motor. 
	/// 
	/// TMC5160 chips are enable-low, which means this must be written LOW to work.
	int enable;

	/// The limit switch associated for this motor. 
	/// 
	/// Set this to -1 to disable limit switches.
	int limitSwitch;
};

/// Configuration for a #StepperMotor.
struct StepperMotorConfig {
	/// The name of this motor.
	String name;

	/// The Root Mean Square current to feed the motor, in mA.
	int current;

	/// The lower physical bound for this motor, in radians from the horizontal.
	/// 
	/// If there is a limit switch, it is located here. For continuous motion, use `-INFINITY`.
	/// 
	/// WARNING: Moving lower than this value can cause damage to the arm. 
	float limitSwitchPosition;

	/// The lower physical bound for this motor, in radians from the horizontal.
	/// 
	/// If there is a limit switch, it is located here. For continuous motion, use `-INFINITY`.
	/// 
	/// WARNING: Moving lower than this value can cause damage to the arm. 
	float minLimit;

	/// The upper physical bound for this motor, in radians from the horizontal.
	/// 
	/// If there is a limit switch, it is at #minLimit. For continuous motion, use `INFINITY`.
	/// 
	/// WARNING: Moving higher than this value can cause damage to the arm.
	float maxLimit;

	/// Does increasing the step count increase the radians?
	bool isPositive;
	
	/// The direction of the limit switch, either +1 or -1.
	int limitSwitchDirection;

	/// The gearbox ratio on this stepper motor.
	/// 
	/// Used to calculate #stepsPerRotation
	int gearboxRatio;

	/// The number of steps the *motor* will need to turn a full rotation, without microstepping.
	/// 
	/// Used to calculate #stepsPerRotation. Most stepper motors use 200.
	int motorStepsPerRotation = 200;

	/// The speed of this motor.	
	float speed;

	/// The acceleration of this motor.
	float accel;

	/// The amount of steps per 360 degrees, or 2π radians.
	/// 
	/// This is #motorStepsPerRotation times #gearboxRatio times 256 microsteps per step.
	int stepsPerRotation();
};

/// Controls a TMC 5160 stepper motor. 
/// 
/// This class drives a stepper motor that may have a few restrictions: 
/// - If $StepperMotorPins::limitSwitch is non-null, there is a limit switch at #StepperMotorConfig::minLimit
/// - This motor will never move lower than the limit switch, or higher than #StepperMotorConfig::maxLimit
/// 
/// For a continuous motor (with no limit switch), use `INFINITY` and `-INFINITY` for the limits. 
/// For a motor with a limit switch, this setup assumes the joint is free to move in the *positive*
/// direction away from the switch -- in other words, negative steps is towards the switch.
/// 
/// When using multiple instances of this class, be sure to call #presetup on *all* of them first, 
/// and only then call #setup on each of them. Then you can use #moveTo or #moveBy as desired. 
class StepperMotor { 
	private: 
		/// The pins this motor is attached to.
		StepperMotorPins pins;

		/// The configuration for this stepper motor.
		StepperMotorConfig config;

		/// The TMC instance for this motor, backed by the `TMCStepper` library.
		TMC5160Stepper driver;

		/// Converts the desired radians to a number of steps, using #StepperMotorConfig::stepsPer180.
		int radToSteps(float radians);

		/// Converts the given step count to radians.
		float stepsToRad(int steps);

		/// The step this motor is at or trying to get to.
		/// 
		/// When moving, this is identical to #TMC5160Stepper::XTARGET. When stationary, this is the
		/// same as #TMC5160Stepper::XACTUAL. Use this when you do not care whether the arm is moving.
		int targetStep;

		/// The step that #TMC5160Stepper::XACTUAL would return at #StepperMotorConfig::minLimit.
		/// 
		/// Since the TMC5160 sets `XACTUAL` to zero on boot, this value changes every startup. If there
		/// is a limit switch, use #calibrate` to move there and record the new `XACTUAL`.
		int offset = 0;

	public: 
		/// The current angle this joint is at, in radians.
		float angle;

		/// Manage a stepper motor with the given pins.
		StepperMotor(StepperMotorPins pins, StepperMotorConfig config);

		/// Ensures this stepper motor will not interfere with other motors.
		/// 
		/// You must call this method on *every* motor before trying to interface with one of them.
		void presetup();

		/// Initializes the motor. Must be called *after* #presetup is called on *all* motors.
		/// 
		/// The motor needs to be fully powered and on for this function to work. If setup fails, this
		/// function will log to the serial monitor and block indefinitely.
		void setup();  

		/// Maintains the state of the motor and searches for problems.
		/// 
		/// Currently, this function checks if the limit switch is being pressed and stops the motor.
		void update();

		/// Stops this motor by setting its target to its current position.
		void stop();

		/// Calibrates the motor.
		/// 
		/// If there is a limit switch (`pins.limitSwitch != NULL`):
		/// - Moves toward the limit switch (negative steps) until #isLimitSwitchPressed returns `true`
		/// - Set `stepsAtMinLimit` to the current position of the motor.
		/// 
		/// Since this is usually called in #setup or by itself, this function handles checking the 
		/// limit switch and stopping by itself in a while loop, which means it is blocking.
		void calibrate();

		/// Moves the motor to a specific angle, in radians. 
		/// 
		/// Differs from #moveBy in that this function receives a target rotation and rotates 
		/// until it matches (see #isFinished). Use this method in IK mode, where you know where to go. 
		void moveTo(float radians);

		/// Moves the motor by a given rotation, in radians. 
		/// 
		/// Differs from #moveTo in that this function receives an offset in radians and rotates 
		/// by that amount. Use this method in precision mode, where you move in small increments. 
		void moveBy(float radians);

		/// Moves to the given step position. 
		/// 
		/// Use in debugging only. In production code, use #moveTo. This method can help determine
		/// when #StepperMotorConfig::stepsPer180 is off, or when the motor is misbehaving.
		void debugMoveToStep(int destination);

		/// Moves by the given amount of steps. 
		/// 
		/// Use in debugging only. In production code, use #moveBy. This method can help determine
		/// when #StepperMotorConfig::stepsPer180 is off, or when the motor is misbehaving.
		void debugMoveBySteps(int steps);
		
		/// Whether this motor is still moving.
		/// 
		/// Works by comparing #TMC5160Stepper::XACTUAL to #TMC5160Stepper::XTARGET. 
		bool isMoving();

		/// Whether the limit switch is being pressed.
		bool isLimitSwitchPressed();
};

#endif
// The following close bracket marks the file for Doxygen
/*! \} */
