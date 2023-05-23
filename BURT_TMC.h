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
};

/// Configuration for a #StepperMotor.
struct StepperMotorConfig {
	/// The name of this motor. Used for debugging.
	String name;

	String unitName = "radians";

	/// The Root Mean Square current to feed the motor, in mA.
	int current;

	/// The speed of the motor, units unknown.
	float speed;

	/// The magnitude of the acceleration of the motor, units unknown.
	float accel;

	/// The minimum angular position, in radians. Leave blank for non-limited motion.
	float minLimit = -INFINITY;

	/// The maximum angular position, in radians. Leave blank for non-limited motion.
	float maxLimit = INFINITY;

	bool isPositive;

	float gearboxRatio = 1.0;

	float conversionFactor = 200 * 256 * gearboxRatio;

	float toUnits = (2*PI) / (conversionFactor);

	float toSteps = conversionFactor / (2*PI);
};

/// Calibrates a stepper motor.
/// 
/// A limit switch (or sensor) serves to calibrate a stepper motor by allowing it to move to a known
/// #position after powering on.
struct LimitSwitch {
	/// The pin this switch or sensor is connected to.
	int pin = -1;

	/// The value that the #pin should read if the switch or sensor is activated.
	int triggeredValue;

	/// Whether to approach this switch or sensor by moving positive (+1) or negative (-1) steps.
	int direction;

	/// The position this switch or sensor is at, in terms of #StepConfig::stepsToPosition. For example,
	/// a rotating arm might have a limit switch at 90 degrees (or pi / 2 rad). This field is useful
	/// when placing a limit switch at #StepperMotorConig::minLimit or #StepperMotorConfig::maxLimit.
	float position;

	int offset = 0;
};

/// Controls a TMC 5160 stepper motor. 
/// 
/// - In `setup`, call #presetup for *every* motor you have, *then* call #setup on each
/// - After a motor has been #setup, call #calibrate on it. Does nothing with no limit switch
/// - In `loop`, call #update for each motor you have
class StepperMotor { 
	private: 
		/// The pins this motor is attached to.
		StepperMotorPins pins;

		/// The configuration for this stepper motor.
		StepperMotorConfig config;

		/// The limit sensor for this motor. Leave blank if none are attached.
		LimitSwitch limitSwitch;

		/// The TMC instance for this motor, backed by the `TMCStepper` library.
		TMC5160Stepper driver;

	public: 
		/// Manage a stepper motor with the given configuration.
		StepperMotor(StepperMotorPins pins, StepperMotorConfig config, LimitSwitch limitSwitch);

		/// Ensures this stepper motor will not interfere with other motors.
		/// 
		/// You must call this method on *every* motor before trying to interface with one of them.
		void presetup();

		/// Initializes the motor. Must be called *after* #presetup is called on *all* motors.
		/// 
		/// The motor needs to be fully powered and on for this function to work. If setup fails, this
		/// function will log to the serial monitor and block indefinitely.
		void setup();

		/// Calibrates the motor.
		/// 
		/// If there is a limit switch (`limitSwitch.pin != -1`):
		/// - Moves toward the limit switch (#LimitSwitch::direction) until #isLimitSwitchPressed is `true`
		/// - Set #stepsAtLimitSensor to the current position of the motor (`driver.XACTUAL`)
		/// 
		/// Since this is usually in `setup` (where `loop` is unavailable) this function handles checking
		/// the limit switch and stopping by itself in a while loop, which means it is blocking.
		void calibrate();

		/// Maintains the state of the motor and searches for problems, calling #stop if needed.
		void update();

		/// Stops this motor by setting its target to its current position.
		/// 
		/// Call this in #update if the motor is passing its limits as defined by #StepperMotorConfig.
		void stop();

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
		void moveToSteps(int destination);

		/// Moves by the given amount of steps. 
		/// 
		/// Use in debugging only. In production code, use #moveBy. This method can help determine
		/// when #StepperMotorConfig::stepsPer180 is off, or when the motor is misbehaving.
		void moveBySteps(int steps);
		
		/// Whether this motor is still moving.
		/// 
		/// Compares #TMC5160Stepper::XACTUAL to #TMC5160Stepper::XTARGET. 
		bool isMoving();

		/// Whether the limit switch is being pressed.
		bool isLimitSwitchPressed();

		float getPosition();
		float getTarget();
};

#endif
// The following close bracket marks the file for Doxygen
/*! \} */
