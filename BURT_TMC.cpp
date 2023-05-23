/*! @file 
 * \addtogroup Motor 
 * \{
*/

#include "BURT_TMC.h"

// ================== StepperMotor ==================

StepperMotor::StepperMotor(StepperMotorPins pins, StepperMotorConfig config, LimitSwitch limitSwitch) : 
	pins(pins),
	config(config),
	limitSwitch(limitSwitch),
	driver(TMC5160Stepper(SPI, pins.chipSelect, 0.075)) { }

void StepperMotor::presetup() {
	if (!IS_CONNECTED) return;
	pinMode(pins.chipSelect, OUTPUT);
	digitalWrite(pins.chipSelect, HIGH);
}

void StepperMotor::setup() {
	if (!IS_CONNECTED) return;
	Serial.print("Initializing motor " + config.name + "... ");

	pinMode(pins.enable, OUTPUT);
	digitalWrite(pins.enable, LOW);
	if (limitSwitch.pin != -1) pinMode(limitSwitch.pin, INPUT_PULLUP);

	driver.begin();
	driver.reset();
	TMC5160Stepper::IOIN_t ioin{ driver.IOIN() };
	if (ioin.version == 0xFF || ioin.version == 0) {
    Serial.println("\nDriver communication error on motor: " + config.name);
    while(true);
	} else if (ioin.sd_mode) {
    Serial.println("\nMotor is configured for Step & Dir mode: " + config.name);
    while(true);
	} else if (ioin.drv_enn) {
    Serial.println("Motor is not hardware enabled: " + config.name);
    while(true);
	}

	// TODO: Decide if everything below this is needed: 
	// See https://github.com/BinghamtonRover/arm-firmware/issues/6
	digitalWrite(pins.enable, HIGH);  // disable driver to clear the cache
	delay(1000);
	digitalWrite(pins.enable, LOW);   // re-enable drive, to start loading in parameters

	driver.GSTAT(7);
	driver.rms_current(config.current);
	driver.tbl(2);
	driver.toff(9);
	driver.pwm_freq(1);
	driver.a1(config.accel);
	driver.v1(config.speed);
	driver.AMAX(config.accel);
	driver.VMAX(config.speed);
	driver.DMAX(config.accel);
	driver.d1(config.accel);
	driver.vstop(100);
	driver.vstart(100);
	driver.RAMPMODE(0);

	Serial.println("Done!");
}

void StepperMotor::update() {
	if (!IS_CONNECTED) return;
	bool isMovingTowardsSwitch = limitSwitch.direction > 0
		? driver.XTARGET() > driver.XACTUAL()
		: driver.XTARGET() < driver.XACTUAL();
	bool isPastLimitSwitch = limitSwitch.direction > 0
		? (driver.XACTUAL() * config.toUnits) > limitSwitch.position
		: (driver.XACTUAL() * config.toUnits) < limitSwitch.position;
	if (isLimitSwitchPressed() && isMovingTowardsSwitch && isPastLimitSwitch) stop();
}

void StepperMotor::stop() {
	if (!IS_CONNECTED) return;
	driver.XTARGET(driver.XACTUAL());
	Serial.print("Motor " + config.name + " needs to stop at ");
	Serial.print(getPosition());
	Serial.println(" " + config.unitName);
}

void StepperMotor::calibrate() { 
	if (limitSwitch.pin == -1 || !IS_CONNECTED) {
		Serial.println("Not calibrating motor without limit switch: " + config.name);
		return;
	} else {
		Serial.print("Calibrating motor: " + config.name + "... ");
	}

	int steps = driver.XACTUAL();
	while(!isLimitSwitchPressed()) {
		steps += 10 * limitSwitch.direction;
		driver.XTARGET(steps);
	}
	if (!isLimitSwitchPressed()) return calibrate();
	stop();  // the while loop overshoots the limit switch and will keep going
	limitSwitch.offset = driver.XACTUAL() * limitSwitch.direction;
	Serial.println("Done!");
}

void StepperMotor::moveTo(float position) {
	moveBy(position - getPosition());
}

void StepperMotor::moveBy(float distance) {
	// When called by [moveTo], this will be the same as the [position] parameter.
	float targetPosition = getPosition() + distance;
	Serial.print("Motor " + config.name + " is at ");
	Serial.print(driver.XACTUAL());
	Serial.print(" steps and ");
	Serial.print(getPosition());
	Serial.println(" " + config.unitName);

	Serial.print("Moving " + config.name + " by ");
	Serial.print(distance);
	Serial.print(" " + config.unitName + ". That is ");
	Serial.print(targetPosition);
	Serial.println(" " + config.unitName);

	// Check bounds
	if (targetPosition > config.maxLimit || targetPosition < config.minLimit) { 
		Serial.print("  ERROR: ");
		Serial.print(targetPosition);
		Serial.print(" is out of bounds (valid inputs are ");
		Serial.print(config.minLimit);
		Serial.print(" to ");
		Serial.print(config.maxLimit);
		Serial.println(config.unitName + ").");
		return; 
	}

	targetPosition -= limitSwitch.offset * config.toUnits;
	int steps = (distance * config.toSteps);
	int targetStep = config.isPositive
		? driver.XACTUAL() + steps
		: driver.XACTUAL() - steps;
	if (IS_CONNECTED) {
		driver.XTARGET(targetStep);
		Serial.print("  Motor " + config.name + " is moving to ");
		Serial.print(getTarget());
		Serial.println(" " + config.unitName);
	} else {
		Serial.println("  Motor not connected, not moving");
	}
}

void StepperMotor::moveToSteps(int steps) {
	Serial.print("Moving " + config.name + " to ");
	Serial.print(steps + limitSwitch.offset);
	Serial.print(" steps... ");

	steps += limitSwitch.offset;
	moveBySteps(steps - driver.XACTUAL());
}

void StepperMotor::moveBySteps(int steps) {
	if (IS_CONNECTED) driver.XTARGET(driver.XACTUAL() + steps);
	else Serial.println("  Motor not connected, not moving");
	Serial.print("Motor [" + config.name + "] is moving to ");
	Serial.print(driver.XTARGET());
	Serial.println(" steps");
}

bool StepperMotor::isMoving() { 
	if (!IS_CONNECTED) return false;
	return driver.XTARGET() != driver.XACTUAL(); 
}

bool StepperMotor::isLimitSwitchPressed() {
	if (limitSwitch.pin == -1 || !IS_CONNECTED) return false;
	else return digitalRead(limitSwitch.pin) == limitSwitch.triggeredValue;
}

float StepperMotor::getPosition() {
	return (driver.XACTUAL() - limitSwitch.offset) * config.toUnits;
}

float StepperMotor::getTarget() {
	return (driver.XTARGET() - limitSwitch.offset) * config.toUnits;
}

// The following close bracket marks the file for Doxygen
/*! \} */
