/*! @file 
 * \addtogroup Motor 
 * \{
*/

#include "BURT_TMC.h"

int StepperMotorConfig::stepsPerRotation() {
	return motorStepsPerRotation * gearboxRatio * 256;
}

StepperMotor::StepperMotor(StepperMotorPins pins, StepperMotorConfig config) : 
	pins(pins),
	config(config),
	driver(TMC5160Stepper(SPI, pins.chipSelect, 0.075)) { }

int StepperMotor::radToSteps(float radians) { 
	return radians / (2*PI) * config.stepsPerRotation();
}

float StepperMotor::stepsToRad(int steps) {
	return (2 * PI * (float) steps) / config.stepsPerRotation();
}

void StepperMotor::presetup() {
	if (!IS_CONNECTED) return;
	pinMode(pins.chipSelect, OUTPUT);
	digitalWrite(pins.chipSelect, HIGH);
}

void StepperMotor::setup() {
	if (!IS_CONNECTED) return;
	Serial.print("Initializing motor " + config.name + "... ");

	pinMode(pins.enable, OUTPUT);
	pinMode(pins.limitSwitch, INPUT_PULLUP);
	digitalWrite(pins.enable, LOW);

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

	// TODO: Decide if this is needed: 
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
	bool isMovingTowardsSwitch = config.isPositive
		? driver.XTARGET() > driver.XACTUAL()
		: driver.XTARGET() < driver.XACTUAL();
	if (isLimitSwitchPressed() && isMovingTowardsSwitch) stop();
}

void StepperMotor::stop() {
	if (!IS_CONNECTED) return;
	driver.XTARGET(driver.XACTUAL());
	targetStep = driver.XACTUAL();
	angle = stepsToRad(targetStep);
	Serial.print("  Motor " + config.name + " is now at ");
	Serial.print(angle);
	Serial.println(" radians");
}

void StepperMotor::calibrate() { 
	if (pins.limitSwitch == -1 || !IS_CONNECTED) {
		Serial.println("Not calibrating motor without limit switch: " + config.name);
		return;
	} else {
		Serial.print("Calibrating motor: " + config.name + "... ");
	}

	while(!isLimitSwitchPressed()) {
		targetStep -= 10;
		driver.XTARGET(targetStep);
	}
	if (!isLimitSwitchPressed()) return calibrate();
	stop();  // the while loop overshoots the limit switch and will keep going
	angle = config.limitSwitchPosition;
	offset = driver.XACTUAL() - radToSteps(angle);
	targetStep = driver.XACTUAL();
	Serial.println("Done!");
}

void StepperMotor::moveTo(float radians) {
	Serial.print("Moving " + config.name + " to ");
	Serial.println(radians);

	float distance = radians - angle;
	moveBy(distance);
}

void StepperMotor::moveBy(float radians) {
	Serial.print("Moving " + config.name + " by ");
	Serial.print(radians);
	Serial.println(" radians");
	float targetAngle = angle + radians;
	if (targetAngle > config.maxLimit || targetAngle < config.minLimit) { 
		Serial.print("  ERROR: Out of bounds (valid inputs are ");
		Serial.print(config.minLimit);
		Serial.print(" to ");
		Serial.print(config.maxLimit);
		Serial.println(").");
		return; 
	}

	int distance = radToSteps(radians);
	if (config.isPositive) targetStep += distance;
	else targetStep -= distance;
	angle = targetAngle;

	if (IS_CONNECTED) {
		driver.XTARGET(targetStep);
		Serial.print("  Motor " + config.name + " is now at ");
		Serial.print(angle);
		Serial.println(" radians");
		Serial.println("Done");
	} else {
		Serial.println("  Motor not connected, not moving");
	}
}

void StepperMotor::debugMoveToStep(int steps) {
	Serial.print("Moving " + config.name + " to ");
	Serial.println(steps);

	int distance = steps - targetStep;
	debugMoveBySteps(distance);
}

void StepperMotor::debugMoveBySteps(int steps) {
	targetStep += steps;
	angle = stepsToRad(targetStep);

	if (IS_CONNECTED) driver.XTARGET(targetStep);
	else Serial.println("  Motor not connected, not moving");
}

bool StepperMotor::isMoving() { 
	if (!IS_CONNECTED) return false;
	return driver.XTARGET() != driver.XACTUAL(); 
}

bool StepperMotor::isLimitSwitchPressed() {
	if (pins.limitSwitch == -1 || !IS_CONNECTED) return false;
	else return digitalRead(pins.limitSwitch) == HIGH;
}

// The following close bracket marks the file for Doxygen
/*! \} */
