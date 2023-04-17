/*! @file 
 * \addtogroup Motor 
 * \{
*/

#include "BURT_TMC.h"

StepperMotor::StepperMotor(StepperMotorPins pins, StepperMotorConfig config) : 
	pins(pins),
	config(config),
	driver(TMC5160Stepper(SPI, pins.chipSelect, 0.075)) { }

int StepperMotor::radToSteps(float radians) { 
	return newAngle * config.stepsPerRotation / (2*PI);
}

void StepperMotor::presetup() {
	if (!IS_CONNECTED) return;
	pinMode(pins.chipSelect, OUTPUT);
	digitalWrite(pins.chipSelect, HIGH);
}

void StepperMotor::setup() {
	if (!IS_CONNECTED) return;
	// TODO: Decide if this is needed: 
	// See https://github.com/BinghamtonRover/arm-firmware/issues/6
	// <--
	pinMode(pins.enable, OUTPUT);
	pinMode(pins.limitSwitch,INPUT_PULLUP);
	digitalWrite(pins.enable, LOW);

	Serial.print("Initializing pins: ");
	Serial.print(pins.enable);
	Serial.print(" and ");
	Serial.print(pins.chipSelect);
	Serial.print(" for ");
	Serial.println(config.name)
	// -->
	driver.begin();
	driver.reset();
	TMC5160Stepper::IOIN_t ioin{ driver.IOIN() };
	if (ioin.version == 0xFF || ioin.version == 0) {
    Serial.print("Driver communication error on motor: ");
    Serial.println(config.name);
    while(true);
	} else if (ioin.sd_mode) {
    Serial.println("Motor is configured for Step & Dir mode: ");
    Serial.println(config.name);
    while(true);
	} else if (ioin.drv_enn) {
    Serial.println("Motor is not hardware enabled: ");
    Serial.println(config.name);
    while(true);
	}

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
}

void StepperMotor::update() {
	if (!IS_CONNECTED) return;
	bool isMovingDown = driver.XTARGET() < driver.XACTUAL();
	if (isLimitSwitchPresed() && isMovingDown) stop();
}

void StepperMotor::stop() {
	if (!IS_CONNECTED) return;
	driver.XTARGET(driver.XACTUAL());
	targetStep = driver.XACTUAL();
}

void StepperMotor::calibrate() { 
	if (pins.limitSwitch == NULL || !IS_CONNECTED) {
		Serial.print("Not calibrating motor without limit switch: " + config.name);
		return;
	} else {
		Serial.print("Calibrating motor: ");
		Serial.println(config.name);
	}

	while(!isLimitSwitchPressed()) {
		// This will technically overshoot the limit switch, but we call [stop]
		currentSteps -= 10;
		driver.XTARGET(currentSteps);
	}
	stop();
	angle = config.minLimit;
	stepsAtMinLimit = driver.XACTUAL();
	Serial.println("  Calibration finished");
}

void StepperMotor::moveTo(float radians) {
	Serial.print("Moving " + name + " to ");
	Serial.println(radians);
	if (radians > config.maxLimit || radians < config.minLimit) { 
		Serial.print("  ERROR: Out of bounds (valid inputs are ");
		Serial.print(config.minLimit);
		Serial.print(" to ");
		Serial.print(config.maxLimit);
		Serial.println(").");
		return; 
	}

	angle = radians;
	targetStep = radToSteps(angle);
	Serial.print("  That is  ");
	Serial.print(targetStep);
	Serial.print(" steps.");

	if (IS_CONNECTED) driver.XTARGET(targetStep);
	else Serial.println("  Motor not connected, not moving");
}

void StepperMotor::moveBy(float radians) {
	moveTo(angle + radians);
}

void StepperMotor::debugMoveToStep(int steps) {
	Serial.print("Moving " + config.name + " to ");
	Serial.print(steps);

	targetStep = steps;
	if (IS_CONNECTED) driver.XTARGET(targetStep);
	else Serial.println("  Motor not connected, not moving");
}

void StepperMotor::debugMoveBySteps(int steps) {
	moveToStep(targetStep + steps);
}

bool StepperMotor::isMoving() { 
	if (!IS_CONNECTED) return false;
	return driver.XTARGET() != driver.XACTUAL(); 
}

bool StepperMotor::isLimitSwitchPressed() {
	if (pins.limitSwitch == NULL || !IS_CONNECTED) return false;
	else return digitalRead(pins.limitSwitch) == LOW;
}

// The following close bracket marks the file for Doxygen
/*! \} */
