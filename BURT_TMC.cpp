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
	return radians / (2*PI) * config.stepsPerRotation + offset;
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
	bool isMovingDown = driver.XTARGET() < driver.XACTUAL();
	if (isLimitSwitchPressed() && isMovingDown) stop();
}

void StepperMotor::stop() {
	if (!IS_CONNECTED) return;
	driver.XTARGET(driver.XACTUAL());
	targetStep = driver.XACTUAL();
}

void StepperMotor::calibrate() { 
	if (pins.limitSwitch == 0 || !IS_CONNECTED) {
		Serial.println("Not calibrating motor without limit switch: " + config.name);
		return;
	} else {
		Serial.print("Calibrating motor: " + config.name + "... ");
	}

	while(!isLimitSwitchPressed()) {
		targetStep -= 10;
		driver.XTARGET(targetStep);
	}
	stop();  // the while loop overshoots the limit switch and will keep going
	angle = config.minLimit;
	offset = driver.XACTUAL() - radToSteps(angle);
	Serial.println("Done!");
}

void StepperMotor::moveTo(float radians) {
	Serial.print("Moving " + config.name + " to ");
	Serial.println(radians);
	// if (radians > config.maxLimit || radians < config.minLimit) { 
	// 	Serial.print("  ERROR: Out of bounds (valid inputs are ");
	// 	Serial.print(config.minLimit);
	// 	Serial.print(" to ");
	// 	Serial.print(config.maxLimit);
	// 	Serial.println(").");
	// 	return; 
	// }

	angle = radians;
	targetStep = radToSteps(angle);
	Serial.print("  That is  ");
	Serial.print(targetStep);
	Serial.println(" steps.");

	if (IS_CONNECTED) driver.XTARGET(targetStep);
	else Serial.println("  Motor not connected, not moving");
}

void StepperMotor::moveBy(float radians) {
	moveTo(angle + radians);
}

void StepperMotor::debugMoveToStep(int steps) {
	Serial.print("Moving " + config.name + " to ");
	Serial.println(steps);

	targetStep = steps;
	angle = radToSteps(steps);
	if (IS_CONNECTED) driver.XTARGET(targetStep);
	else Serial.println("  Motor not connected, not moving");
}

void StepperMotor::debugMoveBySteps(int steps) {
	debugMoveToStep(targetStep + steps);
}

bool StepperMotor::isMoving() { 
	if (!IS_CONNECTED) return false;
	return driver.XTARGET() != driver.XACTUAL(); 
}

bool StepperMotor::isLimitSwitchPressed() {
	if (pins.limitSwitch == 0 || !IS_CONNECTED) return false;
	else return digitalRead(pins.limitSwitch) == LOW;
}

// The following close bracket marks the file for Doxygen
/*! \} */
