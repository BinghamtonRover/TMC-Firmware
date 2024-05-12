#include "BURT_TMC.h"

const int blockDelay = 10;  // ms

StepperMotor::StepperMotor(StepperMotorPins pins, StepperMotorConfig config) : 
  pins(pins),
  config(config),
  driver(TMC5160Stepper(SPI, pins.chipSelect, 0.075))
  { }

StepperMotor::StepperMotor(StepperMotorPins pins, StepperMotorConfig config, LimitSwitch limitSwitch) :
  pins(pins),
  config(config),
  limitSwitch(limitSwitch),
  driver(TMC5160Stepper(SPI, pins.chipSelect, 0.075))
  { }

bool StepperMotor::isMoving() {
  return driver.XTARGET() != driver.XACTUAL();
}

int StepperMotor::currentSteps() {
  return driver.XACTUAL() + limitSwitch.offset;
}

int StepperMotor::targetSteps() {
  return driver.XTARGET() + limitSwitch.offset;
}

double StepperMotor::currentPosition() {
  return currentSteps() * config.stepsPerUnit;
}

double StepperMotor::targetPosition() {
  return targetSteps() * config.stepsPerUnit;
}

void StepperMotor::presetup() {
  pinMode(pins.chipSelect, OUTPUT);
  digitalWrite(pins.chipSelect, HIGH);
}

void StepperMotor::reset_driver() {
  driver.begin();
  driver.reset();
  digitalWrite(pins.enable, HIGH);  // disable driver to clear the cache
	delay(1000);
	digitalWrite(pins.enable, LOW);   // re-enable drive, to start loading in parameters
}

void StepperMotor::check_driver() {
  TMC5160Stepper::IOIN_t ioin { driver.IOIN() };
  if (ioin.version == 0xFF || ioin.version == 0) {
    Serial.print("\nDriver communication error on motor: ");
    Serial.println(config.name);
    while (true);
  } else if (ioin.sd_mode) {
    Serial.println("Motor is configured for Step & Direction mode: ");
    Serial.println(config.name);
    while (true);
  } else if (ioin.drv_enn) {
    Serial.println("Motor is not hardware enabled: ");
    Serial.println(config.name);
    while (true);
  }
}

void StepperMotor::write_settings() {
  // TODO: Decide if everything below this is needed: 
	// See https://github.com/BinghamtonRover/arm-firmware/issues/6
  driver.GSTAT(7);
	driver.rms_current(config.current);
	driver.tbl(2);
	driver.toff(9);
	driver.pwm_freq(1);
	driver.a1(config.acceleration);
	driver.v1(config.speed);
	driver.AMAX(config.acceleration);
	driver.VMAX(config.speed);
	driver.DMAX(config.acceleration);
	driver.d1(config.acceleration);
	driver.vstop(100);
	driver.vstart(100);
	driver.RAMPMODE(0);
}

void StepperMotor::setup() {
  Serial.print("Initializing motor ");
  Serial.print(config.name);
  Serial.print("... ");
  pinMode(pins.enable, OUTPUT);
  digitalWrite(pins.enable, LOW);
  if (limitSwitch.pin != -1) pinMode(limitSwitch.pin, INPUT_PULLUP);
  reset_driver();
  check_driver();
  write_settings();
  Serial.println("Done!");
}

void StepperMotor::calibrate() {
  if (limitSwitch.pin == -1) return;
  while (!limitSwitch.isPressed()) {
    moveBySteps(10 * limitSwitch.direction);
  }
  stop();
  int limitSteps = limitSwitch.position * config.stepsPerUnit;
  limitSwitch.offset = limitSteps - driver.XACTUAL() * limitSwitch.direction;
}

void StepperMotor::update() {
  int target = driver.XTARGET();
  int current = driver.XACTUAL();
  bool isMovingTowardsLimit = limitSwitch.direction > 0
    ? target > current : target < current;
  if (limitSwitch.isPressed() && limitSwitch.isBlocking && isMovingTowardsLimit) stop();
}

void StepperMotor::stop() {
  driver.XTARGET(driver.XACTUAL());
}

void StepperMotor::block() {
  while (isMoving()) delay(blockDelay);
}

void StepperMotor::moveTo(double position) {
  if (!limitSwitch.isValid(position)) return;
  int steps = position * config.stepsPerUnit;
  moveToSteps(steps);
}

void StepperMotor::moveBy(double offset) {
  int steps = offset * config.stepsPerUnit;
  moveBySteps(steps);
}

void StepperMotor::moveToSteps(int steps) {
  driver.XTARGET(steps);
}

void StepperMotor::moveBySteps(int steps) {
  int target = driver.XACTUAL() + steps;
  driver.XTARGET(target);
}
