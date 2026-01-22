#include "BURT_TMC.h"

const int blockDelay = 10;  // ms

StepperMotor::StepperMotor(StepperMotorPins pins, StepDirConfig config) {
  StepperMotorPins StepperMotor::pins;
  StepDirConfig StepperMotor::config;
  StepperMotor::step_dir_mode = true;
  driver(TMC5160Stepper(SPI, pins.chipSelect, 0.075))
}

StepperMotor::StepperMotor(StepperMotorPins pins, InternalRampConfig config) {
  StepperMotorPins StepperMotor::pins;
  InternalRampConfig StepperMotor::config;
  StepperMotor::step_dir_mode = false;
  driver(TMC5160Stepper(SPI, pins.chipSelect, 0.075))
  }

bool StepperMotor::isMoving() {
  return driver.XTARGET() != driver.XACTUAL();
}

int StepperMotor::currentSteps() {
  return driver.XACTUAL() + limitSwitch.offset + limitSwitch.position * config.stepsPerUnit;
}

int StepperMotor::targetSteps() {
  return driver.XTARGET() + limitSwitch.offset + limitSwitch.position * config.stepsPerUnit;
}

double StepperMotor::currentPosition() {
  return currentSteps() / config.stepsPerUnit;
}

double StepperMotor::targetPosition() {
  return targetSteps() / config.stepsPerUnit;
}

void StepperMotor::presetup() {
  pinMode(pins.chipSelect, OUTPUT);
  digitalWrite(pins.chipSelect, HIGH);
}

void StepperMotor::reset_driver() {
  if (status == E_STOPPED) return;
  if (step_dir_mode) {
    driver.begin();
    driver.reset();
    start_time_ms = last_check_ms = 0;
    done_f = false;
    status = RETRYING;
    do
    {
      check_driver();
      delay(1);
    } while (status != STP_DIR_OK);
    write_settings();
    Serial.print("Driver SD Mode status: ");
    Serial.println(driver.sd_mode());
  } else if (!step_dir_mode) {
    driver.begin();
    driver.reset();
    start_time_ms = last_check_ms = 0;
    done_f = false;
    status = RETRYING;
    do
    {
      check_driver();
      delay(1);
    } while (status != POS_OK);
    write_settings();
    Serial.print("Driver is in Internal Ramp Mode");
  }
}

void StepperMotor::check_driver() {
  if (done_f || (status == E_STOPPED)) return;
  if (!start_time_ms) start_time_ms = millis();
  uint32_t now = millis();
  if (now - last_check_ms >= RETRY_DELAY_MS) {
    last_check_ms = now;
    TMC5160Stepper::IOIN_t ioin { driver.IOIN() };
    if (ioin.version == 0xFF || ioin.version == 0) {
      // Comm Error
      status = RETRYING_COMM;
    } 
    else if (ioin.drv_enn) {
      // Driver Enable Error (Hardware) [EN pin is not tied to GND]
      status = RETRYING_ENN;
    }
    else if (ioin.sd_mode) {
      // Step/Dir Mode Good
      done_f = true;
      status = STP_DIR_OK;
    } 
    else {
      // Internal Ramp Mode Good
      done_f = true;
      status = POS_OK;
    }
    if (((now - start_time_ms) > TIMEOUT_MS) && !(done_f)){
      status |= 0x80; // Change Error Code to Timeout version
      done_f = true;
    }
  }
}

void StepperMotor::write_settings() {
  if (step_dir_mode) {
    // General Setup
    driver.GSTAT(0b111); // Clear latched errors
    driver.en_pwm_mode(config.stealthChop_en); // Enable stealthChop if configured
    driver.multistep_filt(true); // Enable internal filtering on STEP pin
    driver.shaft(config.invert_dir); // Invert DIR if configured
    driver.GLOBAL_SCALER(200); // Scales values pertaining to current by (200)

    // Current and Delays
    driver.irun(config.run_current_scale); // Scale IRUN to config
    driver.ihold(config.hold_current_scale); // Scale IHOLD to config
    driver.iholddelay(config.ihold_delay_scale); // Scale IHOLDDELAY to config
    driver.TPOWERDOWN(5); // Delay from StandStill -> Powerdown

    // Threshold to switch from StealthChop to SpreadCycle
    // TPWMTHRS ~= f_clk/((joint_deg_per_s/360)*GEAR_RATIO*STEPS_PER_REV*(256 / MRES))
    driver.TPWMTHRS(config.spreadCycle_start_thrs);  

    // Chopper Configuration (SpreadCycle + MicroPlyer)
    driver.intpol(1); // MRES extrapolated to 256usteps internally to smooth motion (STEP/DIR ONLY)
    driver.mres(0b0100); // %0001 … %1000: 128, 64, 32, 16, 8, 4, 2, FULLSTEP, 0b0100 = 16, allows lower STEP freq from MCU
    driver.tbl(2); // Set comparator blank time (0-3 => 16, 24, 36, 54) (Recommended 1 or 2)
    driver.dedge(config.dedge); // 1 uses falling edge as second step pulse, allows lower step freq from MCU but requires exactly 50% duty cycle
    driver.toff(3); // Off time setting controls duration of slow decay phase, NCLK= 24 + 32*TOFF
  }
  else if (!step_dir_mode) {
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
  // if (!limitSwitch.isAttached()) return;
  // while (!limitSwitch.isPressed()) {
  //   moveBySteps(10 * limitSwitch.direction);
  // }
  stop();
  int limitSteps = limitSwitch.position * config.stepsPerUnit;
  // limitSwitch.offset = limitSteps - driver.XACTUAL() * limitSwitch.direction;
  limitSwitch.offset = -driver.XACTUAL();
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
