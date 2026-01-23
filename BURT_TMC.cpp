#include "BURT_TMC.h"

const int blockDelay = 10;  // ms

StepperMotor::StepperMotor(const StepperGeneralConfig& g,
                           const StepperMotorPins& p,
                           const StepDirConfig& cfg)
    : general(g),
      pins(p),
      driver(TMC5160Stepper(SPI, p.chip_select, 0.075)),
      mode(STEP_DIR_MODE)
{
  config.stepDir = cfg;
}

StepperMotor::StepperMotor(const StepperGeneralConfig& g,
                           const StepperMotorPins& p,
                           const InternalRampConfig& cfg)
    : general(g),
      pins(p),
      driver(TMC5160Stepper(SPI, p.chip_select, 0.075)),
      mode(INT_RAMP_MODE)
{
  config.ramp = cfg;
}

bool StepperMotor::isMoving() {
  return driver.XTARGET() != driver.XACTUAL();
}

int StepperMotor::currentSteps() {
  // return driver.XACTUAL() + limitSwitch.offset + limitSwitch.position * config.ramp.steps_per_unit;
  return -1;
}

int StepperMotor::targetSteps() {
  // return driver.XTARGET() + limitSwitch.offset + limitSwitch.position * config,ramp.steps_per_unit;
  return -1;
}

double StepperMotor::currentPosition() {
  return currentSteps() / general.steps_per_unit;
}

double StepperMotor::targetPosition() {
  return targetSteps() / general.steps_per_unit;
}

void StepperMotor::preSetup() {
  pinMode(pins.chip_select, OUTPUT);
  digitalWrite(pins.chip_select, HIGH);
  if (mode == STEP_DIR_MODE) {
    pinMode(pins.step_pin, OUTPUT);
    pinMode(pins.dir_pin, OUTPUT);
    digitalWrite(pins.step_pin, LOW);
    digitalWrite(pins.dir_pin, LOW);
  }
}

void StepperMotor::resetDriver() {
  if (status == E_STOPPED) return;
  switch (mode) {
      case STEP_DIR_MODE:
        driver.begin();
        driver.reset();
        start_time_ms = last_check_ms = 0;
        done_flag = false;
        status = RETRYING;
        do {
          checkDriver();
          delay(1);
        } while (status != STP_DIR_OK || !(done_flag));
        writeSettings();
        Serial.print("Driver SD Mode status: ");
        Serial.println(driver.sd_mode());
        break;
      case (INT_RAMP_MODE):
        driver.begin();
        driver.reset();
        start_time_ms = last_check_ms = 0;
        done_flag = false;
        status = RETRYING;
        do {
          checkDriver();
          delay(1);
        } while (status != POS_OK || !(done_flag));
        writeSettings();
        Serial.println("Driver is in Internal Ramp Mode");
        break;
      default: 
        Serial.println("Error: Motor not configured in S/D or Int Pos Mode");
        break;
      }
}

void StepperMotor::checkDriver() {
  if (done_flag || (status == E_STOPPED)) return;
  if (!start_time_ms) start_time_ms = millis();
  uint32_t now = millis();
  if (now - last_check_ms >= retry_delay_ms) {
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
      done_flag = true;
      status = STP_DIR_OK;
    } 
    else {
      // Internal Ramp Mode Good
      done_flag = true;
      status = POS_OK;
    }
    if ((now - start_time_ms) > timeout_ms && !done_flag) {
      if (status == RETRYING_COMM)      status = COMM_ERR;
      else if (status == RETRYING_ENN)  status = ENN_ERR;
      else                              status = TIMEOUT;
      done_flag = true;
    }
  }
}

void StepperMotor::writeSettings() {
  switch (mode) {
    case STEP_DIR_MODE:
      // General Setup
      driver.GSTAT(0b111); // Clear latched errors
      driver.en_pwm_mode(config.stepDir.stealth_chop_en); // Enable stealthChop if configured
      driver.multistep_filt(true); // Enable internal filtering on STEP pin
      driver.shaft(config.stepDir.invert_dir); // Invert DIR if configured
      driver.GLOBAL_SCALER(200); // Scales values pertaining to current by (200)

      // Current and Delays
      driver.irun(config.stepDir.run_current_scale); // Scale IRUN to config
      driver.ihold(config.stepDir.hold_current_scale); // Scale IHOLD to config
      driver.iholddelay(config.stepDir.ihold_delay_scale); // Scale IHOLDDELAY to config
      driver.TPOWERDOWN(5); // Delay from StandStill -> Powerdown

      // Threshold to switch from StealthChop to SpreadCycle
      // TPWMTHRS ~= f_clk/((joint_deg_per_s/360)*GEAR_RATIO*STEPS_PER_REV*(256 / MRES))
      driver.TPWMTHRS(config.stepDir.spread_cycle_start_thrs);  

      // Chopper Configuration (SpreadCycle + MicroPlyer)
      driver.intpol(1); // MRES extrapolated to 256usteps internally to smooth motion (STEP/DIR ONLY)
      driver.mres(0b0100); // %0001 … %1000: 128, 64, 32, 16, 8, 4, 2, FULLSTEP, 0b0100 = 16, allows lower STEP freq from MCU
      driver.tbl(2); // Set comparator blank time (0-3 => 16, 24, 36, 54) (Recommended 1 or 2)
      driver.dedge(config.stepDir.double_edge); // 1 uses falling edge as second step pulse, allows lower step freq from MCU but requires exactly 50% duty cycle
      driver.toff(3); // Off time setting controls duration of slow decay phase, NCLK= 24 + 32*TOFF
      break;
    case INT_RAMP_MODE:
      driver.GSTAT(7);
      driver.rms_current(config.ramp.current);
      driver.tbl(2);
      driver.toff(9);
      driver.pwm_freq(1);
      driver.a1(config.ramp.acceleration);
      driver.v1(config.ramp.speed);
      driver.AMAX(config.ramp.acceleration);
      driver.VMAX(config.ramp.speed);
      driver.DMAX(config.ramp.acceleration);
      driver.d1(config.ramp.acceleration);
      driver.vstop(100);
      driver.vstart(100);
      driver.RAMPMODE(0);
      break;
    default: 
      Serial.println("Error: Motor not configured in S/D or Int Pos Mode");
      break;
  }
}

void StepperMotor::setup() {
  Serial.print("Initializing motor ");
  Serial.print(general.name);
  Serial.println("... ");
  resetDriver();
  Serial.print("  => ");
  Serial.println(statusToString(status));
}

void StepperMotor::calibrate() {
  // if (!limitSwitch.isAttached()) return;
  // while (!limitSwitch.isPressed()) {
  //   moveBySteps(10 * limitSwitch.direction);
  // }
  // stop();
  // int limitSteps = limitSwitch.position * general.stepsPerUnit;
  // // limitSwitch.offset = limitSteps - driver.XACTUAL() * limitSwitch.direction;
  // limitSwitch.offset = -driver.XACTUAL();
}

void StepperMotor::update() {
  // int target = driver.XTARGET();
  // int current = driver.XACTUAL();
  // bool isMovingTowardsLimit = limitSwitch.direction > 0
  //   ? target > current : target < current;
  // if (limitSwitch.isPressed() && limitSwitch.isBlocking && isMovingTowardsLimit) stop();
}

void StepperMotor::stop() {
  driver.XTARGET(driver.XACTUAL());
}

void StepperMotor::block() {
  while (isMoving()) delay(blockDelay);
}

void StepperMotor::moveTo(double position) {
  // if (!limitSwitch.isValid(position)) return;
  // int steps = position * general.steps_per_unit;
  // moveToSteps(steps);
}

void StepperMotor::moveBy(double offset) {
  int steps = offset * general.steps_per_unit;
  moveBySteps(steps);
}

void StepperMotor::moveToSteps(int steps) {
  driver.XTARGET(steps);
}

void StepperMotor::moveBySteps(int steps) {
  int target = driver.XACTUAL() + steps;
  driver.XTARGET(target);
}

void StepperMotor::eStop() {
  // Stop STEP driving
  analogWrite(pins.step_pin, 0);
  digitalWrite(pins.step_pin, LOW);

  driver.toff(0); // Disable bridges
  done_flag = true;
  status = E_STOPPED;
}

void StepperMotor::clearEStop() {
  // sanity: keep STEP low until configured
  analogWrite(pins.step_pin, 0);
  digitalWrite(pins.step_pin, LOW);

  driver.toff(3); // Reenable bridges
  start_time_ms = last_check_ms = 0;
  done_flag = false;
  status = RETRYING;
  checkDriver();
}

void StepperMotor::setDir(uint8_t direction) {
  digitalWrite(pins.dir_pin, direction);
}

void StepperMotor::setStepHz(uint32_t f_step) {
  uint32_t f_PWM = config.stepDir.double_edge ? f_step/2 : f_step;

  // Clamp PWM to [20kHz, 200kHz]
  if (f_PWM < min_freq) f_PWM = min_freq;
  if (f_PWM > max_freq) f_PWM = max_freq;  

  analogWriteFrequency(pins.step_pin, f_PWM);
  analogWrite(pins.step_pin, 128); // 50% duty cycle
}

void StepperMotor::setMotorRps(float rps) {
  // Set DIR
  setDir((rps < 0) ? HIGH : LOW);
  rps = fabsf(rps);
  // FORMULA: f_step = n_joint*G*N_step*M_res
  uint32_t f_step = static_cast<uint32_t>(
    rps*config.stepDir.gear_ratio*steps_per_rotation*mres + 0.5f
  );
  setStepHz(f_step);
}