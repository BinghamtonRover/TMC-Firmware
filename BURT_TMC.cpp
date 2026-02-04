#include "BURT_TMC.h"
#include <cmath>

/**
 * @file BURT_TMC.cpp
 * @brief Implementation of StepperMotor using TMC5160. Supports STEP/DIR mode
 *        and internal ramp (position) mode with non-blocking init and
 *        configurable debug traces (enable with BURT_DEBUG).
 */

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

/** @brief True if XTARGET != XACTUAL */
bool StepperMotor::isMoving() {
  return driver.XTARGET() != driver.XACTUAL();
}

/** @brief Current step counter (signed 32-bit) */
int32_t StepperMotor::currentSteps() {
  return static_cast<int32_t>(driver.XACTUAL()); /*+ limitSwitch.offset + limitSwitch.position * config.ramp.steps_per_unit; */
}

/** @brief Target step counter (signed 32-bit) */
int32_t StepperMotor::targetSteps() {
  return static_cast<int32_t>(driver.XTARGET()); /*+ limitSwitch.offset + limitSwitch.position * config.ramp.steps_per_unit; */
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

void StepperMotor::prepReset() {
  if (status == E_STOPPED) return;

  driver.begin();
  driver.reset();
  // delay(5); // MAYBE NEEDED, leave commented for nonblocking (preferred)

  #if defined(BURT_DEBUG)
  uint32_t raw = driver.IOIN();
  TMC5160Stepper::IOIN_t i { raw };
  Serial.print("IOIN raw=0x"); Serial.println(raw, HEX);
  Serial.print("VERSION=0x"); Serial.println(i.version, HEX);
  Serial.print("SD_MODE="); Serial.println(i.sd_mode);
  Serial.print("DRV_ENN="); Serial.println(i.drv_enn);
  #endif

  // prep state
  start_time_ms = 0;
  last_check_ms = millis() - retry_delay_ms;
  status = RETRYING;
  init_in_progress = true;
}

void StepperMotor::tryReset(const unsigned timeout) {
  if (!init_in_progress || status == E_STOPPED) return;

  checkDriver(timeout);

  if (isDone(status)) {
    init_in_progress = false;

    if (isSuccess(status)) {
      ++init_success_cntr;
      writeSettings();
      #if defined(BURT_DEBUG)
      if (mode == INT_RAMP_MODE) Serial.println("Driver is in Internal Ramp Mode");
      else                       Serial.println("Driver is in STEP/DIR Mode");
      #endif
    } else {
      #if defined(BURT_DEBUG)
      Serial.print("Init failed: ");
      Serial.println(statusToString(status));
      #endif
    }
  }
} 

void StepperMotor::checkDriver(const unsigned timeout) {
  if (isDone(status)) return;
  if (!start_time_ms) start_time_ms = millis();
  uint32_t now = millis();
  DriverStatus old = status;

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
    } else {
      // COMM good, ENN good
      if (mode == STEP_DIR_MODE) {
        if (ioin.sd_mode) {
          status = STP_DIR_OK;
        } else {
          // wrong sd_mode for STEP/DIR expected
          status = RETRYING_MODE;
        }
      } else { // INT_RAMP_MODE
        if (!ioin.sd_mode) {
          status = POS_OK;
        } else {
          // wrong sd_mode for INT_RAMP expected
          status = RETRYING_MODE;
        }
      }
    }
  }

  if ((now - start_time_ms) > timeout && !isDone(status)) {
    if (status == RETRYING_COMM)      status = COMM_ERR;
    else if (status == RETRYING_ENN)  status = ENN_ERR;
    else if (status == RETRYING_MODE) status = MODE_ERR;
    else                              status = TIMEOUT;
  }

  // Debug: print status transitions to help debugging initialization
  if (status != old) {
    prev_status = old;
    #if defined(BURT_DEBUG)
    Serial.print(general.name);
    Serial.print(" status -> ");
    Serial.println(statusToString(status));
    #endif
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
      // Clear any latched errors
      driver.GSTAT(0b111);

      // Current regulation: set RMS current target (driver units; depends on RS/GLOBAL_SCALER)
      driver.rms_current(config.ramp.current);

      // Chopper / PWM tuning
      // - tbl: comparator blank time (0..3), affects chopper timing and EMI
      // - toff: off-time (affects slow decay behavior)
      // - pwm_freq: PWM frequency selector (affects audible noise vs resolution)
      driver.tbl(2);
      driver.toff(9);
      driver.pwm_freq(1);

      // Internal ramp profile parameters
      // - A1 / V1: initial ramp (first segment) acceleration/velocity to smoothly leave standstill
      // - AMAX / VMAX: maximum acceleration/velocity used by the internal ramp
      // - DMAX / D1: maximum deceleration and initial decel segment
      // Units are in the TMC register units (driver steps/second and steps/second^2)
      driver.a1(config.ramp.acceleration);
      driver.v1(config.ramp.speed);
      driver.AMAX(config.ramp.acceleration);
      driver.VMAX(config.ramp.speed);
      driver.DMAX(config.ramp.acceleration);
      driver.d1(config.ramp.acceleration);

      // vstop/vstart - thresholds used to determine 'stopped' and safe re-start velocities
      // These should be tuned for mechanical load (higher for heavy loads)
      driver.vstop(100);
      driver.vstart(100);

      // RAMPMODE=0 selects internal position mode where XTARGET drives the position ramp
      driver.RAMPMODE(0);
      break;
    default: 
      #if defined(BURT_DEBUG)
      Serial.println("Error: Motor not configured in S/D or Int Pos Mode");
      #endif
      break;
  }
} 

void StepperMotor::setup() {
  #if defined(BURT_DEBUG)
  Serial.print("Initializing motor ");
  Serial.print(general.name);
  Serial.println("... (non-blocking init started)");
  Serial.println("Call update() repeatedly or waitForInit() to complete initialization.");
  #endif

  // Start non-blocking initialization; `update()` will continue it.
  prepReset();
}

/**
 * @brief Blocking helper that waits for initialization to complete.
 * @param timeout_ms maximum time to wait (0 = wait forever)
 * @return true on success, false on timeout or fatal error
 */
bool StepperMotor::waitForInit(uint32_t timeout_ms) {
  uint32_t start = millis();
  while (init_in_progress) {
    tryReset(loop_timeout_ms);
    delay(retry_delay_ms);
    if (timeout_ms && (millis() - start) > timeout_ms) {
      #if defined(BURT_DEBUG)
      Serial.print(general.name);
      Serial.println(" waitForInit: timeout");
      #endif
      return false;
    }
  }
  return isSuccess(status);
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
  /* 
  int target = driver.XTARGET();
  int current = driver.XACTUAL();
  bool isMovingTowardsLimit = limitSwitch.direction > 0
    ? target > current : target < current;
  if (limitSwitch.isPressed() && limitSwitch.isBlocking && isMovingTowardsLimit) stop();
  */
  if (status == E_STOPPED) return;

  // Drive initialization forward if it is in progress
  if (init_in_progress) {
    tryReset(loop_timeout_ms);
    return;
  }

  // If we are in an error state, periodically attempt to kick the init again
  if (status & 0x80) {
    uint32_t now = millis();
    if (now - last_init_kick_ms >= INIT_KICK_PERIOD_MS) {
      last_init_kick_ms = now;
      prepReset();
    }
  }
}

void StepperMotor::stop() {
  driver.XTARGET(driver.XACTUAL());
}

void StepperMotor::block() {
  while (isMoving()) delay(blockDelay);
}

/** @brief Move to a position given in user units (uses steps_per_unit). */
void StepperMotor::moveTo(double position) {
  // if (!limitSwitch.isValid(position)) return;
  int32_t steps = static_cast<int32_t>(position * general.steps_per_unit);
  moveToSteps(steps);
}

/** @brief Move by an offset in user units. */
void StepperMotor::moveBy(double offset) {
  int32_t steps = static_cast<int32_t>(offset * general.steps_per_unit);
  moveBySteps(steps);
}

/** @brief Set driver XTARGET directly (signed 32-bit). */
void StepperMotor::moveToSteps(int32_t steps) {
  driver.XTARGET(static_cast<int32_t>(steps));
}

/** @brief Increment XTARGET by signed steps. */
void StepperMotor::moveBySteps(int32_t steps) {
  int32_t target = static_cast<int32_t>(driver.XACTUAL()) + steps;
  driver.XTARGET(static_cast<int32_t>(target));
} 

void StepperMotor::eStop() {
  // Stop STEP driving
  analogWrite(pins.step_pin, 0);
  digitalWrite(pins.step_pin, LOW);

  driver.toff(0); // Disable bridges
  status = E_STOPPED;
  init_in_progress = false;
  #if defined(BURT_DEBUG)
  Serial.print(general.name);
  Serial.println(" => E-STOP engaged");
  #endif
}

void StepperMotor::clearEStop() {
  // sanity: keep STEP low until configured
  analogWrite(pins.step_pin, 0);
  digitalWrite(pins.step_pin, LOW);

  driver.toff(3); // Reenable bridges
  last_init_kick_ms = 0;
  prepReset();
  tryReset(init_timeout_ms);
  #if defined(BURT_DEBUG)
  if (status == RETRYING || isDone(status)) Serial.println("E-STOP cleared, attempting re-init");
  #endif
} 

void StepperMotor::setDir(uint8_t direction) {
  digitalWrite(pins.dir_pin, direction);
}

void StepperMotor::setStepHz(uint32_t f_step) {
  uint32_t f_PWM = config.stepDir.double_edge ? f_step/2 : f_step;

  // Clamp PWM to configured min/max
  if (f_PWM < min_freq) f_PWM = min_freq;
  if (f_PWM > max_freq) f_PWM = max_freq;  

  analogWriteFrequency(pins.step_pin, f_PWM);
  analogWrite(pins.step_pin, 128); // 50% duty cycle
}

void StepperMotor::setMotorRps(float rps) {
  // Only valid in STEP/DIR mode
  if (mode != STEP_DIR_MODE) {
    #if defined(BURT_DEBUG)
    Serial.println("setMotorRps() ignored: motor not in STEP/DIR mode");
    #endif
    return;
  }

  // Set DIR
  setDir((rps < 0) ? HIGH : LOW);
  rps = fabsf(rps);
  // FORMULA: f_step = n_joint*G*N_step*M_res
  uint32_t f_step = static_cast<uint32_t>(
    rps*config.stepDir.gear_ratio*static_cast<float>(steps_per_rotation)*static_cast<float>(mres) + 0.5f
  );
  setStepHz(f_step);
}
