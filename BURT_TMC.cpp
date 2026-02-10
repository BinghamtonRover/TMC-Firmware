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
      mode(TMC::STEP_DIR_MODE)
{
  config.stepDir = cfg;
}

StepperMotor::StepperMotor(const StepperGeneralConfig& g,
                           const StepperMotorPins& p,
                           const InternalRampConfig& cfg)
    : general(g),
      pins(p),
      driver(TMC5160Stepper(SPI, p.chip_select, 0.075)),
      mode(TMC::INT_RAMP_MODE)
{
  config.ramp = cfg;
}

/** @brief True if XTARGET != XACTUAL */
bool StepperMotor::isMoving() {
  if  (mode == TMC::INT_RAMP_MODE) {
      return driver.XTARGET() != driver.XACTUAL();
  }
  else return (step_hz != 0);
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
  if (mode == TMC::STEP_DIR_MODE) {
    pinMode(pins.step_pin, OUTPUT);
    pinMode(pins.dir_pin, OUTPUT);
    digitalWrite(pins.step_pin, LOW);
    digitalWrite(pins.dir_pin, LOW);
  }
}

void StepperMotor::prepReset() {
  if (status == TMC::E_STOPPED) return;

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

  // prep state: initialize heartbeat to trigger check immediately on next update()
  uint32_t now = millis();
  start_time_ms = now;           // Reset init timer
  last_check_ms = now - retry_delay_ms;  // Allow immediate first check
  last_heartbeat_ms = now - HEARTBEAT_INTERVAL_MS;  // Trigger heartbeat on next update()
  status = TMC::RETRYING;
}

void StepperMotor::tryReset(const unsigned timeout) {
  if (isDone(status) || status == TMC::E_STOPPED) return;

  checkDriver(timeout);

  if (isDone(status)) {
    if (isSuccess(status)) {
      ++init_success_cntr;
      writeSettings();
      #if defined(BURT_DEBUG)
      if (mode == TMC::INT_RAMP_MODE) Serial.println("Driver is in Internal Ramp Mode");
      else                            Serial.println("Driver is in STEP/DIR Mode");
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
  
  uint32_t now = millis();
  if (!start_time_ms) start_time_ms = now;
  
  DriverStatus old = status;

  // Perform IOIN check on both:
  // 1. Scheduled retry_delay_ms cadence (for state progression)
  // 2. Heartbeat-based cadence (ensures checks even if update() is infrequent)
  bool time_for_retry = now - last_check_ms >= retry_delay_ms;
  bool heartbeat_check = now - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS;
  
  if (time_for_retry || heartbeat_check) {
    if (time_for_retry) last_check_ms = now;
    if (heartbeat_check) last_heartbeat_ms = now;

    auto ioin = readIOIN();
    status = assessIOIN(ioin, true);
  }

  // Timeout check: exceeds specified window without achieving success
  if ((now - start_time_ms) > timeout && !isDone(status)) {
    if      (status == TMC::RETRYING_COMM) status = TMC::COMM_ERR;
    else if (status == TMC::RETRYING_ENN)  status = TMC::ENN_ERR;
    else if (status == TMC::RETRYING_MODE) status = TMC::MODE_ERR;
    else                                   status = TMC::TIMEOUT;
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

/**
 * @brief Periodic runtime heartbeat poll that validates driver health while
 *        the driver is operational. Runs at `HEARTBEAT_INTERVAL_MS` and will
 *        trigger a restart of the init state-machine if a fault is detected.
 */
void StepperMotor::checkHeartbeat() {
  uint32_t now = millis();
  if (now - last_heartbeat_ms < HEARTBEAT_INTERVAL_MS) return;
  last_heartbeat_ms = now;
  auto ioin = readIOIN();
  auto res = assessIOIN(ioin, false);
  if (isSuccess(res)) return; // heartbeat OK

  // Runtime detected an error: set concrete error status and restart init
  status = res;
#if defined(BURT_DEBUG)
  Serial.print(general.name); Serial.print(" heartbeat: detected fault -> ");
  Serial.println(statusToString(status));
#endif
  prepReset();
}

TMC5160Stepper::IOIN_t StepperMotor::readIOIN() {
  uint32_t raw = driver.IOIN();
  return TMC5160Stepper::IOIN_t{ raw };
}

StepperMotor::DriverStatus StepperMotor::assessIOIN(const TMC5160Stepper::IOIN_t& ioin, bool for_init) {
  if (ioin.version == 0xFF || ioin.version == 0) return for_init ? TMC::RETRYING_COMM : TMC::COMM_ERR;
  if (ioin.drv_enn) return for_init ? TMC::RETRYING_ENN : TMC::ENN_ERR;

  if (mode == TMC::STEP_DIR_MODE) {
    if (ioin.sd_mode) return TMC::STP_DIR_OK;
    return for_init ? TMC::RETRYING_MODE : TMC::MODE_ERR;
  } else {
    if (!ioin.sd_mode) return TMC::POS_OK;
    return for_init ? TMC::RETRYING_MODE : TMC::MODE_ERR;
  }
}


void StepperMotor::writeSettings() {
  switch (mode) {
    case TMC::STEP_DIR_MODE:
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
    case TMC::INT_RAMP_MODE:
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
 *
 * Implementation notes:
 *  - This function loops, calling `tryReset(loop_timeout_ms)` and then
 *    sleeping `retry_delay_ms` until `isDone(status)` returns true.
 *  - `checkDriver()` prints status transitions (and driver IOIN details) when
 *    `BURT_DEBUG` is defined, which makes it useful for diagnosing initialization
 *    failures. A timeout will also print a short message when `BURT_DEBUG` is
 *    enabled.
 *
 * Example usage:
 *  StepperMotor motor(...);
 *  motor.setup();
 *  if (!motor.waitForInit(200)) {
 *    Serial.println("Motor init failed: check wiring, EN pin, or SPI bus");
 *  }
 *
 * Caution: this is a blocking call — prefer `setup()` + periodic `update()` in
 * time-critical applications.
 *
 * @param timeout_ms maximum time to wait (0 = wait forever)
 * @return true on success, false on timeout or fatal error
 */
bool StepperMotor::waitForInit(uint32_t timeout_ms) {
  uint32_t start = millis();
  while (!isDone(status)) {
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
  if (status == TMC::E_STOPPED) return;
  if (!isDone(status)) {
    tryReset(loop_timeout_ms);
    return;
  }

  // AUTOMATIC FAULT RECOVERY: If we are in an error state, periodically attempt recovery
  // by resetting the driver and restarting initialization. This handles transient errors.
  if (isError(status)) {
    uint32_t now = millis();
    if (now - last_reinit_attempt_ms >= REINIT_ATTEMPT_PERIOD_MS) {
#if defined(BURT_DEBUG)
      Serial.print(general.name);
      Serial.println(" attempting automatic fault recovery (re-init)...");
#endif
      last_reinit_attempt_ms = now;
      prepReset();  // Restart initialization state machine
      tryReset(loop_timeout_ms);  // Perform first check immediately
    }
  }

  // Runtime heartbeat poll during normal operation
  if (isSuccess(status)) {
    checkHeartbeat();
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
  status = TMC::E_STOPPED;
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
  last_reinit_attempt_ms = 0;
  prepReset();
  tryReset(init_timeout_ms);
  #if defined(BURT_DEBUG)
  if (status == TMC::RETRYING || isDone(status)) Serial.println("E-STOP cleared, attempting re-init");
  #endif
} 

void StepperMotor::setDir(uint8_t direction) {
  digitalWrite(pins.dir_pin, direction);
}

void StepperMotor::setStepHz(uint32_t f_step) {
  uint32_t f_PWM = config.stepDir.double_edge ? f_step/2 : f_step;

  // Clamp PWM to configured min/max
  if (f_PWM > max_freq) f_PWM = max_freq;  

  analogWriteFrequency(pins.step_pin, f_PWM);
  analogWrite(pins.step_pin, 128); // 50% duty cycle

  step_hz = f_PWM;
}

void StepperMotor::setMotorRps(float rps) {
  // Only valid in STEP/DIR mode
  if (mode != TMC::STEP_DIR_MODE) {
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
