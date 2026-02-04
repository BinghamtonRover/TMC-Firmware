#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "TmcStepper.h"
#include "limit.h"

/**
 * @file BURT_TMC.h
 * @brief Stepper motor driver abstraction for TMC5160 supporting STEP/DIR and
 *        internal ramp (position) modes.
 */

/** Physical and kinematic constants */
constexpr float    pi                   = 3.141592653589793f;
constexpr uint16_t steps_per_rotation   = 200;
constexpr uint16_t degrees_per_rotation = 360;
constexpr float    radians_per_rotation = 2.0f * pi;
constexpr uint16_t mres                 = 16;               /**< Microstep resolution factor (MRES) */
constexpr int      microsteps_per_step  = 256;              /**< Microsteps per full step */
constexpr unsigned min_freq             = 1000;  // Change after testing
constexpr unsigned max_freq             = 20000; // Change after testing

constexpr float microsteps_per_radian = microsteps_per_step * steps_per_rotation / radians_per_rotation;
constexpr float microsteps_per_degree = microsteps_per_step * steps_per_rotation / degrees_per_rotation;

/**
 * @brief Pins required by the low-level driver when operating in STEP/DIR mode
 */
struct StepperMotorPins {
  const uint8_t chip_select; /**< SPI chip select pin for TMC5160 */
  const uint8_t step_pin;    /**< STEP pin (for STEP/DIR mode) */
  const uint8_t dir_pin;     /**< DIR  pin (for STEP/DIR mode) */
};

enum DriverMode {
  STEP_DIR_MODE,
  INT_RAMP_MODE,
};

struct StepperGeneralConfig {
  const char* name;
  float       steps_per_unit;
};

struct StepDirConfig {
  // Kinematics
  float gear_ratio;

  // Double Edge
  bool double_edge;

  // Current and standstill behavior
  int     run_current_scale;
  int     hold_current_scale;
  uint8_t ihold_delay_scale;

  // Wiring direction
  bool invert_dir;

  // Control Mode
  // StealthChop/SpreadCycle (Higher Speed control mode) Thresholds
  // If needed: set spread_cycle_start above _stop for hysteresis
  bool     stealth_chop_en;
  uint32_t spread_cycle_start_thrs;
};

struct InternalRampConfig {
  int current;
  int speed;
  int acceleration;
};

enum DriverStatus : uint8_t {
  // Bit layout: [7] = error, [6:5] = E Stop, [4] = success, [3:0] = subtype
  STP_DIR_OK    = 0x10 | 0x01, // 0001 0001 - Driver is in STEP/DIR Mode and ready to go 
  POS_OK        = 0x10 | 0x02, // 0001 0010 - Internal Position mode good to go

  RETRYING      = 0x00 | 0x01, // 0000 0001 - Nothing on last attempt, still trying 
  RETRYING_COMM = 0x00 | 0x02, // 0000 0010 - Comm error on last attempt, still trying 
  RETRYING_ENN  = 0x00 | 0x03, // 0000 0011 - !(drv_enn) on last attempt, still trying 
  RETRYING_MODE = 0x00 | 0x04, // 0000 0100 - Incorrect sd_mode on last attempt, rechecking

  TIMEOUT       = 0x80 | 0x01, // 1000 0001 - Timeout with no response 
  COMM_ERR      = 0x80 | 0x02, // 1000 0010 - Timeout with Driver Communication Error 
  ENN_ERR       = 0x80 | 0x03, // 1000 0011 - Timeout with Motor is not hardware enabled 
  MODE_ERR      = 0x80 | 0x04, // 1000 0100 - Incorrect sd_mode (hardware trace error)
  
  E_STOPPED     = 0x60 | 0x00, // 0110 0000 - Not settable in check_driver, latches in e_stop
};

class StepperMotor {
private: 
  inline static uint8_t init_success_cntr = 0;

  StepperGeneralConfig general;
  StepperMotorPins     pins;
  TMC5160Stepper       driver;
  DriverMode           mode;
  union { 
    StepDirConfig stepDir;
    InternalRampConfig ramp;
  } config;

  // Vars for check_driver/resets 
  static constexpr uint32_t init_timeout_ms   = 50;
  static constexpr uint32_t loop_timeout_ms   = 25;
  static constexpr uint8_t  retry_delay_ms    = 10;
  uint32_t                  start_time_ms     = 0;
  uint32_t                  last_check_ms     = 0;
  uint32_t                  last_init_kick_ms = 0;
  bool                      init_in_progress  = false;
  DriverStatus              status            = RETRYING;
  DriverStatus              prev_status       = RETRYING; /**< last reported status, used for transition logging */

  static constexpr uint32_t INIT_KICK_PERIOD_MS = 1000;

  static inline const char* statusToString(DriverStatus s) {
    switch (s) {
      case STP_DIR_OK:    return "STEP/DIR Mode: Success";
      case POS_OK:        return "Internal POS/VEL Mode: Success";
      case RETRYING:      return "Retrying Driver Check";
      case RETRYING_COMM: return "Retrying (Comm)";
      case RETRYING_ENN:  return "Retrying (ENN)";
      case RETRYING_MODE: return "Retrying (Wrong SD_MODE)";
      case TIMEOUT:       return "Timeout";
      case COMM_ERR:      return "Timeout + Comm Error";
      case ENN_ERR:       return "Timeout + ENN Error";
      case MODE_ERR:      return "Timeout + Wrong SD_MODE; check trace on TMC";
      case E_STOPPED:     return "Driver Software E-Stop is latched, reset the driver";
      default:            return "Unknown";
    }
  }

  void prepReset();
  void tryReset(const unsigned timeout);
  void checkDriver(const unsigned timeout);
  void writeSettings();

public:
  StepperMotor(const StepperGeneralConfig& g,
               const StepperMotorPins& p,
               const StepDirConfig& cfg);
  StepperMotor(const StepperGeneralConfig& g,
               const StepperMotorPins& p,
               const InternalRampConfig& cfg);

  static inline uint8_t getInitSuccessCount()  { return init_success_cntr; }
  static inline bool isSuccess(DriverStatus s) { return (s & 0x10) != 0; }
  static inline bool isError(DriverStatus s)   { return (s & 0x80) != 0; }
  static inline bool isDone(DriverStatus s)    { return isSuccess(s) || isError(s) || (s == E_STOPPED); }

  /** @brief Is the driver currently driving toward a target? */
  bool isMoving();
  /** @brief Current step counter (driver XACTUAL). Signed 32-bit to allow large/negative counts. */
  int32_t currentSteps();
  /** @brief Current target step counter (driver XTARGET). Signed 32-bit to allow large/negative counts. */
  int32_t targetSteps();
  /** @brief Position in user units (steps/units configured in StepperGeneralConfig) */
  double currentPosition();
  double targetPosition();

  /** @brief Prepare pins and initial state (call early in setup) */
  void preSetup();
  /** @brief Begin initialization; non-blocking init is performed by repeated calls to update() */
  void setup();
  void calibrate();
  void update();
  void stop();
  void block();

  void moveTo(double position);
  void moveBy(double offset);
  void moveToSteps(int32_t steps);
  void moveBySteps(int32_t steps);

  void eStop();
  void clearEStop();

  /** @brief Read-only access to the driver's last known status */
  DriverStatus getStatus() const { return status; }

  /**
   * @brief Non-blocking: starts init and returns immediately. Call `update()` repeatedly
   * to continue initialization. For convenience, call `waitForInit(timeout_ms)` to block
   * until init is finished or timed out.
   * @param timeout_ms maximum time to wait (0 = wait forever)
   * @return true if initialization succeeded, false on timeout or error
   */
  bool waitForInit(uint32_t timeout_ms = 0);

  /** @brief Check if initialization is currently in progress (non-blocking init). */
  bool isInitInProgress() const { return init_in_progress; }

  /**
   * @brief Set motor speed in revolutions per second (STEP/DIR only).
   * @note Negative values drive reverse direction; function is a no-op in other modes.
   */
  void setMotorRps(float rps);
  void setDir(uint8_t direction);
  void setStepHz(uint32_t f_step);
};
