#pragma once
#include <Arduino.h>
#include <variant>
#include "TmcStepper.h"
#include "limit.h"

constexpr float    pi                   = 3.141592653589793f;
constexpr uint16_t steps_per_rotation   = 200;
constexpr uint16_t degrees_per_rotation = 360;
constexpr float    radians_per_rotation = 2.0f * pi;
constexpr uint16_t mres                 = 16;
constexpr int      microsteps_per_step  = 256;
constexpr unsigned min_freq             = 1000;  // Change after testing
constexpr unsigned max_freq             = 20000; // Change after testing
constexpr unsigned retry_delay_ms       = 10;    // Tune after testing

constexpr float microsteps_per_radian = microsteps_per_step * steps_per_rotation / radians_per_rotation;
constexpr float microsteps_per_degree = microsteps_per_step * steps_per_rotation / degrees_per_rotation;

struct StepperMotorPins {
  const uint8_t chip_select;
  const uint8_t step_pin;
  const uint8_t dir_pin;
};

enum DriverMode {
  STEP_DIR_MODE,
  INT_POS_MODE,
};

struct StepperGeneralConfig {
  const char* name;
  DriverMode  mode;
  float       steps_per_unit;
}

struct StepDirConfig {
  // Kinematics
  const float gear_ratio;

  // Double Edge
  const bool double_edge;

  // Current and standstill behavior
  const int     run_current_scale;
  const int     hold_current_scale;
  const uint8_t ihold_delay_scale;

  // Wiring direction
  const bool invert_dir;

  // Control Mode
  // StealthChop/SpreadCycle (Higher Speed control mode) Thresholds
  // If needed: set spread_cycle_start above _stop for hysteresis
  const bool     stealth_chop_en;
  const uint32_t spread_cycle_start_thrs;
};

struct InternalRampConfig {
  int         current;
  int         speed;
  int         acceleration;
};

enum DriverStatus : uint8_t {
  // Bit layout: [7] = error, [6:5] = E Stop, [4] = success, [3:0] = subtype
  STP_DIR_OK    = 0x10 | 0x01, // 0001 0001 - Driver is in STEP/DIR Mode and ready to go 
  POS_OK        = 0x10 | 0x02, // 0001 0010 - Internal Position mode good to go
  RETRYING      = 0x00 | 0x01, // 0000 0001 - Nothing on last attempt, still trying 
  RETRYING_COMM = 0x00 | 0x02, // 0000 0010 - Comm error on last attempt, still trying 
  RETRYING_ENN  = 0x00 | 0x03, // 0000 0011 - !(drv_enn) on last attempt, still trying 
  TIMEOUT       = 0x80 | 0x01, // 1000 0001 - Timeout with no response 
  COMM_ERR      = 0x80 | 0x02, // 1000 0010 - Timeout with Driver Communication Error 
  ENN_ERR       = 0x80 | 0x03, // 1000 0011 - Timeout with Motor is not hardware enabled 
  E_STOPPED     = 0x60 | 0x00, // 0110 0000 - Not settable in check_driver, latches in e_stop
};

class StepperMotor {
private:
  StepperGeneralConfig general;
  StepperMotorPins     pins;
  TMC5160Stepper       driver;
  union { 
    StepDirConfig stepDir;
    InternalRampConfig ramp;
  } config;

  // Vars for check_driver
  static constexpr uint32_t timeout_ms     = 500;
  static constexpr uint8_t  retry_delay_ms = 25;
  uint32_t                  start_time_ms  = 0;
  uint32_t                  last_check_ms  = 0;
  bool                      done_flag      = false;
  DriverStatus              status         = RETRYING;
  DriverMode                mode;

  static inline const char* statusToString(DriverStatus s) {
    switch (s) {
      case STP_DIR_OK:    return "STEP/DIR Mode: Success";
      case POS_OK:        return "Internal POS/VEL Mode: Success";
      case RETRYING:      return "Retrying Driver Check";
      case RETRYING_COMM: return "Retrying (Comm)";
      case RETRYING_ENN:  return "Retrying (ENN)";
      case TIMEOUT:       return "Timeout (500 ms)";
      case COMM_ERR:      return "Timeout + Comm Error";
      case ENN_ERR:       return "Timeout + ENN Error";
      case E_STOPPED:     return "Driver Software E-Stop is latched, reset the driver";
      default:            return "Unknown";
    }
  }

  void resetDriver();
  void checkDriver();
  void writeSettings();

public:
  StepperMotor(StepperMotorPins pins, StepDirConfig      config);
  StepperMotor(StepperMotorPins pins, InternalRampConfig config);

  bool   isMoving();
  int    currentSteps();
  int    targetSteps();
  double currentPosition();
  double targetPosition();

  void preSetup();
  void setup();
  void calibrate();
  void update();
  void stop();
  void block();

  void moveTo(double position);
  void moveBy(double offset);
  void moveToSteps(int steps);
  void moveBySteps(int steps);

  void eStop();
  void clearEStop();

  void setMotorRps(float rps);
  void setDir(uint8_t direction);
  void setStepHz(uint32_t f_step);
};