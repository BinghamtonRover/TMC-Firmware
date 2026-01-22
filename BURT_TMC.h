#pragma once
#include <Arduino.h>
#include <variant>
#include "TmcStepper.h"

#include "limit.h"

constexpr float    pi                 = 3.141592653589793f;
constexpr uint16_t stepsPerRotation   = 200;
constexpr uint16_t degreesPerRotation = 360;
constexpr float    radiansPerRotation = 2.0f * pi;
constexpr uint16_t mres               = 16;
constexpr int      microstepsPerStep  = 256;
constexpr unsigned min_freq           = 1000; // Change after testing
constexpr unsigned max_freq           = 20000; // Change after testing

constexpr float microstepsPerRadian = microstepsPerStep * stepsPerRotation / radiansPerRotation;
constexpr float microstepsPerDegree = microstepsPerStep * stepsPerRotation / degreesPerRotation;

struct StepperMotorPins {
  const uint8_t chipSelect;
  const uint8_t step_pin;
  const uint8_t dir_pin;
};

struct StepDirConfig {
  const char* name; // Motor identifier

  // Kinematics
  const float gear_ratio;    

  // Double Edge
  const bool dedge;        

  // Current and standstill behavior
  const int     run_current_scale; 
  const int     hold_current_scale;
  const uint8_t ihold_delay_scale; 
  
  // Wiring direction
  const bool invert_dir; 
  
  // Control Mode
  // StealthChop/SpreadCycle (Higher Speed control mode) Thresholds
  // If needed: set spread_cycle_start above _stop for hysteresis
  const bool     stealthChop_en; 
  const uint32_t spread_cycle_start_thrs; 
};

struct InternalRampConfig {
  const char* name;
  int         current;
  int         speed;
  int         acceleration;
  float       stepsPerUnit;
};

enum DriverStatus : uint8_t {
  // Bit layout: [7] = error, , [6:5] = E Stop, [4] = success, [3:0] = subtype
  STP_DIR_OK    = 0x10 | 0x01, // 0001 0001 - Driver is in STEP/DIR Mode and ready to go
  POS_OK        = 0x10 | 0x02, // 0001 0010 - ON FULL INTEGRATION ENABLE THIS
  RETRYING      = 0x00 | 0x01, // 0000 0001 - Nothing on last attempt, still trying
  RETRYING_COMM = 0x00 | 0x02, // 0000 0010 - Comm error on last attempt, still trying
  RETRYING_ENN  = 0x00 | 0x03, // 0000 0011 - !(drv_enn) on last attempt, still trying
  TIMEOUT       = 0x80 | 0x01, // 1000 0001 - Timeout with no response
  COMM_ERR      = 0x80 | 0x02, // 1000 0010 - Timeout with Driver Communication Error
  ENN_ERR       = 0x80 | 0x03, // 1000 0011 - Timeout with Motor is not hardware enabled
  E_STOPPED     = 0x60 | 0x00, // 0110 0000 - Not settable in check_driver, latches in e_stop
};

enum DriverMode {
  step_dir,
  int_pos
};

class StepperMotor {
  private: 
    StepperMotorPins                                pins;
		TMC5160Stepper                                  driver;
    std::variant<StepDirConfig, InternalRampConfig> config;

    // Vars for check_driver
    static constexpr uint32_t TIMEOUT_MS = 500;
    static constexpr uint8_t  RETRY_DELAY_MS = 25;
    uint32_t                  start_time_ms = 0;
    uint32_t                  last_check_ms = 0;
    bool                      done_f = false;
    DriverStatus              status = RETRYING;
    DriverMode                mode;

    static inline const char* status_to_string(DriverStatus s) {
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
    
    void reset_driver();
    void check_driver();
    void write_settings();

  public: 
    StepperMotor(StepperMotorPins pins, StepDirConfig      config);
    StepperMotor(StepperMotorPins pins, InternalRampConfig config);

    bool isMoving();
    int currentSteps();
    int targetSteps();
    double currentPosition();
    double targetPosition();

    void presetup();
    void setup();
    void calibrate();
    void update();
    void stop();
    void block();

    void moveTo(double position);
    void moveBy(double offset);
    void moveToSteps(int steps);
    void moveBySteps(int steps);

    void e_stop();
    void clear_e_stop();

    void set_motor_rps(float rps);
    void set_dir(uint8_t direction);
    void set_step_hz(uint32_t f_step);
};
