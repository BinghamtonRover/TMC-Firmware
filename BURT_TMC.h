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

class StepperMotor {
  private: 
    StepperMotorPins   pins;
		TMC5160Stepper     driver;
    std::variant<StepDirConfig, InternalRampConfig> config;

    void reset_driver();
    void check_driver();
    void write_settings();

    bool step_dir_mode;

  public: 
    StepperMotor(StepperMotorPins pins, StepDirConfig config);
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
};
