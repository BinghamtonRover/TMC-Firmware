#pragma once
#include <Arduino.h>
#include "TmcStepper.h"

#include "limit.h"

constexpr float    pi                 = 3.141592653589793f;
constexpr uint16_t stepsPerRotation   = 200;
constexpr uint16_t degreesPerRotation = 360;
constexpr float    radiansPerRotation = 2.0f * pi;
constexpr uint16_t mres               = 16;
constexpr int      microstepsPerStep = 256;

constexpr float microstepsPerRadian = microstepsPerStep * stepsPerRotation / radiansPerRotation;
constexpr float microstepsPerDegree = microstepsPerStep * stepsPerRotation / degreesPerRotation;

struct StepperMotorPins {
  const uint8_t chipSelect;
  const uint8_t step_pin;
  const uint8_t dir_pin;
};

struct StepperMotorConfig {
  String name;
  int current;
  int speed;
  int acceleration;
  double stepsPerUnit;
};

class StepperMotor {
  private: 
    StepperMotorPins pins;
    StepperMotorConfig config;
		TMC5160Stepper driver;

    void reset_driver();
    void check_driver();
    void write_settings();

  public: 
    LimitSwitch limitSwitch;
    StepperMotor(StepperMotorPins pins, StepperMotorConfig config);
    StepperMotor(StepperMotorPins pins, StepperMotorConfig config, LimitSwitch limitSwitch);

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
