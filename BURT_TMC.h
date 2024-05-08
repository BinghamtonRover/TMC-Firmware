#pragma once
#include <Arduino.h>
#include "TmcStepper.h"

const double pi = 3.141592653589793;
const int microstepsPerStep = 256;
const int stepsPerRotation = 200;
const int degreesPerRotation = 360;
const double radiansPerRotation = 2 * pi;

const double stepsPerRadian = microstepsPerStep * stepsPerRotation / radiansPerRotation;
const double stepsPerDegree = microstepsPerStep * stepsPerRotation / degreesPerRotation;

struct StepperMotorPins {
  int enable;
  int chipSelect;
};

struct StepperMotorConfig {
  String name;
  int current;
  int speed;
  int acceleration;
  double stepsPerUnit;
};

struct LimitSwitch {
  int pin = -1;
  int triggeredValue;
  int direction = 1;
  bool isBlocking = true;
  double position;
  double minLimit = -INFINITY;
  double maxLimit = INFINITY;

  int offset = 0;
  bool isPressed();
};

class StepperMotor {
  private: 
    StepperMotorPins pins;
    StepperMotorConfig config;
    LimitSwitch limitSwitch;
		TMC5160Stepper driver;

    void reset_driver();
    void check_driver();
    void write_settings();

  public: 
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
