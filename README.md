# Stepper Motor

## The Arm

The code here controls the arm, based on input from an Xbox controller. The arm
can swivel clockwise and counter-clockwise, lift up and down, and extend in and out. The gripper can be separately lifted up and down, rotated clockwise and counter-clockwise and pinch open and closed. The following image should illustrate that.

The arm has two modes: precision and IK. In precision mode, you move the joints individually by small increments. In IK mode, you control a reticle that determines the position of the gripper in 3D space. The system provides the translations between 3D coordinates and joint angles. 

## The stepper motors

The arm uses TMC 5160 stepper motors, which have a lot of features but are complex and hard to use without adding bloat to a `.ino` sketch. This library serves to simplify all the aspects of oeprating a TMC 5160 stepper motor with the `StepperMotor` class. It's implementation builds on [`TMCStepper`](https://github.com/teemuatlut/TMCStepper/tree/Release_v1) library, which it uses as a backend. 

## Usage

This library supports two modes of operation:

- **STEP/DIR mode** — the MCU generates STEP pulses and DIR signals (good for custom control loops).
- **Internal Ramp (position) mode** — use the TMC5160's internal ramping to move to targets.

### Migration notes (important)

- Constructor API changed: create a `StepperGeneralConfig` (name + steps_per_unit), a `StepperMotorPins` struct, and pass either a `StepDirConfig` or `InternalRampConfig` depending on mode.
- Step counters and move-by/move-to step functions now use **signed 32-bit** (`int32_t`).

### Example: STEP/DIR mode (recommended for custom control)

```cpp
#include "BURT_TMC.h"

StepperGeneralConfig g { "arm_joint", 100.0f /* steps per unit */ };
StepperMotorPins pins { .chip_select = 10, .step_pin = 3, .dir_pin = 4 };
StepDirConfig cfg {
  .gear_ratio = 1.0f,
  .double_edge = false,
  .run_current_scale = 16,
  .hold_current_scale = 8,
  .ihold_delay_scale = 1,
  .invert_dir = false,
  .stealth_chop_en = true,
  .spread_cycle_start_thrs = 10000
};

StepperMotor motor(g, pins, cfg);

void setup() {
  motor.preSetup();   // config GPIOs
  motor.setup();      // start non-blocking init
  motor.waitForInit(1000); // optional blocking wait (timeout in ms)
}

void loop() {
  motor.update(); // continue init & handle retries; call from loop
}
```

### Example: Internal ramp (TMC driven position)

```cpp
InternalRampConfig ramp { .current = 100, .speed = 1000, .acceleration = 500 };
StepperMotor motor(g, pins, ramp);
// use the same setup()/update() pattern shown above
```

### Moving the motor

- `moveTo(position_in_units)` and `moveBy(offset_in_units)` accept user units (they use `steps_per_unit` from `StepperGeneralConfig`).
- `moveToSteps(int32_t steps)` and `moveBySteps(int32_t steps)` operate directly on driver step counters.

#### STEP/DIR mode notes 
- Use `setMotorRps(float rps)` to set rotational speed (revolutions per second). Negative values select reverse rotation (the function will set `DIR` accordingly).
- `setMotorRps()` is only valid in **STEP/DIR mode** — calling it in Internal Ramp mode is a no-op and will print a debug message when `BURT_DEBUG` is defined.
- You can control STEP frequency directly with `setStepHz(uint32_t f_step)`. If you configured `double_edge = true` in `StepDirConfig`, the effective PWM frequency used is `f_step/2` (this assumes a 50% duty cycle requirement for double-edge). The driver clamps frequencies to safe limits; see `min_freq`/`max_freq` in `BURT_TMC.h`.
- Example (STEP/DIR):
```cpp
motor.setMotorRps(1.5f); // 1.5 RPS forward
motor.setMotorRps(-0.5f); // 0.5 RPS reverse
// Or set exact step frequency:
motor.setStepHz(20000);
```

#### Internal Ramp mode notes 
- Use `moveTo()` / `moveBy()` / `moveToSteps()` to control position and let the TMC5160 handle acceleration/deceleration using `InternalRampConfig` parameters (`current`, `speed`, `acceleration`).
- Tune `vstart`, `vstop`, `A1`, `V1`, `AMAX`, and `VMAX` in `writeSettings()` for your mechanical load.



### Debugging & logging

- Enable debug prints by defining `BURT_DEBUG` at compile time (recommended in development):
  - In a sketch: `#define BURT_DEBUG` before including headers
- Debug prints are gated and will not show in production unless the macro is defined.

### Safety & e-stop

- Use `eStop()` to immediately disable motor outputs and latch a software e-stop. Call `clearEStop()` to try to reinitialize (may require wiring checks if hardware ENN is not asserted).

---
