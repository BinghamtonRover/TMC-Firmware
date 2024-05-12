#include "limit.h"

bool LimitSwitch::isPressed() {
  return pin != -1 && digitalRead(pin) == triggeredValue;
}

bool LimitSwitch::isValid(double position) {
  return pin == -1 || (position <= maxLimit && position >= minLimit);
}

bool LimitSwitch::isAttached() {
  return pin != -1;
}
