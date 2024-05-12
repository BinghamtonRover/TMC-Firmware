#include "limit.h"

bool LimitSwitch::isPressed() {
  return isAttached() && digitalRead(pin) == triggeredValue;
}

bool LimitSwitch::isValid(double position) {
  return !isAttached() || (position <= maxLimit && position >= minLimit);
}

bool LimitSwitch::isAttached() {
  return pin != -1;
}
