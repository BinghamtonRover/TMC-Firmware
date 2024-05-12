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
  bool isValid(double position);
  bool isAttached();
};
