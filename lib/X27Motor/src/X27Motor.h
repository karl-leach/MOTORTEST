#pragma once
#include <Arduino.h>

#ifndef MOTOR_AIN1_PIN
#define MOTOR_AIN1_PIN 18
#endif
#ifndef MOTOR_AIN2_PIN
#define MOTOR_AIN2_PIN 19
#endif
#ifndef MOTOR_BIN1_PIN
#define MOTOR_BIN1_PIN 25
#endif
#ifndef MOTOR_BIN2_PIN
#define MOTOR_BIN2_PIN 26
#endif
#ifndef MOTOR_STANDBY_PIN
#define MOTOR_STANDBY_PIN 16
#endif
#ifndef MOTOR_STANDBY_ACTIVE_HIGH
#define MOTOR_STANDBY_ACTIVE_HIGH true
#endif

class X27Motor {
public:
  X27Motor(int ain1 = MOTOR_AIN1_PIN, int ain2 = MOTOR_AIN2_PIN,
          int bin1 = MOTOR_BIN1_PIN, int bin2 = MOTOR_BIN2_PIN,
          int standby = MOTOR_STANDBY_PIN, bool standbyActiveHigh = MOTOR_STANDBY_ACTIVE_HIGH);

  void begin(bool doHoming = true);
  void setMinMaxValue(int minVal, int maxVal);
  void setFullTravelSteps(uint32_t steps);
  void setStepDelay(uint16_t ms);

  // setPosition in user units (clamped to min/max)
  void setPosition(int value);
  int getPosition();

  void stop();
  bool isHomed();

private:
  int _ain1, _ain2, _bin1, _bin2, _standby;
  bool _standbyActiveHigh;

  // mapping
  int _minVal;
  int _maxVal;
  uint32_t _fullSteps; // number of micro-steps across full range (e.g., 600)

  // stepping
  volatile int _currentStep;
  volatile int _targetStep;
  volatile bool _homed;
  uint16_t _stepDelayMs;

  TaskHandle_t _taskHandle;
  static void taskRun(void *arg);

  void applySequence(int seqIndex);
  void stepForwardOnce();
  void stepBackwardOnce();
  void setStandby(bool enable);
};
