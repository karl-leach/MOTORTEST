#include "X27Motor.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "driver/gpio.h"
#define FAST_HIGH 1
#define FAST_LOW 0
static inline void writePinFast(int pin, uint8_t level) {
  gpio_set_level((gpio_num_t)pin, level);
}
#include "freertos/portmacro.h"
#if !defined(portMUX_INITIALIZER_UNLOCKED)
// ensure definition exists
#define portMUX_INITIALIZER_UNLOCKED (portMUX_TYPE) {0}
#endif
static portMUX_TYPE x27_lock = portMUX_INITIALIZER_UNLOCKED;
#define X27_ENTER_CRITICAL() portENTER_CRITICAL(&x27_lock)
#define X27_EXIT_CRITICAL() portEXIT_CRITICAL(&x27_lock)
#else
#define FAST_HIGH HIGH
#define FAST_LOW LOW
static inline void writePinFast(int pin, uint8_t level) {
  digitalWrite(pin, level);
}
#define X27_ENTER_CRITICAL() taskENTER_CRITICAL()
#define X27_EXIT_CRITICAL() taskEXIT_CRITICAL()
#endif

X27Motor::X27Motor(int ain1, int ain2, int bin1, int bin2, int standby, bool standbyActiveHigh)
  : _ain1(ain1), _ain2(ain2), _bin1(bin1), _bin2(bin2), _standby(standby), _standbyActiveHigh(standbyActiveHigh)
{
  _minVal = 0;
  _maxVal = 100;
  _fullSteps = 600; // default full travel steps as provided
  _currentStep = 0;
  _targetStep = 0;
  _homed = false;
  _stepDelayMs = 1; // default minimal delay for max speed
  _taskHandle = nullptr;
}

void X27Motor::setStandby(bool enable) {
  if (enable) writePinFast(_standby, _standbyActiveHigh ? FAST_HIGH : FAST_LOW);
  else writePinFast(_standby, _standbyActiveHigh ? FAST_LOW : FAST_HIGH);
}

void X27Motor::begin() {
  pinMode(_ain1, OUTPUT);
  pinMode(_ain2, OUTPUT);
  pinMode(_bin1, OUTPUT);
  pinMode(_bin2, OUTPUT);
  pinMode(_standby, OUTPUT);
  // enable driver
  writePinFast(_standby, _standbyActiveHigh ? FAST_HIGH : FAST_LOW);

  // ensure outputs off
  applySequence(0);



  // create background task
  if (!_taskHandle) {
    xTaskCreatePinnedToCore(taskRun, "X27MotorTask", 2048, this, 3, &_taskHandle, 1);
    _currentStep = 600;
    setPosition(_minVal,true); // move to minVal position after homing
  }
}

void X27Motor::setMinMaxValue(int minVal, int maxVal) {
  if (maxVal <= minVal) return;
  _minVal = minVal;
  _maxVal = maxVal;
}

void X27Motor::setFullTravelSteps(uint32_t steps) {
  if (steps == 0) return;
  _fullSteps = steps;
}

void X27Motor::setStepDelay(uint16_t ms) {
  if (ms == 0) ms = 1;
  _stepDelayMs = ms;
}

void X27Motor::setPosition(int value, bool isHoming) {

  if(!_homed && !isHoming) {
    // If not homed and not a homing call, ignore setPosition calls
    Serial.println("Warning: setPosition called before homing. Ignoring.");
    return;
  }

  //Serial.println("setPosition called with value: " + String(value) + ", isHoming: " + String(isHoming));

  if (value < _minVal) value = _minVal;
  if (value > _maxVal) value = _maxVal;
  // map value to 0.._fullSteps
  long range = (long)_maxVal - (long)_minVal;
  if (range <= 0) return;
  long rel = (long)value - (long)_minVal;
  long target = (rel * (long)_fullSteps + range/2) / range; // rounded
  if (target < 0) target = 0;
  if (target > (long)_fullSteps) target = _fullSteps;
  X27_ENTER_CRITICAL();
  _targetStep = (int)target;
  X27_EXIT_CRITICAL();

  //Serial.println("Target step set to: " + String(_targetStep));
  //Serial.println("Current step is: " + String(_currentStep));
}

int X27Motor::getPosition() {
  // map currentStep back to user units
  long range = (long)_maxVal - (long)_minVal;
  if (range <= 0) return _minVal;
  long val = ((long)_currentStep * range + _fullSteps/2) / (long)_fullSteps;
  return (int)(val + _minVal);
}

void X27Motor::stop() {
  X27_ENTER_CRITICAL();
  _targetStep = _currentStep;
  X27_EXIT_CRITICAL();
}

bool X27Motor::isHomed() {
  return _homed;
}

void X27Motor::setHomed(bool homed) {
  _homed = homed;
  //Serial.println("setHomed called, homed = " + String(homed));
}

void X27Motor::applySequence(int seqIndex) {
  // 4-step sequence: A1,A2,B1,B2
  int s = seqIndex & 0x3;
  switch (s) {
    case 0:
      writePinFast(_ain1, FAST_HIGH); writePinFast(_ain2, FAST_LOW); writePinFast(_bin1, FAST_HIGH); writePinFast(_bin2, FAST_LOW);
      break;
    case 1:
      writePinFast(_ain1, FAST_LOW); writePinFast(_ain2, FAST_HIGH); writePinFast(_bin1, FAST_HIGH); writePinFast(_bin2, FAST_LOW);
      break;
    case 2:
      writePinFast(_ain1, FAST_LOW); writePinFast(_ain2, FAST_HIGH); writePinFast(_bin1, FAST_LOW); writePinFast(_bin2, FAST_HIGH);
      break;
    case 3:
      writePinFast(_ain1, FAST_HIGH); writePinFast(_ain2, FAST_LOW); writePinFast(_bin1, FAST_LOW); writePinFast(_bin2, FAST_HIGH);
      break;
  }
}

void X27Motor::stepForwardOnce() {
  int seq = (_currentStep + 1) & 0x3;
  applySequence(seq);
  _currentStep++;
  if (_currentStep > (int)_fullSteps) _currentStep = _fullSteps;
  //Serial.println("Stepped forward to step: " + String(_currentStep));
}

void X27Motor::stepBackwardOnce() {
  int seq = (_currentStep - 1) & 0x3;
  applySequence(seq);
  _currentStep--;
  if (_currentStep < 0) _currentStep = 0;
  //Serial.println("Stepped backward to step: " + String(_currentStep));
}

void X27Motor::taskRun(void *arg) {
  X27Motor *dev = static_cast<X27Motor*>(arg);
  const TickType_t idleDelay = pdMS_TO_TICKS(5);
  while (true) {
    int target = dev->_targetStep;
    int current = dev->_currentStep;
    if (target == current) {
      vTaskDelay(idleDelay);
      if(!dev->isHomed())
      {
        dev->setHomed(true); // ensure homed flag 
        dev->_targetStep = 0;
        dev->_currentStep = 0;
      }
      continue;
    }
    else if (target > current) {
      dev->stepForwardOnce();
    } else {
      dev->stepBackwardOnce();
    }

    vTaskDelay(pdMS_TO_TICKS(dev->_stepDelayMs));
  }
}
