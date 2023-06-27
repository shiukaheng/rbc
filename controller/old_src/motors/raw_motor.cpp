#include "raw_motor.h"

RawMotor::RawMotor(int lpwm_pin, int rpwm_pin, int smoothener_window_size) {
    _lpwm_pin = lpwm_pin;
    _rpwm_pin = rpwm_pin;
    _smoothener = new Smoothener(smoothener_window_size);
    pinMode(_lpwm_pin, OUTPUT);
    pinMode(_rpwm_pin, OUTPUT);
}

RawMotor::~RawMotor() {
    delete _smoothener;
}


void RawMotor::setPWMRaw(int pwm_value) {
    // Limit range between -255 and 255
    _actual_pwm = constrain(pwm_value, -255, 255); // Constrain is an Arduino function
    _use_smoothener = false;
}

void RawMotor::setPWM(int pwm_value) {
    _target_pwm = constrain(pwm_value, -255, 255);
    _use_smoothener = true;
}

void RawMotor::update() {
    if (_use_smoothener) {
        _actual_pwm = _smoothener->update(_target_pwm);
    }
    if (_actual_pwm > 0) {
        analogWrite(_rpwm_pin, _actual_pwm);
        analogWrite(_lpwm_pin, 0);
    } else {
        analogWrite(_rpwm_pin, 0);
        analogWrite(_lpwm_pin, -_actual_pwm);
    }
}