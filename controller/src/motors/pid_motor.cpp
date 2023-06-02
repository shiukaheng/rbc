#include "pid_motor.h"

PIDMotor::PIDMotor(PIDMotorConfig config) {
    motor_config = config;
    // Set parameter variables
    _lpwm_pin = config.motor_pinout.lpwm_pin;
    _rpwm_pin = config.motor_pinout.rpwm_pin;
    _hall_a_pin = config.motor_pinout.hall_a_pin;
    _hall_b_pin = config.motor_pinout.hall_b_pin;
    _gear_ratio = config.gear_ratio;
    // PID variables
    setpoint = 0;
    input = 0;
    output = 0;
    // Initialize PID controller
    _pid_controller.setOutputLimits(-255, 255);
    _pid_controller.begin(&input, &output, &setpoint, config.kp, config.ki, config.kd);
    // Initialize encoder
    _encoder = new EncoderReader(config.motor_pinout.hall_a_pin, config.motor_pinout.hall_b_pin, config.ppr, config.gear_ratio);
    // Initialize motor
    _motor = new RawMotor(config.motor_pinout.lpwm_pin, config.motor_pinout.rpwm_pin, config.smoothener_window_size);
    // Initialize stall smoothener
    _stall_smoothener = new ExpTimeSmoothener(0.01);
}

PIDMotor::~PIDMotor() {
    delete _encoder;
    delete _motor;
}

void PIDMotor::setPWMRaw(int pwm_value) {
    // Disable PID
    _pid_controller.stop();
    _pid_controller.reset(); // TODO: Optimize so it only gets called on change
    _motor->setPWMRaw(pwm_value);
}

void PIDMotor::setPWM(int pwm_value) {
    _pid_controller.stop();
    _pid_controller.reset();
    _motor->setPWM(pwm_value);
}

void PIDMotor::setRPS(float rps) {
    _pid_controller.start();
    setpoint = rps;
}

void PIDMotor::update() {
    // Update encoder
    _encoder->update();
    // Update PID
    input = _encoder->getRPS(); // TODO: Integrate gear ratio as parameter to encoder class
    _pid_controller.compute();
    // Stalling prevention
    // If setpoint is 0, smoothened_rps is below threshold, but PWM is not 0, then reset PID
    double smoothened_rps = _stall_smoothener->update(input);
    if (setpoint == 0 && smoothened_rps < 0.001 && output != 0 && _stall_prevention_triggered == false) {
        _pid_controller.reset();
        _stall_prevention_triggered = true;
    } else {
        _stall_prevention_triggered = false;
    }
    // Update motor
    _motor->setPWM(output);
    _motor->update();
}

void PIDMotor::isr() {
    _encoder->isr();
}

double PIDMotor::getRPS() {
    return _encoder->getRPS();
}

double PIDMotor::getCumulativeRad() {
    return _encoder->getCumulativeRad();
}

ArduPID& PIDMotor::getPID() {
    return _pid_controller;
}