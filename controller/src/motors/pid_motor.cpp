#include "pid_motor.h"

// TODO: Integrate gear ratio as parameter to encoder class

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
    _pid_controller.setBias(config.min_startup_pwm);
    _pid_controller.begin(&input, &output, &setpoint, config.kp, config.ki, config.kd);
    // Initialize encoder
    _encoder = new EncoderReaderNaive(config.motor_pinout.hall_a_pin, config.motor_pinout.hall_b_pin, config.ppr, config.gear_ratio);
    // Initialize motor
    _motor = new RawMotor(config.motor_pinout.lpwm_pin, config.motor_pinout.rpwm_pin, config.smoothener_window_size);
    // Initialize stall smoothener
    _stall_smoothener = new ExpTimeSmoothener(0.01);
    // Initialize derivative constrainer for velocity constraint
    _derivative_constrainer = new DerivativeConstrainer(20.);
}

PIDMotor::~PIDMotor() {
    delete _encoder;
    delete _motor;
    delete _stall_smoothener;
    delete _derivative_constrainer;
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
    long encoder_updated = _encoder->update(); // This is a terrible way to manage dt. It should be centrally managed in one big loop, with one dt. But for now, this will do.
    if (encoder_updated == -1) {
        return;
    }
    // Update PID
    input = _encoder->getRPS();
    // input = _derivative_constrainer->update(_encoder->getRPS(), encoder_updated / 1000000.); // Constraint acceleration to 10 rad/s^2 as what is physically possible (trying to eliminate misreadings)
    _pid_controller.compute();
    // Stalling prevention
    // If setpoint is 0, smoothened_rps is below threshold, but PWM is not 0, then force PWM to 0
    double smoothened_rps = _stall_smoothener->update(input);
    if (setpoint == 0 && abs(smoothened_rps) < 0.001 && output != 0) {
        if (_stall_prevention_triggered == false) {
            // _pid_controller.reset();
            _stall_prevention_triggered = true;
        }
        _motor->setPWM(0);
    } else {
        _stall_prevention_triggered = false;
        _motor->setPWM(output);
    }
    // Update motor
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