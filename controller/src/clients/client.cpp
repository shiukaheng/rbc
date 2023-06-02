#include "client.h"

// PID Client for a single motor

SerialPIDClient::SerialPIDClient(PIDMotorConfig config) {
    _motor = new PIDMotor(config);
    _inputBuffer = "";
}

SerialPIDClient::~SerialPIDClient() {
    delete _motor;
}

void SerialPIDClient::isr() {
    _motor->isr();
}

void SerialPIDClient::update() {
    _motor->update();
    Serial.println(_motor->getRPS());
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        _inputBuffer += inChar;
        if (inChar == '\n') {
            // Parse the input buffer
            int pwm = _inputBuffer.toInt();
            // TODO: Set the speed
            _motor->setRPS(pwm);
            // Clear the buffer
            _inputBuffer = "";
        }
    }
}

// PWM Client for a single motor

SerialPWMClient::SerialPWMClient(MotorConfig config) {
    _inputBuffer = "";
    _encoderReader = new EncoderReader(config.motor_pinout.hall_a_pin, config.motor_pinout.hall_b_pin, config.ppr, config.gear_ratio);
    _motor = new RawMotor(config.motor_pinout.lpwm_pin, config.motor_pinout.rpwm_pin, config.smoothener_window_size);
}

SerialPWMClient::~SerialPWMClient() {
    delete _encoderReader;
    delete _motor;
}

void SerialPWMClient::isr() {
    _encoderReader->isr();
}

void SerialPWMClient::update() {
    _encoderReader->update();
    _motor->update();
    Serial.println(_encoderReader->getRPS());
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        _inputBuffer += inChar;
        if (inChar == '\n') {
            // Parse the input buffer
            int pwm = _inputBuffer.toInt();
            _motor->setPWM(pwm);
            // Clear the buffer
            _inputBuffer = "";
        }
    }
}