#include "ros_serial_client.h"

// PID client for robocock wheels

ROSSerialClient::ROSSerialClient(RBCConfig config) {
    _motor1 = new PIDMotor(config.motor_configs[0]);
    _motor2 = new PIDMotor(config.motor_configs[1]);
    _motor3 = new PIDMotor(config.motor_configs[2]);
    _motor4 = new PIDMotor(config.motor_configs[3]);
}

ROSSerialClient::~ROSSerialClient() {
    delete _motor1;
    delete _motor2;
    delete _motor3;
    delete _motor4;
}

void ROSSerialClient::isr1() {
    _motor1->isr();
}

void ROSSerialClient::isr2() {
    _motor2->isr();
}

void ROSSerialClient::isr3() {
    _motor3->isr();
}

void ROSSerialClient::isr4() {
    _motor4->isr();
}

void ROSSerialClient::update() {
    _motor1->update();
    _motor2->update();
    _motor3->update();
    _motor4->update();
}
