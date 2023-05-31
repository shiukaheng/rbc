#include "ros_serial_client.h"

// PID client for robocock wheels

ROSSerialClient::ROSSerialClient(RBCConfig config) {
    _motor1 = new PIDMotor(config.motor_configs[0]);
    _motor2 = new PIDMotor(config.motor_configs[1]);
    _motor3 = new PIDMotor(config.motor_configs[2]);
    _motor4 = new PIDMotor(config.motor_configs[3]);
    _nh.initNode();
    _nh.advertise(_wheel_velocities_pub);
    _nh.subscribe(_target_wheel_velocities_sub);
    _nh.subscribe(_wheel_pid_parameters_sub);
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

void ROSSerialClient::_targetWheelVelocitiesCallback(const robocock::TargetWheelVelocities& msg) {
    _motor1->setRPM(msg.wheel1);
    _motor2->setRPM(msg.wheel2);
    _motor3->setRPM(msg.wheel3);
    _motor4->setRPM(msg.wheel4);
}

void ROSSerialClient::_wheelPIDParametersCallback(const robocock::WheelPIDParameters& msg) {
    // Do nothing for now
}

void ROSSerialClient::update() {
    _motor1->update();
    _motor2->update();
    _motor3->update();
    _motor4->update();

    float rpm1 = _motor1->getRPM();
    float rpm2 = _motor2->getRPM();
    float rpm3 = _motor3->getRPM();
    float rpm4 = _motor4->getRPM();

    _wheel_velocities_msg.wheel1 = rpm1;
    _wheel_velocities_msg.wheel2 = rpm2;
    _wheel_velocities_msg.wheel3 = rpm3;
    _wheel_velocities_msg.wheel4 = rpm4;

    _wheel_velocities_pub.publish(&_wheel_velocities_msg);

    _nh.spinOnce();
}
