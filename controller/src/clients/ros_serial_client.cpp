#include "ros_serial_client.h"

// PID client for robocock wheels

ROSSerialClient::ROSSerialClient(RBCConfig config) {
    _motor1 = new PIDMotor(config.motor_configs[0]);
    _motor2 = new PIDMotor(config.motor_configs[1]);
    _motor3 = new PIDMotor(config.motor_configs[2]);
    _motor4 = new PIDMotor(config.motor_configs[3]);
    _nh.getHardware()->setBaud(115200);
    _nh.initNode();
    _nh.advertise(_wheel_states_pub);
    _nh.subscribe(_target_wheel_velocities_sub);
    _nh.subscribe(_wheel_pid_parameters_sub);
    _motor1->getPID().setOutputLimits(0,0);
    _motor2->getPID().setOutputLimits(0,0);
    _motor3->getPID().setOutputLimits(0,0);
    _motor4->getPID().setOutputLimits(0,0);
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
    _motor1->getPID().setCoefficients(msg.wheel1_p, msg.wheel1_i, msg.wheel1_d);
    _motor2->getPID().setCoefficients(msg.wheel2_p, msg.wheel2_i, msg.wheel2_d);
    _motor3->getPID().setCoefficients(msg.wheel3_p, msg.wheel3_i, msg.wheel3_d);
    _motor4->getPID().setCoefficients(msg.wheel4_p, msg.wheel4_i, msg.wheel4_d);
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

    _wheel_states_msg.wheel1_velocity = rpm1;
    _wheel_states_msg.wheel1_setpoint = _motor1->setpoint;
    _wheel_states_msg.wheel1_output = _motor1->output;

    _wheel_states_msg.wheel2_velocity = rpm2;
    _wheel_states_msg.wheel2_setpoint = _motor2->setpoint;
    _wheel_states_msg.wheel2_output = _motor2->output;

    _wheel_states_msg.wheel3_velocity = rpm3;
    _wheel_states_msg.wheel3_setpoint = _motor3->setpoint;
    _wheel_states_msg.wheel3_output = _motor3->output;

    _wheel_states_msg.wheel4_velocity = rpm4;
    _wheel_states_msg.wheel4_setpoint = _motor4->setpoint;
    _wheel_states_msg.wheel4_output = _motor4->output;

    _wheel_states_pub.publish(&_wheel_states_msg);

    _nh.spinOnce();
}
