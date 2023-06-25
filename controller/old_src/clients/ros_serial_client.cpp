#include "ros_serial_client.h"

// PID client for robocock wheels

RobotCore::RobotCore(RBCConfig config) {
    _motor1 = new Motor(config.motor_configs[0]);
    _motor2 = new Motor(config.motor_configs[1]);
    _motor3 = new Motor(config.motor_configs[2]);
    _motor4 = new Motor(config.motor_configs[3]);
    _nh.getHardware()->setBaud(57600);
    _nh.initNode();
    _nh.advertise(_wheel_states_pub);
    _nh.subscribe(_target_wheel_velocities_sub);
    _nh.subscribe(_wheel_pid_parameters_sub);
    _nh.subscribe(_wheel_accumulated_i_sub);
    _motor1->getPID().setOutputLimits(0,0);
    _motor2->getPID().setOutputLimits(0,0);
    _motor3->getPID().setOutputLimits(0,0);
    _motor4->getPID().setOutputLimits(0,0);
    disable_motors_flag = false;
}

RobotCore::~RobotCore() {
    delete _motor1;
    delete _motor2;
    delete _motor3;
    delete _motor4;
}

void RobotCore::isr1() {
    _motor1->isr();
}

void RobotCore::isr2() {
    _motor2->isr();
}

void RobotCore::isr3() {
    _motor3->isr();
}

void RobotCore::isr4() {
    _motor4->isr();
}

void RobotCore::_targetWheelVelocitiesCallback(const robocock::TargetWheelVelocities& msg) {
    _setpoints[0] = msg.wheel1;
    _setpoints[1] = msg.wheel2;
    _setpoints[2] = msg.wheel3;
    _setpoints[3] = msg.wheel4;
}

void RobotCore::_wheelPIDParametersCallback(const robocock::WheelPIDParameters& msg) {
    _motor1->getPID().setCoefficients(msg.wheel1_p, msg.wheel1_i, msg.wheel1_d);
    _motor2->getPID().setCoefficients(msg.wheel2_p, msg.wheel2_i, msg.wheel2_d);
    _motor3->getPID().setCoefficients(msg.wheel3_p, msg.wheel3_i, msg.wheel3_d);
    _motor4->getPID().setCoefficients(msg.wheel4_p, msg.wheel4_i, msg.wheel4_d);
    _motor1->getEncoder().setMaxAbsAcceleration(msg.wheel1_encoder_acceleration_threshold);
    _motor2->getEncoder().setMaxAbsAcceleration(msg.wheel2_encoder_acceleration_threshold);
    _motor3->getEncoder().setMaxAbsAcceleration(msg.wheel3_encoder_acceleration_threshold);
    _motor4->getEncoder().setMaxAbsAcceleration(msg.wheel4_encoder_acceleration_threshold);
}

void RobotCore::_wheelAccumulatedICallback(const robocock::WheelAccumulatedI& msg) {
    _motor1->getPID().iOut = msg.wheel1_i_accum;
    _motor2->getPID().iOut = msg.wheel2_i_accum;
    _motor3->getPID().iOut = msg.wheel3_i_accum;
    _motor4->getPID().iOut = msg.wheel4_i_accum;
}

void RobotCore::update() {

    // Set motor setpoints
    if (disable_motors_flag) {
        _motor1->setRPS(0);
        _motor2->setRPS(0);
        _motor3->setRPS(0);
        _motor4->setRPS(0);
    } else {
        _motor1->setRPS(_setpoints[0]);
        _motor2->setRPS(_setpoints[1]);
        _motor3->setRPS(_setpoints[2]);
        _motor4->setRPS(_setpoints[3]);
    }

    _motor1->update();
    _motor2->update();
    _motor3->update();
    _motor4->update();

    _wheel_states_msg.wheel1_velocity = _motor1->getRPS();
    _wheel_states_msg.wheel1_setpoint = _motor1->setpoint;
    _wheel_states_msg.wheel1_output = _motor1->output;
    _wheel_states_msg.wheel1_position = _motor1->getCumulativeRad();
    _wheel_states_msg.wheel1_i_accum = _motor1->getPID().iOut;

    _wheel_states_msg.wheel2_velocity = _motor2->getRPS();
    _wheel_states_msg.wheel2_setpoint = _motor2->setpoint;
    _wheel_states_msg.wheel2_output = _motor2->output;
    _wheel_states_msg.wheel2_position = _motor2->getCumulativeRad();
    _wheel_states_msg.wheel2_i_accum = _motor2->getPID().iOut;

    _wheel_states_msg.wheel3_velocity = _motor3->getRPS();
    _wheel_states_msg.wheel3_setpoint = _motor3->setpoint;
    _wheel_states_msg.wheel3_output = _motor3->output;
    _wheel_states_msg.wheel3_position = _motor3->getCumulativeRad();
    _wheel_states_msg.wheel3_i_accum = _motor3->getPID().iOut;

    _wheel_states_msg.wheel4_velocity = _motor4->getRPS();
    _wheel_states_msg.wheel4_setpoint = _motor4->setpoint;
    _wheel_states_msg.wheel4_output = _motor4->output;
    _wheel_states_msg.wheel4_position = _motor4->getCumulativeRad();
    _wheel_states_msg.wheel4_i_accum = _motor4->getPID().iOut;

    _wheel_states_pub.publish(&_wheel_states_msg);

    _nh.spinOnce();
}

void RobotCore::emergencyStop() {
    disable_motors_flag = true;
    _motor1->getPID().reset();
    _motor2->getPID().reset();
    _motor3->getPID().reset();
    _motor4->getPID().reset();
    _motor1->setRPS(0);
    _motor2->setRPS(0);
    _motor3->setRPS(0);
    _motor4->setRPS(0);
}

void RobotCore::resume() {
    disable_motors_flag = false;
}