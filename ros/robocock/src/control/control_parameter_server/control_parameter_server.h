#include "ros/ros.h"
#include "robocock/WheelPIDParameters.h"

/**
 * A simple server that takes in the following parameters:
 * - wheel1_p (float)
 * - wheel1_i (float)
 * - wheel1_d (float)
 * - wheel2_p (float)
 * - wheel2_i (float)
 * - wheel2_d (float)
 * - wheel3_p (float)
 * - wheel3_i (float)
 * - wheel3_d (float)
 * - wheel4_p (float)
 * - wheel4_i (float)
 * - wheel4_d (float)
 * - regular_update_rate (float)
 * 
 * All the values default to 0.0, except for regular_update_rate, which defaults to 1.
 *
 * The role of the server is to publish the parameters semi-regularly to /wheel_pid_parameters (which is read on the Arduino side).
 * It cannot directly be a parameter because we want to be able to change the parameters at runtime.
 * 
 * Instead, this server will take in the parameters in rosparam, and publish them regularly.
 * 
 * This server also exposes a topic, /set_wheel_pid_parameters, which takes in a WheelPIDParameters message, 
 * and is forwarded to /wheel_pid_parameters.
 * 
 * The result is that we can configure default parameters in rosparam, and then change them at runtime using the /set_wheel_pid_parameters topic.
*/

struct MotorParams {
    float p;
    float i;
    float d;
};

class ControlParameterServer {
    private:
        MotorParams _wheel1 = {0.0, 0.0, 0.0};
        MotorParams _wheel2 = {0.0, 0.0, 0.0};
        MotorParams _wheel3 = {0.0, 0.0, 0.0};
        MotorParams _wheel4 = {0.0, 0.0, 0.0};
        float _regular_update_rate = 2.0;
        ros::Publisher _wheel_pid_parameters_pub;
        ros::Subscriber _set_wheel_pid_parameters_sub;
        ros::NodeHandle _nh;
        ros::Timer _timer;
        
    public:
        ControlParameterServer() {
            
            // Initialize the publisher and subscriber1
            _wheel_pid_parameters_pub = _nh.advertise<robocock::WheelPIDParameters>("wheel_pid_parameters", 1000);
            _set_wheel_pid_parameters_sub = _nh.subscribe("set_wheel_pid_parameters", 1000, &ControlParameterServer::setWheelPIDParametersCallback, this);

            // Initialize the parameters of the wheels
            ros::param::get("~wheel1_p", _wheel1.p);
            ros::param::get("~wheel1_i", _wheel1.i);
            ros::param::get("~wheel1_d", _wheel1.d);
            ros::param::get("~wheel2_p", _wheel2.p);
            ros::param::get("~wheel2_i", _wheel2.i);
            ros::param::get("~wheel2_d", _wheel2.d);
            ros::param::get("~wheel3_p", _wheel3.p);
            ros::param::get("~wheel3_i", _wheel3.i);
            ros::param::get("~wheel3_d", _wheel3.d);
            ros::param::get("~wheel4_p", _wheel4.p);
            ros::param::get("~wheel4_i", _wheel4.i);
            ros::param::get("~wheel4_d", _wheel4.d);
            ros::param::get("~regular_update_rate", _regular_update_rate);

            // Initialize the timer
            _timer = _nh.createTimer(ros::Duration(_regular_update_rate), &ControlParameterServer::timerCallback, this);
        }
        /**
         * Called to publish the parameters to the /wheel_pid_parameters topic (Server -> Arduino)
        */
        void publishWheelPIDParameters() {
            robocock::WheelPIDParameters msg;
            msg.wheel1_p = _wheel1.p;
            msg.wheel1_i = _wheel1.i;
            msg.wheel1_d = _wheel1.d;
            msg.wheel2_p = _wheel2.p;
            msg.wheel2_i = _wheel2.i;
            msg.wheel2_d = _wheel2.d;
            msg.wheel3_p = _wheel3.p;
            msg.wheel3_i = _wheel3.i;
            msg.wheel3_d = _wheel3.d;
            msg.wheel4_p = _wheel4.p;
            msg.wheel4_i = _wheel4.i;
            msg.wheel4_d = _wheel4.d;
            _wheel_pid_parameters_pub.publish(msg);
        }
        /**
         * Called when message is received on the /set_wheel_pid_parameters topic (User -> Server)
        */
        void setWheelPIDParametersCallback(const robocock::WheelPIDParameters::ConstPtr& msg) {
            _wheel1.p = msg->wheel1_p;
            _wheel1.i = msg->wheel1_i;
            _wheel1.d = msg->wheel1_d;
            _wheel2.p = msg->wheel2_p;
            _wheel2.i = msg->wheel2_i;
            _wheel2.d = msg->wheel2_d;
            _wheel3.p = msg->wheel3_p;
            _wheel3.i = msg->wheel3_i;
            _wheel3.d = msg->wheel3_d;
            _wheel4.p = msg->wheel4_p;
            _wheel4.i = msg->wheel4_i;
            _wheel4.d = msg->wheel4_d;

            publishWheelPIDParameters(); // Forward the message to the Arduino
        }
        /**
         * Called when the timer goes off (Server -> Arduino)
        */
        void timerCallback(const ros::TimerEvent& event) {
            publishWheelPIDParameters();
        }
};