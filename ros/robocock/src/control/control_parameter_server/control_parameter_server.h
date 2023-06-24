#include "ros/ros.h"
#include "robocock/WheelPIDParameters.h"
#include "robocock/WheelAccumulatedI.h"
#include "robocock/WheelStates.h"

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
 * - wheel1_encoder_acceleration_threshold (float)
 * - wheel2_encoder_acceleration_threshold (float)
 * - wheel3_encoder_acceleration_threshold (float)
 * - wheel4_encoder_acceleration_threshold (float)
 * - wheel1_i_initial (float)
 * - wheel2_i_initial (float)
 * - wheel3_i_initial (float)
 * - wheel4_i_initial (float)
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
    float encoder_acceleration_threshold;
    float i_initial;
};

class ControlParameterServer {
    private:
        MotorParams _wheel1 = {0.0, 0.0, 0.0, 200.0, 0.0};
        MotorParams _wheel2 = {0.0, 0.0, 0.0, 200.0, 0.0};
        MotorParams _wheel3 = {0.0, 0.0, 0.0, 200.0, 0.0};
        MotorParams _wheel4 = {0.0, 0.0, 0.0, 200.0, 0.0};
        ros::Publisher _wheel_pid_parameters_pub;
        ros::Publisher _wheel_accumulated_i_pub;
        ros::NodeHandle _nh;
        ros::Timer _timer;
        
    public:
        ControlParameterServer() : _nh("control_parameter_server") {
            
            // Initialize the publisher and subscriber
            _wheel_pid_parameters_pub = _nh.advertise<robocock::WheelPIDParameters>("/wheel_pid_parameters", 1000);
            _wheel_accumulated_i_pub = _nh.advertise<robocock::WheelAccumulatedI>("/wheel_i_accum", 1000);

            // Initialize the parameters of the wheels
            _nh.param<float>("wheel1_p", _wheel1.p, 0.0);
            _nh.param<float>("wheel1_i", _wheel1.i, 0.0);
            _nh.param<float>("wheel1_d", _wheel1.d, 0.0);
            _nh.param<float>("wheel1_encoder_acceleration_threshold", _wheel1.encoder_acceleration_threshold, 200.0);
            _nh.param<float>("wheel1_i_initial", _wheel1.i_initial, 0.0);
            _nh.param<float>("wheel2_p", _wheel2.p, 0.0);
            _nh.param<float>("wheel2_i", _wheel2.i, 0.0);
            _nh.param<float>("wheel2_d", _wheel2.d, 0.0);
            _nh.param<float>("wheel2_encoder_acceleration_threshold", _wheel2.encoder_acceleration_threshold, 200.0);
            _nh.param<float>("wheel2_i_initial", _wheel2.i_initial, 0.0);
            _nh.param<float>("wheel3_p", _wheel3.p, 0.0);
            _nh.param<float>("wheel3_i", _wheel3.i, 0.0);
            _nh.param<float>("wheel3_d", _wheel3.d, 0.0);
            _nh.param<float>("wheel3_encoder_acceleration_threshold", _wheel3.encoder_acceleration_threshold, 200.0);
            _nh.param<float>("wheel3_i_initial", _wheel3.i_initial, 0.0);
            _nh.param<float>("wheel4_p", _wheel4.p, 0.0);
            _nh.param<float>("wheel4_i", _wheel4.i, 0.0);
            _nh.param<float>("wheel4_d", _wheel4.d, 0.0);
            _nh.param<float>("wheel4_encoder_acceleration_threshold", _wheel4.encoder_acceleration_threshold, 200.0);
            _nh.param<float>("wheel4_i_initial", _wheel4.i_initial, 0.0);


            // Wait for /wheel_states to be published
            ROS_INFO("Waiting for /wheel_states to be published...");
            ros::topic::waitForMessage<robocock::WheelStates>("/wheel_states");
            ROS_INFO("Received /wheel_states, publishing initial parameters once to /wheel_pid_parameters and /wheel_i_accum");

            // Publish parameters to topics
            publishWheelPIDParameters();
            publishWheelAccumulatedI();
        }
        /**
         * Called to publish the parameters to the /wheel_pid_parameters topic (Server -> Arduino)
        */
        void publishWheelPIDParameters() {
            robocock::WheelPIDParameters msg;
            msg.wheel1_p = _wheel1.p;
            msg.wheel1_i = _wheel1.i;
            msg.wheel1_d = _wheel1.d;
            msg.wheel1_encoder_acceleration_threshold = _wheel1.encoder_acceleration_threshold;
            msg.wheel2_p = _wheel2.p;
            msg.wheel2_i = _wheel2.i;
            msg.wheel2_d = _wheel2.d;
            msg.wheel2_encoder_acceleration_threshold = _wheel2.encoder_acceleration_threshold;
            msg.wheel3_p = _wheel3.p;
            msg.wheel3_i = _wheel3.i;
            msg.wheel3_d = _wheel3.d;
            msg.wheel3_encoder_acceleration_threshold = _wheel3.encoder_acceleration_threshold;
            msg.wheel4_p = _wheel4.p;
            msg.wheel4_i = _wheel4.i;
            msg.wheel4_d = _wheel4.d;
            msg.wheel4_encoder_acceleration_threshold = _wheel4.encoder_acceleration_threshold;
            _wheel_pid_parameters_pub.publish(msg);
        }
        void publishWheelAccumulatedI() {
            robocock::WheelAccumulatedI msg;
            msg.wheel1_i_accum = _wheel1.i_initial;
            msg.wheel2_i_accum = _wheel2.i_initial;
            msg.wheel3_i_accum = _wheel3.i_initial;
            msg.wheel4_i_accum = _wheel4.i_initial;
            _wheel_accumulated_i_pub.publish(msg);
        }
};