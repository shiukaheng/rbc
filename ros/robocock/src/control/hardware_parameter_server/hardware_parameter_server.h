#include "ros/ros.h"
#include "robocock/BaseParameters.h"
#include "robocock/BaseAdaptiveState.h"
#include "robocock/BaseState.h"

#define NUM_MOTORS 4

class HardwareParameterServer {
    private:
        robocock::BaseParameters base_parameters_msg;
        robocock::BaseAdaptiveState base_adaptive_state_msg;
        ros::Publisher base_parameters_pub;
        ros::Publisher base_adaptive_state_pub;
        ros::NodeHandle _nh;
        // Generic type method for initializing a parameter
        template <typename T>
        void initParam(T& param, int motor_id, std::string name, T default_value) {
            _nh.param<T>("motor_" + std::to_string(motor_id + 1) + "/" + name, param, default_value);
        }
        bool second_order_predictor_buffer[NUM_MOTORS] = {false};
        // void syncBoolToUint() {
        //     for (int i = 0; i < NUM_MOTORS; i++) {
        //         base_parameters_msg.parameters[i].second_order_predictor = second_order_predictor_buffer[i];
        //     }
        // }
    public:
        HardwareParameterServer() : _nh("hardware_parameter_server") {
            
            // Initialize the publisher and subscriber
            base_parameters_pub = _nh.advertise<robocock::BaseParameters>("/base_parameters", 1000);
            base_adaptive_state_pub = _nh.advertise<robocock::BaseAdaptiveState>("/base_adaptive_state", 1000);

            for (int i = 0; i < NUM_MOTORS; i++) {
                initParam<float>(base_parameters_msg.parameters[i].p_in, i, "p_in", 35.0);
                initParam<float>(base_parameters_msg.parameters[i].i_in, i, "i_in", 0.0015);
                initParam<float>(base_parameters_msg.parameters[i].d_in, i, "d_in", 0.0);
                initParam<float>(base_parameters_msg.parameters[i].bias, i, "bias", 50.0);
                // initParam<float>(base_parameters_msg.parameters[i].i_accumulator_min, i, "i_accumulator_min", 0.0);
                // initParam<float>(base_parameters_msg.parameters[i].i_accumulator_max, i, "i_accumulator_max", 60.0);
                // initParam<float>(base_parameters_msg.parameters[i].output_min, i, "output_min", -255.0);
                // initParam<float>(base_parameters_msg.parameters[i].output_max, i, "output_max", 255.0);
                // // initParam<float>(base_parameters_msg.parameters[i].deadband_min, i, "deadband_min", 0.0);
                // // initParam<float>(base_parameters_msg.parameters[i].deadband_max, i, "deadband_max", 0.0);
                // initParam<float>(base_parameters_msg.parameters[i].target_update_rate, i, "target_update_rate", 30.0);
                // initParam<float>(base_parameters_msg.parameters[i].max_abs_acceleration, i, "max_abs_acceleration", 200.0);
                // initParam<float>(base_parameters_msg.parameters[i].max_abs_velocity, i, "max_abs_velocity", 20.0);
                // initParam<bool>(second_order_predictor_buffer[i], i, "second_order_predictor", false); // Required because the message type is uint and requires conversion
                // initParam<float>(base_parameters_msg.parameters[i].smoothener_alpha, i, "smoothener_alpha", 0.0001);    
                initParam<float>(base_adaptive_state_msg.adaptive_states[i].i_accumulator, i, "i_accumulator_initial", 20.0);
            }
            
            ROS_INFO("Waiting for /base_state to be published...");
            ros::topic::waitForMessage<robocock::BaseState>("/base_state");
            ROS_INFO("Received /base_state message, publishing parameters /base_parameters and /adaptive_state");

            // Publish parameters every 1 second
            ros::Rate loop_rate(1);

            while (ros::ok()) {
                ROS_INFO("Publishing parameters...");
                base_parameters_pub.publish(base_parameters_msg);
                base_adaptive_state_pub.publish(base_adaptive_state_msg);
                ros::spinOnce();
                loop_rate.sleep();
            }

            // // Delay 0.5 seconds to make sure the subscriber is ready
            // ros::Duration(0.5).sleep();

            // // syncBoolToUint();
            // base_parameters_pub.publish(base_parameters_msg);
        }
};