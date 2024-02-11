#ifndef DYNAMIC_CONVEYOR_CONTROLLER__PARAMETER_HANDLER_HPP_
#define DYNAMIC_CONVEYOR_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace dynamic_conveyor_controller {
    struct Parameters {
        std::string conveyor_lift_left_actuator_name;
        double base_desired_linear_vel;
        double max_linear_accel;
        double max_jerk;
        double min_approach_linear_velocity;
        double min_linear_velocity;
        double MIN_T;
        double MAX_T;
        double T_DT;
        double S_MIN;
        double controller_frequency;
        double control_duration;
        double lookahead_dist;
        double transform_tolerance;
        double pre_correction_timeout;
    };

    
    class ParameterHandler {
        public:
        ParameterHandler(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string &plugin_name);
        ~ParameterHandler() = default;
        Parameters * getParameters() { return &params_; }

        protected:
        Parameters params_;
        std::string plugin_name_;
    };
}

#endif // DYNAMIC_CONVEYOR_CONTROLLER__PARAMETER_HANDLER_HPP_