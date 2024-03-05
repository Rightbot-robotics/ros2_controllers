#ifndef DYNAMIC_CONVEYOR_CONTROLLER__PARAMETER_HANDLER_HPP_
#define DYNAMIC_CONVEYOR_CONTROLLER__PARAMETER_HANDLER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace dynamic_conveyor_controller {
struct Parameters {
    std::string left_lift_actuator_name;
    std::string right_lift_actuator_name;
    std::string left_encoder_sensor_name;
    std::string right_encoder_sensor_name;
    std::string belt_actuator_name;
    double initial_belt_speed_rpm;
    double enc_to_dist_multiplication_factor;
    double enc_to_dist_offset_factor;
    double gantry_target_distance_tolerance;
    double gantry_target_timeout;
    double belt_target_velocity_tolerance;
    double belt_target_velocity_window_time_ms;
    double belt_target_timeout;
    double enc_to_gantry_sanity_tolerance;
    double enc_to_enc_sanity_tolerance;
    double left_minus_right_travel_offset;
    double final_moveback_distance;
};


class ParameterHandler {
    public:
    ParameterHandler(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
    ~ParameterHandler() = default;
    Parameters get_parameters() { return params_; }
    bool load_parameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
    bool is_params_loaded() { return params_loaded_; }

    protected:
    Parameters params_;
    std::string plugin_name_;
    bool params_loaded_ = false;
};

template <typename ParamT>
bool strict_get_parameter(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string param_name,
    ParamT & param_value
)
{
    bool param_available;
    param_available = node->get_parameter(param_name, param_value);
    if(!param_available) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Parameter %s not available, which is required",
            param_name.c_str()
        );
    }
    return param_available;
}
}

#endif // DYNAMIC_CONVEYOR_CONTROLLER__PARAMETER_HANDLER_HPP_