#include "dynamic_conveyor_controller/parameter_handler.hpp"


namespace dynamic_conveyor_controller
{

ParameterHandler::ParameterHandler(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
    params_loaded_ = load_parameters(node);
}

bool ParameterHandler::load_parameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
    bool status_ok = true;
    
    status_ok = status_ok && strict_get_parameter(node, "left_lift_actuator_name", params_.left_lift_actuator_name);
    status_ok = status_ok && strict_get_parameter(node, "right_lift_actuator_name", params_.right_lift_actuator_name);
    status_ok = status_ok && strict_get_parameter(node, "left_encoder_sensor_name", params_.left_encoder_sensor_name);
    status_ok = status_ok && strict_get_parameter(node, "right_encoder_sensor_name", params_.right_encoder_sensor_name);
    status_ok = status_ok && strict_get_parameter(node, "belt_actuator_name", params_.belt_actuator_name);
    status_ok = status_ok && strict_get_parameter(node, "enc_to_dist_multiplication_factor", params_.enc_to_dist_multiplication_factor);
    status_ok = status_ok && strict_get_parameter(node, "enc_to_distance_offset_factor", params_.enc_to_distance_offset_factor);
    node->get_parameter_or("initial_belt_speed_rpm", params_.initial_belt_speed_rpm, 200.0);
    node->get_parameter_or("gantry_target_distance_tolerance", params_.gantry_target_distance_tolerance, 0.0005);
    node->get_parameter_or("gantry_target_timeout", params_.gantry_target_timeout, 20.0);
    node->get_parameter_or("belt_target_velocity_tolerance", params_.belt_target_velocity_tolerance, 10.0);
    node->get_parameter_or("belt_target_velocity_window_time_ms", params_.belt_target_velocity_window_time_ms, 10.0);
    node->get_parameter_or("belt_target_timeout", params_.belt_target_timeout, 5.0);
    
    return status_ok;
}

}