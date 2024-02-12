#include <dynamic_conveyor_controller/dynamic_conveyor_controller.hpp>


namespace dynamic_conveyor_controller
{

DynamicConveyorController::DynamicConveyorController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DynamicConveyorController::on_init() {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorController::on_init()");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DynamicConveyorController::command_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorController::command_interface_configuration()");
    std::vector<std::string> conf_names;
    conf_names.push_back("gantry_left/position");
    conf_names.push_back("gantry_right/position");
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration DynamicConveyorController::state_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorController::state_interface_configuration()");
    std::vector<std::string> conf_names;
    conf_names.push_back("gantry_left/position");
    conf_names.push_back("gantry_right/position");
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type DynamicConveyorController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    // RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorController::update()");
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DynamicConveyorController::on_configure(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorController::on_configure()");
    ParameterHandler ph(get_node());
    if(!ph.is_params_loaded()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to load parameters");
        return controller_interface::CallbackReturn::ERROR;
    }
    params_ = ph.get_parameters();
    RCLCPP_INFO(get_node()->get_logger(), "left_lift_actuator_name: %s", params_.left_lift_actuator_name.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "right_lift_actuator_name: %s", params_.right_lift_actuator_name.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "belt_actuator_name: %s", params_.belt_actuator_name.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "initial_belt_speed_rpm: %f", params_.initial_belt_speed_rpm);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DynamicConveyorController::on_activate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorController::on_activate()");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DynamicConveyorController::on_deactivate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorController::on_deactivate()");
    return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace dynamic_conveyor_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamic_conveyor_controller::DynamicConveyorController, controller_interface::ControllerInterface)