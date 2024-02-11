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

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  dynamic_conveyor_controller::DynamicConveyorController, controller_interface::ControllerInterface)