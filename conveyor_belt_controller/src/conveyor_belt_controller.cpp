#include <conveyor_belt_controller/conveyor_belt_controller.hpp>


namespace conveyor_belt_controller
{

ConveyorBeltController::ConveyorBeltController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ConveyorBeltController::on_init() {
    RCLCPP_INFO(get_node()->get_logger(), "ConveyorBeltController::on_init()");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ConveyorBeltController::command_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "ConveyorBeltController::command_interface_configuration()");
    std::vector<std::string> conf_names;
    conf_names.push_back("gantry_left/position");
    conf_names.push_back("gantry_right/position");
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration ConveyorBeltController::state_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "ConveyorBeltController::state_interface_configuration()");
    std::vector<std::string> conf_names;
    conf_names.push_back("gantry_left/position");
    conf_names.push_back("gantry_right/position");
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type ConveyorBeltController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    // RCLCPP_INFO(get_node()->get_logger(), "ConveyorBeltController::update()");
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ConveyorBeltController::on_configure(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "ConveyorBeltController::on_configure()");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConveyorBeltController::on_activate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "ConveyorBeltController::on_activate()");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConveyorBeltController::on_deactivate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "ConveyorBeltController::on_deactivate()");
    return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace conveyor_belt_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  conveyor_belt_controller::ConveyorBeltController, controller_interface::ControllerInterface)