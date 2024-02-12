#include <dynamic_conveyor_controller/dynamic_conveyor_controller.hpp>


namespace dynamic_conveyor_controller
{

DynamicConveyorController::DynamicConveyorController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DynamicConveyorController::on_init() {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorController::on_init()");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DynamicConveyorController::command_interface_configuration() const {
    std::vector<std::string> conf_names;
    conf_names.push_back(params_.left_lift_actuator_name + "/" + "position");
    conf_names.push_back(params_.right_lift_actuator_name + "/" + "position");
    conf_names.push_back(params_.belt_actuator_name + "/" + "velocity");
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration DynamicConveyorController::state_interface_configuration() const {
    std::vector<std::string> conf_names;
    conf_names.push_back(params_.left_lift_actuator_name + "/" + "position");
    conf_names.push_back(params_.right_lift_actuator_name + "/" + "position");
    conf_names.push_back(params_.left_encoder_sensor_name + "/" + "position");
    conf_names.push_back(params_.right_encoder_sensor_name + "/" + "position");
    conf_names.push_back(params_.belt_actuator_name + "/" + "velocity");
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
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DynamicConveyorController::on_activate(
    const rclcpp_lifecycle::State &) {
    
    // Acquiring command interfaces
    std::vector<std::string> required_command_interfaces_name {
        params_.left_lift_actuator_name + "/" + "position",
        params_.right_lift_actuator_name + "/" + "position",
        params_.belt_actuator_name + "/" + "velocity",
        "hello"
    };
    if(!get_loaned_interfaces(
        command_interfaces_,
        required_command_interfaces_name,
        joint_command_interfaces_
    )) {
        RCLCPP_ERROR(get_node()->get_logger(), "All the requested command interface is not found");
        return controller_interface::CallbackReturn::ERROR;
    }

    // Acquiring state interfaces
    std::vector<std::string> required_state_interfaces_name {
        params_.left_lift_actuator_name + "/" + "position",
        params_.right_lift_actuator_name + "/" + "position",
        params_.left_encoder_sensor_name + "/" + "position",
        params_.right_encoder_sensor_name + "/" + "position",
        params_.belt_actuator_name + "/" + "velocity"
    };
    if(!get_loaned_interfaces(
        state_interfaces_,
        required_state_interfaces_name,
        joint_state_interfaces_
    )) {
        RCLCPP_ERROR(get_node()->get_logger(), "All the requested state interface is not found");
        return controller_interface::CallbackReturn::ERROR;
    }

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