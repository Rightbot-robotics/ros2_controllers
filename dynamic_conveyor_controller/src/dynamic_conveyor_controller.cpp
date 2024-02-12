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

void DynamicConveyorController::conveyor_command_service_callback(
    rightbot_interfaces::srv::ConveyorCommand::Request::SharedPtr req,
    rightbot_interfaces::srv::ConveyorCommand::Response::SharedPtr resp) {

    using namespace rightbot_interfaces::srv;

    uint32_t system_status_cp = 0;

    switch(req->command_type) {
        case ConveyorCommand::Request::SET_HEIGHT: {
            double gantry_travel_distance = get_equation_result(req->command_value, height_cmd_inverse_kinematics_coeffs_);
            left_gantry_target_distance_ = gantry_travel_distance - left_gantry_initial_offset_;
            right_gantry_target_distance_ = gantry_travel_distance - right_gantry_initial_offset_;
            lift_request_available_ = true;
            break;
        }
        case ConveyorCommand::Request::SET_ANGLE: {
            double gantry_travel_distance = get_equation_result(req->command_value, angle_cmd_inverse_kinematics_coeffs_);
            left_gantry_target_distance_ = gantry_travel_distance - left_gantry_initial_offset_;
            right_gantry_target_distance_ = gantry_travel_distance - right_gantry_initial_offset_;
            lift_request_available_ = true;
            break;
        }
        case ConveyorCommand::Request::SET_VELOCITY: {
            target_belt_velocity_ = req->command_value;
            last_commanded_belt_velocity_ = target_belt_velocity_;
            if(system_status_cp & SystemStatus::BELT_RUNNING) {
                belt_velocity_request_available_ = true;
            }
            break;
        }
        case ConveyorCommand::Request::START_BELT: {
            target_belt_velocity_ = last_commanded_belt_velocity_;
            belt_velocity_request_available_ = true;
            break;
        }
        case ConveyorCommand::Request::STOP_BELT: {
            target_belt_velocity_ = 0.0;
            belt_velocity_request_available_ = true;
            break;
        }
        case ConveyorCommand::Request::REALIGN_LEFT: {
            realign_left_ = true;
            realign_right_ = false;
            realign_request_available_ = true;
            break;
        }
        case ConveyorCommand::Request::REALIGN_RIGHT: {
            realign_left_ = false;
            realign_right_ = true;
            realign_request_available_ = true;
            break;
        }
        default:
            resp->status = "INVALID_COMMAND_TYPE";
            return;
    }

    {
        std::unique_lock lk(response_wait_mutex_);
        response_wait_cv_.wait(lk, [this]() { return response_string_ != ""; });
        resp->status = response_string_;
        response_string_ = "";
    }
}

double DynamicConveyorController::get_equation_result(double x, std::vector<double> coeffs) {
    double result = 0;
    int degree = (int)coeffs.size();
    for(int i = 0; i < degree; i++) {
        result += coeffs[i] * pow(x, (degree-1-i));
    }
    return result;
}

}  // namespace dynamic_conveyor_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamic_conveyor_controller::DynamicConveyorController, controller_interface::ControllerInterface)