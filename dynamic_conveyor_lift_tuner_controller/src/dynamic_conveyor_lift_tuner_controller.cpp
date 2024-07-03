#include <dynamic_conveyor_lift_tuner_controller/dynamic_conveyor_lift_tuner_controller.hpp>


namespace dynamic_conveyor_lift_tuner_controller
{

DynamicConveyorLiftTunerController::DynamicConveyorLiftTunerController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DynamicConveyorLiftTunerController::on_init() {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::on_init()");
    position_command_left_ = std::nan("");
    position_command_right_ = std::nan("");
    position_command_all_ = std::nan("");
    kp_command_left_ = std::nan("");
    kp_command_right_ = std::nan("");
    ki_command_left_ = std::nan("");
    ki_command_right_ = std::nan("");
    kd_command_left_ = std::nan("");
    kd_command_right_ = std::nan("");
    stop_command_all_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DynamicConveyorLiftTunerController::command_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::command_interface_configuration()");
    std::vector<std::string> conf_names;
    conf_names.push_back("conveyor_lift_left/position");
    conf_names.push_back("conveyor_lift_left/control_state");
    conf_names.push_back("conveyor_lift_left/kp");
    conf_names.push_back("conveyor_lift_left/ki");
    conf_names.push_back("conveyor_lift_left/kd");
    
    conf_names.push_back("conveyor_lift_right/position");
    conf_names.push_back("conveyor_lift_right/control_state");
    conf_names.push_back("conveyor_lift_right/kp");
    conf_names.push_back("conveyor_lift_right/ki");
    conf_names.push_back("conveyor_lift_right/kd");

    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration DynamicConveyorLiftTunerController::state_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::state_interface_configuration()");
    std::vector<std::string> conf_names;
    conf_names.push_back("conveyor_lift_left/position");
    conf_names.push_back("conveyor_lift_left/kp");
    conf_names.push_back("conveyor_lift_left/ki");
    conf_names.push_back("conveyor_lift_left/kd");

    conf_names.push_back("conveyor_lift_right/position");
    conf_names.push_back("conveyor_lift_right/kp");
    conf_names.push_back("conveyor_lift_right/ki");
    conf_names.push_back("conveyor_lift_right/kd");
    
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type DynamicConveyorLiftTunerController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    // RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::update()");
    if (stop_command_all_) {
        joint_command_interfaces_.at("conveyor_lift_right/control_state").get().set_value(2.0);
        joint_command_interfaces_.at("conveyor_lift_left/control_state").get().set_value(2.0);
    }

    if (position_command_left_ != std::nan("") || position_command_right_ != std::nan("") || position_command_all_ != std::nan("")) {
        joint_command_interfaces_.at("conveyor_lift_right/position").get().set_value(position_command_right_);
        joint_command_interfaces_.at("conveyor_lift_left/position").get().set_value(position_command_left_);
    }

    if (kp_command_left_ != std::nan("") || kp_command_right_ != std::nan("")) {
        joint_command_interfaces_.at("conveyor_lift_right/kp").get().set_value(kp_command_right_);
        joint_command_interfaces_.at("conveyor_lift_left/kp").get().set_value(kp_command_left_);
    }

    if (ki_command_left_ != std::nan("") || ki_command_right_ != std::nan("")) {
        joint_command_interfaces_.at("conveyor_lift_right/ki").get().set_value(ki_command_right_);
        joint_command_interfaces_.at("conveyor_lift_left/ki").get().set_value(ki_command_left_);
    }

    if (kd_command_left_ != std::nan("") || kd_command_right_ != std::nan("")) {
        joint_command_interfaces_.at("conveyor_lift_right/kd").get().set_value(kd_command_right_);
        joint_command_interfaces_.at("conveyor_lift_left/kd").get().set_value(kd_command_left_);
    }

    if (position_command_all_ != std::nan("")) {
        joint_command_interfaces_.at("conveyor_lift_right/position").get().set_value(position_command_all_);
        joint_command_interfaces_.at("conveyor_lift_left/position").get().set_value(position_command_all_);
    }
    
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DynamicConveyorLiftTunerController::on_configure(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::on_configure()");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DynamicConveyorLiftTunerController::on_activate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::on_activate()");
    
    // Acquiring command interfaces
    std::vector<std::string> required_command_interfaces_name {
        "conveyor_lift_left/position",
        "conveyor_lift_left/control_state",
        "conveyor_lift_left/kp",
        "conveyor_lift_left/ki",
        "conveyor_lift_left/kd",

        "conveyor_lift_right/position",
        "conveyor_lift_right/control_state",
        "conveyor_lift_right/kp",
        "conveyor_lift_right/ki",
        "conveyor_lift_right/kd"
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
        "conveyor_lift_left/position",
        "conveyor_lift_left/kp",
        "conveyor_lift_left/ki",
        "conveyor_lift_left/kd",

        "conveyor_lift_right/position",
        "conveyor_lift_right/kp",
        "conveyor_lift_right/ki",
        "conveyor_lift_right/kd"
    };

    if(!get_loaned_interfaces(
        state_interfaces_,
        required_state_interfaces_name,
        joint_state_interfaces_
    )) {
        RCLCPP_ERROR(get_node()->get_logger(), "All the requested state interface is not found");
        return controller_interface::CallbackReturn::ERROR;
    }
    
    // Creating services
    conveyor_lift_tuner_commad_srv_ = get_node()->create_service<rightbot_interfaces::srv::ConveyorLiftTunerCommand>(
        "conveyor_lift_tuner_command",
        std::bind(
            &DynamicConveyorLiftTunerController::conveyor_lift_tuner_command_service_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DynamicConveyorLiftTunerController::on_deactivate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::on_deactivate()");
    conveyor_lift_tuner_commad_srv_.reset();
    joint_command_interfaces_.clear();
    joint_state_interfaces_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}

void DynamicConveyorLiftTunerController::conveyor_lift_tuner_command_service_callback(
    rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::SharedPtr req,
    rightbot_interfaces::srv::ConveyorLiftTunerCommand::Response::SharedPtr resp) {

        switch(req->command_type) {
            case rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::SET_POSITION_LEFT: {
                position_command_left_ = req->command_value[0];
                resp->status = true;   
                break;
            }
                
            case rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::SET_POSITION_RIGHT: {
                position_command_right_ = req->command_value[0];
                resp->status = true;
                break;
            }
                2
            case rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::SET_POSITION: {
                position_command_all_ = req->command_value[0];
                resp->status = true;
                break;
            }
                
            case rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::SET_PID_GAINS: {
                kp_command_left_ = req->command_value[0];
                kp_command_right_ = req->command_value[0];
                ki_command_left_ = req->command_value[1];
                ki_command_right_ = req->command_value[1];
                kd_command_left_ = req->command_value[2];
                kd_command_right_ = req->command_value[2];
                resp->status = true;
                break;
            }
            
            case rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::EMEREGNCY_STOP: {
                stop_command_all_ = true;
                resp->status = true;
                break;
            }
            default:
                resp->status = false;
                return;
        }
    }
}  // namespace dynamic_conveyor_lift_tuner_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  dynamic_conveyor_lift_tuner_controller::DynamicConveyorLiftTunerController, controller_interface::ControllerInterface)