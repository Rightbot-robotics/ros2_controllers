#include <dynamic_conveyor_lift_tuner_controller/dynamic_conveyor_lift_tuner_controller.hpp>


namespace dynamic_conveyor_lift_tuner_controller
{

DynamicConveyorLiftTunerController::DynamicConveyorLiftTunerController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DynamicConveyorLiftTunerController::on_init() {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::on_init()");
    position_command_left_ = std::nan("");
    position_command_right_ = std::nan("");
    position_command_all_ = std::nan("");
    prev_position_kp_command_ = std::nan("");
    prev_position_ki_command_ = std::nan("");
    prev_position_kd_command_ = std::nan("");
    prev_velocity_kp_command_ = std::nan("");
    prev_velocity_ki_command_ = std::nan("");
    prev_velocity_kd_command_ = std::nan("");
    stop_command_all_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DynamicConveyorLiftTunerController::command_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::command_interface_configuration()");
    std::vector<std::string> conf_names;
    conf_names.push_back("conveyor_lift_left/position");
    conf_names.push_back("conveyor_lift_left/control_state");
    conf_names.push_back("conveyor_lift_left/position_kp");
    conf_names.push_back("conveyor_lift_left/position_ki");
    conf_names.push_back("conveyor_lift_left/position_kd");
    conf_names.push_back("conveyor_lift_left/velocity_kp");
    conf_names.push_back("conveyor_lift_left/velocity_ki");
    conf_names.push_back("conveyor_lift_left/velocity_kd");

    
    conf_names.push_back("conveyor_lift_right/position");
    conf_names.push_back("conveyor_lift_right/control_state");
    conf_names.push_back("conveyor_lift_right/position_kp");
    conf_names.push_back("conveyor_lift_right/position_ki");
    conf_names.push_back("conveyor_lift_right/position_kd");
    conf_names.push_back("conveyor_lift_right/velocity_kp");
    conf_names.push_back("conveyor_lift_right/velocity_ki");
    conf_names.push_back("conveyor_lift_right/velocity_kd");

    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration DynamicConveyorLiftTunerController::state_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::state_interface_configuration()");
    std::vector<std::string> conf_names;
    conf_names.push_back("conveyor_lift_left/position");
    // conf_names.push_back("conveyor_lift_left/position_kp");
    // conf_names.push_back("conveyor_lift_left/position_ki");
    // conf_names.push_back("conveyor_lift_left/position_kd");
    // conf_names.push_back("conveyor_lift_left/velocity_kp");
    // conf_names.push_back("conveyor_lift_left/velocity_ki");
    // conf_names.push_back("conveyor_lift_left/velocity_kd");

    conf_names.push_back("conveyor_lift_right/position");
    // conf_names.push_back("conveyor_lift_right/position_kp");
    // conf_names.push_back("conveyor_lift_right/position_ki");
    // conf_names.push_back("conveyor_lift_right/position_kd");
    // conf_names.push_back("conveyor_lift_right/velocity_kp");
    // conf_names.push_back("conveyor_lift_right/velocity_ki");
    // conf_names.push_back("conveyor_lift_right/velocity_kd");
    
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type DynamicConveyorLiftTunerController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    // RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorLiftTunerController::update()");
    if (stop_command_all_) {
        joint_command_interfaces_.at("conveyor_lift_right/control_state").get().set_value(2.0);
        joint_command_interfaces_.at("conveyor_lift_left/control_state").get().set_value(2.0);
    }

    if (position_command_left_ != prev_position_command_left_) {
        joint_command_interfaces_.at("conveyor_lift_left/position").get().set_value(position_command_left_);
    }

    if (position_command_right_ != prev_position_command_right_) {
        joint_command_interfaces_.at("conveyor_lift_right/position").get().set_value(position_command_right_);
    }
    if (position_kp_command_ != prev_position_kp_command_) {
        joint_command_interfaces_.at("conveyor_lift_right/position_kp").get().set_value(position_kp_command_);
        joint_command_interfaces_.at("conveyor_lift_left/position_kp").get().set_value(position_kp_command_);
    }

    if (position_ki_command_ != prev_position_ki_command_) {
        joint_command_interfaces_.at("conveyor_lift_right/position_ki").get().set_value(position_ki_command_);
        joint_command_interfaces_.at("conveyor_lift_left/position_ki").get().set_value(position_ki_command_);
    }

    if (position_kd_command_ != prev_position_kd_command_) {
        joint_command_interfaces_.at("conveyor_lift_right/position_kd").get().set_value(position_kd_command_);
        joint_command_interfaces_.at("conveyor_lift_left/position_kd").get().set_value(position_kd_command_);
    }

    if (velocity_kp_command_ != prev_velocity_kp_command_) {
        joint_command_interfaces_.at("conveyor_lift_right/velocity_kp").get().set_value(velocity_kp_command_);
        joint_command_interfaces_.at("conveyor_lift_left/velocity_kp").get().set_value(velocity_kp_command_);
    }

    if (velocity_ki_command_ != prev_velocity_ki_command_) {
        joint_command_interfaces_.at("conveyor_lift_right/velocity_ki").get().set_value(velocity_ki_command_);
        joint_command_interfaces_.at("conveyor_lift_left/velocity_ki").get().set_value(velocity_ki_command_);
    }

    if (velocity_kd_command_ != prev_velocity_kd_command_) {
        joint_command_interfaces_.at("conveyor_lift_right/velocity_kd").get().set_value(velocity_kd_command_);
        joint_command_interfaces_.at("conveyor_lift_left/velocity_kd").get().set_value(velocity_kd_command_);
    }

    if (position_command_all_ != prev_position_command_all_) {
        joint_command_interfaces_.at("conveyor_lift_right/position").get().set_value(position_command_all_);
        joint_command_interfaces_.at("conveyor_lift_left/position").get().set_value(position_command_all_);
    }
    
    prev_position_command_left_ = position_command_left_;
    prev_position_command_right_ = position_command_right_;
    prev_position_command_all_ = position_command_all_;

    prev_position_kp_command_ = position_kp_command_;
    prev_position_ki_command_ = position_ki_command_;
    prev_position_kd_command_ = position_kd_command_;

    prev_velocity_kp_command_ = velocity_kp_command_;
    prev_velocity_ki_command_ = velocity_ki_command_;
    prev_velocity_kd_command_ = velocity_kd_command_;

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
        "conveyor_lift_left/position_kp",
        "conveyor_lift_left/position_ki",
        "conveyor_lift_left/position_kd",
        "conveyor_lift_left/velocity_kp",
        "conveyor_lift_left/velocity_ki",
        "conveyor_lift_left/velocity_kd",


        "conveyor_lift_right/position",
        "conveyor_lift_right/control_state",
        "conveyor_lift_right/position_kp",
        "conveyor_lift_right/position_ki",
        "conveyor_lift_right/position_kd",
        "conveyor_lift_right/velocity_kp",
        "conveyor_lift_right/velocity_ki",
        "conveyor_lift_right/velocity_kd"

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
        // "conveyor_lift_left/position_kp",
        // "conveyor_lift_left/position_ki",
        // "conveyor_lift_left/position_kd",
        // "conveyor_lift_left/velocity_kp",
        // "conveyor_lift_left/velocity_ki",
        // "conveyor_lift_left/velocity_kd",

        "conveyor_lift_right/position",
        // "conveyor_lift_right/position_kp",
        // "conveyor_lift_right/position_ki",
        // "conveyor_lift_right/position_kd",
        // "conveyor_lift_right/velocity_kp",
        // "conveyor_lift_right/velocity_ki",
        // "conveyor_lift_right/velocity_kd"
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
                
            case rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::SET_POSITION: {
                position_command_all_ = req->command_value[0];
                resp->status = true;
                break;
            }
                
            case rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::SET_POSITION_PID_GAINS: {
                position_kp_command_ = req->command_value[0];
                position_ki_command_ = req->command_value[1];
                position_kd_command_ = req->command_value[2];
                resp->status = true;
                break;
            }
            
            case rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::SET_VELOCITY_PID_GAINS: {
                velocity_kp_command_ = req->command_value[0];
                velocity_ki_command_ = req->command_value[1];
                velocity_kd_command_ = req->command_value[2];
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