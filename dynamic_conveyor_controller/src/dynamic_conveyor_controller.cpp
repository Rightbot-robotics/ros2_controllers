#include <dynamic_conveyor_controller/dynamic_conveyor_controller.hpp>


namespace dynamic_conveyor_controller
{

DynamicConveyorController::DynamicConveyorController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn DynamicConveyorController::on_init() {
    RCLCPP_INFO(get_node()->get_logger(), "DynamicConveyorController::on_init()");
    on_first_update_loop_ = true;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DynamicConveyorController::command_interface_configuration() const {
    std::vector<std::string> conf_names;
    conf_names.push_back(params_.left_lift_actuator_name + "/" + "position");
    conf_names.push_back(params_.right_lift_actuator_name + "/" + "position");
    conf_names.push_back(params_.left_lift_actuator_name + "/" + "control_state");
    conf_names.push_back(params_.right_lift_actuator_name + "/" + "control_state");
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

controller_interface::return_type DynamicConveyorController::update(const rclcpp::Time & time, const rclcpp::Duration &) {

    if(command_available_) {
        command_acknowledged_ = true;
        command_available_ = false;
    }
    
    left_encoder_angle_ = joint_state_interfaces_.at(params_.left_encoder_sensor_name + "/" + "position").get().get_value();
    right_encoder_angle_ = joint_state_interfaces_.at(params_.right_encoder_sensor_name + "/" + "position").get().get_value();
    left_gantry_raw_distance_ = joint_state_interfaces_.at(params_.left_lift_actuator_name + "/" + "position").get().get_value();
    right_gantry_raw_distance_ = joint_state_interfaces_.at(params_.right_lift_actuator_name + "/" + "position").get().get_value();

    left_encoder_distance_ = enc_angle_to_distance(left_encoder_angle_);
    right_encoder_distance_ = enc_angle_to_distance(right_encoder_angle_);

    if(on_first_update_loop_) {
        left_gantry_initial_encoder_offset_ = left_encoder_distance_ - left_gantry_raw_distance_;
        right_gantry_initial_encoder_offset_ = right_encoder_distance_ - right_gantry_raw_distance_;
        RCLCPP_INFO(get_node()->get_logger(), "Initial encoder offsets: %f, %f", left_gantry_initial_encoder_offset_, right_gantry_initial_encoder_offset_);
        on_first_update_loop_ = false;
    }

    left_gantry_distance_ = left_gantry_raw_distance_ + left_gantry_initial_encoder_offset_;
    right_gantry_distance_ = right_gantry_raw_distance_ + right_gantry_initial_encoder_offset_;

    state_msg_.left_to_right_commanded_offset = left_minus_right_travel_offset_;
    state_msg_.left_to_right_actual_offset = (left_encoder_distance_ - right_encoder_distance_);
    state_msg_.left_encoder_distance = left_encoder_distance_;
    state_msg_.left_actuator_distance = left_gantry_distance_;
    state_msg_.right_encoder_distance = right_encoder_distance_;
    state_msg_.right_actuator_distance = right_gantry_distance_;

    state_pub_->publish(state_msg_);

    if(check_sanity_ && !sanity_check()) {
        if(!commanded_stop_){
            RCLCPP_ERROR(get_node()->get_logger(), "sanity check failed, commanding current position as target");
            joint_command_interfaces_.at(params_.left_lift_actuator_name + "/" + "control_state").get().set_value(2.0);
            joint_command_interfaces_.at(params_.right_lift_actuator_name + "/" + "control_state").get().set_value(2.0);
            commanded_stop_ = true;
        }
        {
            std::lock_guard lk(response_wait_mutex_);
            response_string_ = "SANITY_CHECK_FAILED";
        }
        response_wait_cv_.notify_one();
        relative_move_request_available_ = false;
        relative_move_request_available_ = false;
        executing_gantry_move_command_ = false;
        return controller_interface::return_type::OK;
    }

    if(realign_request_available_) {
        RCLCPP_INFO(get_node()->get_logger(), "realigning request available");
        realign_difference_ = right_gantry_distance_ - left_gantry_distance_ + left_minus_right_travel_offset_;
        RCLCPP_INFO(get_node()->get_logger(), "realigning difference: %f", realign_difference_);
        if(realign_left_) {
            left_gantry_target_distance_ = left_gantry_raw_distance_ + realign_difference_;
            right_gantry_target_distance_ = right_gantry_raw_distance_;
        }
        else if(realign_right_) {
            right_gantry_target_distance_ = right_gantry_raw_distance_ - realign_difference_;
            left_gantry_target_distance_ = left_gantry_raw_distance_;
        }
        RCLCPP_INFO(get_node()->get_logger(), "realigning target distances: L:%f, R:%f", left_gantry_target_distance_, right_gantry_target_distance_);
        realign_request_available_ = false;
        gantry_move_request_available_ = true;
    }

    if(relative_move_request_available_) {
        RCLCPP_INFO(get_node()->get_logger(), "relative move request available");
        left_gantry_target_distance_ = left_gantry_distance_ - left_gantry_initial_encoder_offset_ + relative_move_distance_;
        right_gantry_target_distance_ = right_gantry_distance_ - right_gantry_initial_encoder_offset_ + relative_move_distance_;
        RCLCPP_INFO(get_node()->get_logger(), "relative move target distances: L:%f, R:%f", left_gantry_target_distance_, right_gantry_target_distance_);
        relative_move_request_available_ = false;
        gantry_move_request_available_ = true;
    }

    if(gantry_move_request_available_) {
        RCLCPP_INFO(get_node()->get_logger(), "gantry target distance: L:%f, R:%f", left_gantry_target_distance_, right_gantry_target_distance_);
        joint_command_interfaces_.at(params_.left_lift_actuator_name + "/" + "position").get().set_value(left_gantry_target_distance_);
        joint_command_interfaces_.at(params_.right_lift_actuator_name + "/" + "position").get().set_value(right_gantry_target_distance_);
        lift_command_sent_time_ = time;
        gantry_move_request_available_ = false;
        executing_gantry_move_command_ = true;
        commanded_stop_ = false;
    }
    if(executing_gantry_move_command_) {
        left_gantry_target_diff_ = left_gantry_target_distance_ - left_gantry_raw_distance_;
        right_gantry_target_diff_ = right_gantry_target_distance_ - right_gantry_raw_distance_;
        RCLCPP_INFO(get_node()->get_logger(), "gantry target diff: L:%f, R:%f", left_gantry_target_diff_, right_gantry_target_diff_);
        if(
            std::abs(left_gantry_target_diff_) < params_.gantry_target_distance_tolerance &&
            std::abs(right_gantry_target_diff_) < params_.gantry_target_distance_tolerance
        ) {
            executing_gantry_move_command_ = false;
            RCLCPP_INFO(get_node()->get_logger(), "gantry target achieved");
            {
                std::lock_guard lk(response_wait_mutex_);
                response_string_ = "SUCCESS";
            }
            response_wait_cv_.notify_one();
        }
        else if (time - lift_command_sent_time_ > gantry_target_timeout_) {
            executing_gantry_move_command_ = false;
            RCLCPP_ERROR(get_node()->get_logger(), "gantry target timeout");
            {
                std::lock_guard lk(response_wait_mutex_);
                response_string_ = "TIMEOUT";
            }
            response_wait_cv_.notify_one();
        }
    }

    if(belt_velocity_request_available_) {
        joint_command_interfaces_.at(params_.belt_actuator_name + "/" + "velocity").get().set_value(target_belt_velocity_);
        belt_command_sent_time_ = time;
        belt_velocity_window_enter_time_ = time;
        belt_velocity_request_available_ = false;
        executing_belt_velocity_command_ = true;
        previously_in_belt_velocity_window_ = true;
    }
    if(executing_belt_velocity_command_) {
        current_belt_velocity_ = joint_state_interfaces_.at(params_.belt_actuator_name + "/" + "velocity").get().get_value();
        if(std::abs(target_belt_velocity_ - current_belt_velocity_) < params_.belt_target_velocity_tolerance) {
            if(!previously_in_belt_velocity_window_) {
                belt_velocity_window_enter_time_ = time;
                previously_in_belt_velocity_window_ = true;
            }
            else {
                if(time - belt_velocity_window_enter_time_ > belt_target_velocity_window_time_) {
                    executing_belt_velocity_command_ = false;
                    RCLCPP_INFO(get_node()->get_logger(), "belt target achieved");
                    {
                        std::lock_guard lk(response_wait_mutex_);
                        response_string_ = "SUCCESS";
                    }
                    response_wait_cv_.notify_one();
                }
            }
        }
        else {
            previously_in_belt_velocity_window_ = false;
            if(time - belt_command_sent_time_ > belt_command_timeout_) {
                executing_belt_velocity_command_ = false;
                RCLCPP_ERROR(get_node()->get_logger(), "belt target timeout");
                {
                    std::lock_guard lk(response_wait_mutex_);
                    response_string_ = "TIMEOUT";
                }
                response_wait_cv_.notify_one();
            }
        }
    }

    return controller_interface::return_type::OK;
}

bool DynamicConveyorController::sanity_check() {
    is_sane_ = true;
    if(std::abs(left_encoder_distance_ - left_gantry_distance_) > params_.enc_to_gantry_sanity_tolerance) {
        is_sane_ = false;
        if(was_sane_)
            RCLCPP_ERROR(get_node()->get_logger(), "left encoder-gantry value mismatch: %f, %f", left_encoder_distance_, left_gantry_distance_);
    }
    if(std::abs(right_encoder_distance_ - right_gantry_distance_) > params_.enc_to_gantry_sanity_tolerance) {
        is_sane_ = false;
        if(was_sane_)
            RCLCPP_ERROR(get_node()->get_logger(), "right encoder-gantry value mismatch: %f, %f", right_encoder_distance_, right_gantry_distance_);
    }
    if(std::abs(left_encoder_distance_ - right_encoder_distance_ - left_minus_right_travel_offset_) > params_.enc_to_enc_sanity_tolerance) {
        is_sane_ = false;
        if(was_sane_)
            RCLCPP_ERROR(get_node()->get_logger(), "left_encoder-right_encoder value mismatch: %f, %f", left_encoder_distance_, right_encoder_distance_);
    }
    was_sane_ = is_sane_;
    return is_sane_;
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
    gantry_target_timeout_ = std::chrono::duration<double>(params_.gantry_target_timeout);
    belt_target_velocity_window_time_ = std::chrono::duration<double>(params_.belt_target_velocity_window_time_ms / 1000.0);
    belt_command_timeout_ = std::chrono::duration<double>(params_.belt_target_timeout);
    left_minus_right_travel_offset_ = params_.left_minus_right_travel_offset;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DynamicConveyorController::on_activate(
    const rclcpp_lifecycle::State &) {
    
    // Acquiring command interfaces
    std::vector<std::string> required_command_interfaces_name {
        params_.left_lift_actuator_name + "/" + "position",
        params_.right_lift_actuator_name + "/" + "position",
        params_.left_lift_actuator_name + "/" + "control_state",
        params_.right_lift_actuator_name + "/" + "control_state",
        params_.belt_actuator_name + "/" + "velocity"
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

    // Creating services
    conveyor_commad_srv_ = get_node()->create_service<rightbot_interfaces::srv::ConveyorCommand>(
        "conveyor_command",
        std::bind(
            &DynamicConveyorController::conveyor_command_service_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    // State publisher
    state_pub_ = get_node()->create_publisher<rightbot_interfaces::msg::ConveyorState>("conveyor_state", rclcpp::SystemDefaultsQoS());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DynamicConveyorController::on_deactivate(
    const rclcpp_lifecycle::State &) {
    conveyor_commad_srv_.reset();
    state_pub_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
}

void DynamicConveyorController::conveyor_command_service_callback(
    rightbot_interfaces::srv::ConveyorCommand::Request::SharedPtr req,
    rightbot_interfaces::srv::ConveyorCommand::Response::SharedPtr resp) {

    using namespace rightbot_interfaces::srv;

    uint32_t system_status_cp = 0;

    switch(req->command_type) {
        case ConveyorCommand::Request::SET_HEIGHT: {
            RCLCPP_INFO(get_node()->get_logger(), "Height command received: %f", req->command_value);
            double gantry_travel_distance = get_equation_result(req->command_value, height_cmd_inverse_kinematics_coeffs_);
            left_gantry_target_distance_ = gantry_travel_distance - left_gantry_initial_encoder_offset_;
            right_gantry_target_distance_ = gantry_travel_distance - right_gantry_initial_encoder_offset_ - left_minus_right_travel_offset_;
            gantry_move_request_available_ = true;
            check_sanity_ = true;
            break;
        }
        case ConveyorCommand::Request::SET_ANGLE: {
            RCLCPP_INFO(get_node()->get_logger(), "Angle command received: %f", req->command_value);
            double gantry_travel_distance = get_equation_result(req->command_value, angle_cmd_inverse_kinematics_coeffs_);
            left_gantry_target_distance_ = gantry_travel_distance - left_gantry_initial_encoder_offset_;
            right_gantry_target_distance_ = gantry_travel_distance - right_gantry_initial_encoder_offset_ - left_minus_right_travel_offset_;
            gantry_move_request_available_ = true;
            check_sanity_ = true;
            break;
        }
        case ConveyorCommand::Request::SET_VELOCITY: {
            RCLCPP_INFO(get_node()->get_logger(), "Velocity command received: %f", req->command_value);
            target_belt_velocity_ = req->command_value;
            last_commanded_belt_velocity_ = target_belt_velocity_;
            if((system_status_cp & SystemStatus::BELT_RUNNING) != 0) {
                belt_velocity_request_available_ = true;
            }
            else {
                resp->status = "SUCCESS";
                return;
            }
            break;
        }
        case ConveyorCommand::Request::START_BELT: {
            RCLCPP_INFO(get_node()->get_logger(), "Start belt command received");
            target_belt_velocity_ = last_commanded_belt_velocity_;
            belt_velocity_request_available_ = true;
            break;
        }
        case ConveyorCommand::Request::STOP_BELT: {
            RCLCPP_INFO(get_node()->get_logger(), "Stop belt command received");
            target_belt_velocity_ = 0.0;
            belt_velocity_request_available_ = true;
            break;
        }
        case ConveyorCommand::Request::REALIGN_LEFT: {
            RCLCPP_INFO(get_node()->get_logger(), "Realign left command received");
            realign_left_ = true;
            realign_right_ = false;
            realign_request_available_ = true;
            check_sanity_ = false;
            break;
        }
        case ConveyorCommand::Request::REALIGN_RIGHT: {
            RCLCPP_INFO(get_node()->get_logger(), "Realign right command received");
            realign_left_ = false;
            realign_right_ = true;
            realign_request_available_ = true;
            check_sanity_ = false;
            break;
        }
        case ConveyorCommand::Request::SET_OFFSET: {
            RCLCPP_INFO(get_node()->get_logger(), "Offset command received: %f", req->command_value);
            left_minus_right_travel_offset_ = req->command_value;
            resp->status = "SUCCESS";
            check_sanity_ = true;
            return;
        }
        case ConveyorCommand::Request::MOVE_GANTRY_RELATIVE: {
            RCLCPP_INFO(get_node()->get_logger(), "Move gantry relative command received: %f", req->command_value);
            relative_move_distance_ = req->command_value;
            relative_move_request_available_ = true;
            check_sanity_ = true;
            break;
        }
        default:
            resp->status = "INVALID_COMMAND_TYPE";
            return;
    }

    wait_until_command_acknowledged();

    {
        std::unique_lock lk(response_wait_mutex_);
        response_string_ = "";
        while(response_string_ == "") {
            response_wait_cv_.wait_for(lk, std::chrono::milliseconds(100), [this]() { return response_string_ != ""; });
        }
        resp->status = response_string_;
        response_string_ = "";
    }
    check_sanity_ = true;
}

double DynamicConveyorController::get_equation_result(double x, std::vector<double> coeffs) {
    double result = 0;
    int degree = (int)coeffs.size();
    for(int i = 0; i < degree; i++) {
        result += coeffs[i] * pow(x, (degree-1-i));
    }
    return result;
}

double DynamicConveyorController::enc_angle_to_distance(double angle) {
    return angle * params_.enc_to_dist_multiplication_factor + params_.enc_to_dist_offset_factor;
}

bool DynamicConveyorController::wait_until_command_acknowledged() {
    command_available_ = true;
    command_acknowledged_ = false;
    while(!command_acknowledged_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

}  // namespace dynamic_conveyor_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamic_conveyor_controller::DynamicConveyorController, controller_interface::ControllerInterface)