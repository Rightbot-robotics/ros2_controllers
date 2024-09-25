#include <swerve_drive_controller/swerve_drive_controller.hpp>


namespace swerve_drive_controller
{

SwerveDriveController::SwerveDriveController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn SwerveDriveController::on_init() {
    RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::on_init()");
    try
    {
        // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    base_frame_id_ = params_.drive_frame_id;
    odom_frame_id_ = params_.odom_frame_id;
    max_loop_period_ = std::chrono::duration<double>(params_.max_loop_period);

    num_modules_ = params_.swerve_modules_name.size();
    std::vector<std::pair<double, double>> module_positions;
    for(auto module_name : params_.swerve_modules_name) {
        double module_x_pos = params_.modules_info.swerve_modules_name_map.at(module_name).wheel_x;
        double module_y_pos = params_.modules_info.swerve_modules_name_map.at(module_name).wheel_y;
        module_positions.push_back({module_x_pos, module_y_pos});
    }

    kinematics_ = std::make_shared<SwerveDriveKinematics>(module_positions);
    odom_processor_ = std::make_shared<OdometryProcessor>(base_frame_id_, odom_frame_id_);

    cmd_vel_expiry_time_ = std::chrono::system_clock::now();
    cmd_velocity_ = Velocity();
    cmd_velocity_cp_ = Velocity();
    target_velocity_ = Velocity();
    prev_target_velocity_ = Velocity();
    curr_steer_wheel_angle_ = std::vector<double>(num_modules_, 0.0);
    curr_drive_wheel_angle_ = std::vector<double>(num_modules_, 0.0);
    curr_drive_wheel_vel_ = std::vector<double>(num_modules_, 0.0);
    cmd_vel_expired_ = true;
    base_at_zero_vel_ = true;

    large_angle_diff_handling_phase_ = "none";

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveDriveController::command_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::command_interface_configuration()");
    std::vector<std::string> conf_names;
    for (std::string module_name : params_.swerve_modules_name) {
        std::string steer_joint = params_.modules_info.swerve_modules_name_map.at(module_name).steer_joint_name;
        std::string drive_joint = params_.modules_info.swerve_modules_name_map.at(module_name).drive_joint_name;
        conf_names.push_back(steer_joint + "/position");
        conf_names.push_back(drive_joint + "/velocity");
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::state_interface_configuration()");
    std::vector<std::string> conf_names;
    for (std::string module_name : params_.swerve_modules_name) {
        std::string steer_joint = params_.modules_info.swerve_modules_name_map.at(module_name).steer_joint_name;
        std::string drive_joint = params_.modules_info.swerve_modules_name_map.at(module_name).drive_joint_name;
        conf_names.push_back(steer_joint + "/position");
        conf_names.push_back(drive_joint + "/position");
        conf_names.push_back(drive_joint + "/velocity");
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn SwerveDriveController::on_configure(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::on_configure()");
    for (std::string module_name : params_.swerve_modules_name) {
        std::string steer_joint = params_.modules_info.swerve_modules_name_map.at(module_name).steer_joint_name;
        std::string drive_joint = params_.modules_info.swerve_modules_name_map.at(module_name).drive_joint_name;

        required_command_interfaces_.push_back(steer_joint + "/position");
        required_command_interfaces_.push_back(drive_joint + "/velocity");
        
        required_state_interfaces_.push_back(steer_joint + "/position");
        required_state_interfaces_.push_back(drive_joint + "/position");
        required_state_interfaces_.push_back(drive_joint + "/velocity");
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_activate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::on_activate()");

    if(!get_loaned_interfaces(
        command_interfaces_,
        required_command_interfaces_,
        loaned_command_interfaces_
    )) {
        RCLCPP_ERROR(get_node()->get_logger(), "All the requested command interface is not found");
        return controller_interface::CallbackReturn::ERROR;
    }
    if(!get_loaned_interfaces(
        state_interfaces_,
        required_state_interfaces_,
        loaned_state_interfaces_
    )) {
        RCLCPP_ERROR(get_node()->get_logger(), "All the requested state interface is not found");
        return controller_interface::CallbackReturn::ERROR;
    }

    update_loop_first_pass_ = true;

    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        params_.cmd_vel_topic,
        10,
        std::bind(&SwerveDriveController::cmd_vel_callback, this, std::placeholders::_1)
    );

    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        params_.odom_topic,
        rclcpp::SystemDefaultsQoS()
    );

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::on_deactivate()");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveDriveController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    if(update_loop_first_pass_) {
        RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::update()");
        prev_loop_time_ = std::chrono::system_clock::now();
    }

    curr_loop_time_ = std::chrono::system_clock::now();
    loop_period_ = std::chrono::duration<double>(curr_loop_time_ - prev_loop_time_);
    if(loop_period_ > max_loop_period_) {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "Loop period is too high!!!. Loop period: %f",
            loop_period_.count()
        );
        loop_period_ = max_loop_period_;
    }

    update_base_state_variables();
    handle_odometry();
    handle_cmd_vel();

    update_loop_first_pass_ = false;
    prev_loop_time_ = curr_loop_time_;
    return controller_interface::return_type::OK;
}

void SwerveDriveController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
        cmd_velocity_.set_speed(*msg);
        cmd_vel_expiry_time_ = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
            std::chrono::system_clock::now() + std::chrono::duration<double>(params_.cmd_vel_timeout)
        );
    }
}

void SwerveDriveController::update_base_state_variables() {
    int i = 0;
    for(std::string module_name : params_.swerve_modules_name) {
        curr_steer_wheel_angle_[i] = loaned_state_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).steer_joint_name + "/position").get().get_value();
        curr_drive_wheel_angle_[i] = loaned_state_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).drive_joint_name + "/position").get().get_value();
        curr_drive_wheel_vel_[i] = loaned_state_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).drive_joint_name + "/velocity").get().get_value();
        i++;
    }
}

void SwerveDriveController::handle_odometry() {
    if(update_loop_first_pass_) {
        prev_drive_wheel_angle_ = curr_drive_wheel_angle_;
        for(int i = 0; i < num_modules_; i++) {
            swerve_modules_pos_diff_.emplace_back(0, 0);
            swerve_modules_vel_.emplace_back(0, 0);
        }
        return;
    }
    
    for(int i = 0; i < num_modules_; i++) {
        double wheel_radius = params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).wheel_radius;
        double wheel_dist_diff = (curr_drive_wheel_angle_[i] - prev_drive_wheel_angle_[i]) * wheel_radius;
        swerve_modules_pos_diff_[i].first = wheel_dist_diff * std::cos(curr_steer_wheel_angle_[i]);
        swerve_modules_pos_diff_[i].second = wheel_dist_diff * std::sin(curr_steer_wheel_angle_[i]);
        swerve_modules_vel_[i].first = curr_drive_wheel_vel_[i] * std::cos(curr_steer_wheel_angle_[i]);
        swerve_modules_vel_[i].second = curr_drive_wheel_vel_[i] * std::sin(curr_steer_wheel_angle_[i]);
    }

    kinematics_->forward_kinematics(swerve_modules_pos_diff_, curr_base_pos_diff_);
    kinematics_->forward_kinematics(swerve_modules_vel_, curr_base_velocity_);

    odom_processor_->update_odometry(curr_base_pos_diff_, curr_base_velocity_);

    odom_pub_->publish(odom_processor_->odom_msg);
    if(params_.publish_tf) {
        tf_broadcaster_->sendTransform(odom_processor_->transform_msg);
    }

    if(curr_base_velocity_.is_zero()) {
        if(!base_at_zero_vel_) {
            RCLCPP_INFO(get_node()->get_logger(), "Drive reached at zero velocity");
        }
        base_at_zero_vel_ = true;
    }
    else {
        if(base_at_zero_vel_) {
            RCLCPP_INFO(get_node()->get_logger(), "Drive started moving");
        }
        base_at_zero_vel_ = false;
    }

    prev_drive_wheel_angle_ = curr_drive_wheel_angle_;
}

void SwerveDriveController::handle_cmd_vel() {
    if(update_loop_first_pass_) {
        for(int i = 0; i < num_modules_; i++) {
            swerve_modules_vel_cmd_.emplace_back(0, 0);
            steer_angle_cmd_.emplace_back(0);
            drive_wheel_vel_cmd_.emplace_back(0);
            prev_drive_wheel_vel_cmd_.emplace_back(0);
        }
        prev_steer_angle_cmd_ = curr_steer_wheel_angle_;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
        cmd_velocity_cp_.set_speed(
            cmd_velocity_.linear_x,
            cmd_velocity_.linear_y,
            cmd_velocity_.angular_z
        );
        cmd_vel_expiry_time_cp_ = cmd_vel_expiry_time_;
    }

    target_velocity_.set_speed(cmd_velocity_cp_);

    if(std::chrono::system_clock::now() > cmd_vel_expiry_time_cp_) {
        if(!cmd_vel_expired_) {
            RCLCPP_WARN(get_node()->get_logger(), "Command velocity timed out, setting target velocity to 0");
        }
        cmd_vel_expired_ = true;
        target_velocity_.set_speed(0.0, 0.0, 0.0);
    }
    else {
        if(cmd_vel_expired_) {
            RCLCPP_INFO(get_node()->get_logger(), "Started receiving new command velocity");
            RCLCPP_INFO(get_node()->get_logger(), "Target velocity: x-%f, y-%f, w-%f", target_velocity_.linear_x, target_velocity_.linear_y, target_velocity_.angular_z);
        }
        cmd_vel_expired_ = false;
    }

    apply_kinematics_limits(target_velocity_);
    prev_target_velocity_ = target_velocity_;

    kinematics_->inverse_kinematics(target_velocity_, swerve_modules_vel_cmd_);
    for(int i = 0; i < num_modules_; i++) {
        steer_angle_cmd_[i] = std::atan2(swerve_modules_vel_cmd_[i].second, swerve_modules_vel_cmd_[i].first);
        if(swerve_modules_vel_cmd_[i].first < 0 && std::abs(swerve_modules_vel_cmd_[i].second) < epsilon_) {
            if(curr_steer_wheel_angle_[i] < 0) {
                steer_angle_cmd_[i] = -1 * M_PI;
            }
            else {
                steer_angle_cmd_[i] = M_PI;
            }
        }
        double wheel_radius = params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).wheel_radius;
        drive_wheel_vel_cmd_[i] = std::hypot(swerve_modules_vel_cmd_[i].first, swerve_modules_vel_cmd_[i].second) / wheel_radius;
    }

    apply_joint_limits(steer_angle_cmd_, drive_wheel_vel_cmd_);


    {
        int i = 0;
        for(std::string module_name : params_.swerve_modules_name) {
            loaned_command_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).steer_joint_name + "/position").get().set_value(steer_angle_cmd_[i]);
            loaned_command_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).drive_joint_name + "/velocity").get().set_value(drive_wheel_vel_cmd_[i]);
            i++;
        }
    }
}

void SwerveDriveController::apply_kinematics_limits(Velocity& vel_in) {
    if(std::abs(vel_in.linear_x) > params_.max_linear_vel_x) {
        RCLCPP_WARN(get_node()->get_logger(), "Linear x velocity %f exceeds max linear velocity %f", vel_in.linear_x, params_.max_linear_vel_x);
        vel_in.linear_x = std::copysign(params_.max_linear_vel_x, vel_in.linear_x);
        RCLCPP_WARN(get_node()->get_logger(), "Linear x velocity limited to %f", vel_in.linear_x);
    }
    if(std::abs(vel_in.linear_y) > params_.max_linear_vel_y) {
        RCLCPP_WARN(get_node()->get_logger(), "Linear y velocity %f exceeds max linear velocity %f", vel_in.linear_y, params_.max_linear_vel_y);
        vel_in.linear_y = std::copysign(params_.max_linear_vel_y, vel_in.linear_y);
        RCLCPP_WARN(get_node()->get_logger(), "Linear y velocity limited to %f", vel_in.linear_y);
    }
    if(std::abs(vel_in.angular_z) > params_.max_angular_vel_z) {
        RCLCPP_WARN(get_node()->get_logger(), "Angular z velocity %f exceeds max angular velocity %f", vel_in.angular_z, params_.max_angular_vel_z);
        vel_in.angular_z = std::copysign(params_.max_angular_vel_z, vel_in.angular_z);
        RCLCPP_WARN(get_node()->get_logger(), "Angular z velocity limited to %f", vel_in.angular_z);
    }

    double commanded_acceleration;

    commanded_acceleration = (vel_in.linear_x - prev_target_velocity_.linear_x) / loop_period_.count();
    if(std::abs(commanded_acceleration) > params_.linear_accel_x) {
        vel_in.linear_x = prev_target_velocity_.linear_x + std::copysign(params_.linear_accel_x, commanded_acceleration) * loop_period_.count();
    }
    commanded_acceleration = (vel_in.linear_y - prev_target_velocity_.linear_y) / loop_period_.count();
    if(std::abs(commanded_acceleration) > params_.linear_accel_y) {
        vel_in.linear_y = prev_target_velocity_.linear_y + std::copysign(params_.linear_accel_y, commanded_acceleration) * loop_period_.count();
    }
    commanded_acceleration = (vel_in.angular_z - prev_target_velocity_.angular_z) / loop_period_.count();
    if(std::abs(commanded_acceleration) > params_.angular_accel_z) {
        vel_in.angular_z = prev_target_velocity_.angular_z + std::copysign(params_.angular_accel_z, commanded_acceleration) * loop_period_.count();
    }
}

void SwerveDriveController::apply_joint_limits(std::vector<double>& steer_angle_cmd, std::vector<double>& drive_vel_cmd) {
    bool zero_vel_command = true;
    for(int i = 0; i < num_modules_; i++) {
        if(std::abs(drive_vel_cmd[i]) > epsilon_) {
            zero_vel_command = false;
            break;
        }
    }

    if(zero_vel_command) {
        steer_angle_cmd = prev_steer_angle_cmd_;
    }

    double overall_velocity_reduction_factor = 1.0;
    for(int i = 0; i < num_modules_; i++) {
        double curr_angle = steer_angle_cmd[i];
        double alternate_angle;
        if(curr_angle < 0)
            alternate_angle = curr_angle + M_PI;
        else if(curr_angle > 0)
            alternate_angle = curr_angle - M_PI;
        else {
            if(curr_steer_wheel_angle_[i] < 0)
                alternate_angle = -1 * M_PI;
            else
                alternate_angle = M_PI;
        }

        bool curr_angle_valid = (
            curr_angle >= params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).min_steer_angle &&
            curr_angle <= params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).max_steer_angle
        );
        bool alternate_angle_valid = (
            alternate_angle >= params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).min_steer_angle &&
            alternate_angle <= params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).max_steer_angle
        );
        bool consider_curr_angle = true;

        if(!curr_angle_valid && !alternate_angle_valid) {
            RCLCPP_ERROR(get_node()->get_logger(), "Joint angle %f is out of bounds for %s", curr_angle, params_.swerve_modules_name[i].c_str());
            for(int j = 0; j < num_modules_; j++) {
                steer_angle_cmd[j] = prev_steer_angle_cmd_[j];
                drive_vel_cmd[j] = 0;
            }
            break;
        }
        else if(curr_angle_valid && alternate_angle_valid) {
            double curr_angle_diff = std::abs(curr_angle - curr_steer_wheel_angle_[i]);
            double alternate_angle_diff = std::abs(alternate_angle - curr_steer_wheel_angle_[i]);
            if(std::abs(curr_angle_diff - alternate_angle_diff) < 0.0872665 || curr_angle_diff < alternate_angle_diff) {
                consider_curr_angle = true;
            }
            else {
                consider_curr_angle = false;
            }
        }
        else if(curr_angle_valid && !alternate_angle_valid) {
            consider_curr_angle = true;
        }
        else if(!curr_angle_valid && alternate_angle_valid) {
            consider_curr_angle = false;
        }

        if(consider_curr_angle) {
            steer_angle_cmd[i] = curr_angle;
        }
        else {
            steer_angle_cmd[i] = alternate_angle;
            drive_vel_cmd[i] *= -1.0;
        }

        double curr_velocity_reduction_factor = std::abs(params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).max_drive_velocity / drive_vel_cmd[i]);
        if(curr_velocity_reduction_factor < 1.0) {
            RCLCPP_WARN(get_node()->get_logger(), "%s exceeds max velocity. Requested velocity: %f rad/s, Max velocity: %f rad/s", params_.swerve_modules_name[i].c_str(), drive_vel_cmd[i], params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).max_drive_velocity);
            if (curr_velocity_reduction_factor < overall_velocity_reduction_factor) {
                overall_velocity_reduction_factor = curr_velocity_reduction_factor;
            }
        }
    }

    if(overall_velocity_reduction_factor < 1.0) {
        RCLCPP_INFO(get_node()->get_logger(), "Overall velocity reduction factor: %f", overall_velocity_reduction_factor);
        for(int i = 0; i < num_modules_; i++) {
            drive_vel_cmd[i] *= overall_velocity_reduction_factor;
        }
    }

    apply_steer_command_limits(steer_angle_cmd, drive_vel_cmd);
    // apply_drive_command_limits(drive_vel_cmd);

    prev_steer_angle_cmd_ = steer_angle_cmd;
    prev_drive_wheel_vel_cmd_ = drive_vel_cmd;
}

void SwerveDriveController::apply_steer_command_limits(std::vector<double>& steer_angle_cmd, std::vector<double>& drive_vel_cmd) {
    
    bool detected_large_angle_difference = false;
    for(int i = 0; i < num_modules_; i++) {
        if(std::abs(steer_angle_cmd[i] - prev_steer_angle_cmd_[i]) > params_.max_steer_angle_difference) {
            detected_large_angle_difference = true;
            break;
        }
    }

    if(detected_large_angle_difference && large_angle_diff_handling_phase_ == "none") {
        large_angle_diff_handling_phase_ = "slowdown";
        RCLCPP_INFO(get_node()->get_logger(), "Large angle difference detected");
        RCLCPP_INFO(get_node()->get_logger(), "Large angle difference handling: Entering slowdown phase");
    }

    if(large_angle_diff_handling_phase_ == "slowdown") {
        if(base_at_zero_vel_) {
            RCLCPP_INFO(get_node()->get_logger(), "Large angle difference handling: Entering angle correction phase");
            large_angle_diff_handling_phase_ = "correction";
        }
        else {
            for(int i = 0; i < num_modules_; i++) {
                steer_angle_cmd[i] = prev_steer_angle_cmd_[i];
                drive_vel_cmd[i] = 0;
            }
        }
    }

    if(large_angle_diff_handling_phase_ == "correction") {
        bool target_angles_reached = true;
        for(int i = 0; i < num_modules_; i++) {
            if(std::abs(steer_angle_cmd[i] - curr_steer_wheel_angle_[i]) > params_.steer_angle_reached_threshold) {
                target_angles_reached = false;
                break;
            }
        }

        if(target_angles_reached) {
            RCLCPP_INFO(get_node()->get_logger(), "Large angle difference handling: Target angles reached, entering normal operation phase");
            large_angle_diff_handling_phase_ = "none";
        }
        else {
            for(int i = 0; i < num_modules_; i++) {
                drive_vel_cmd[i] = 0;
            }
        }
    }

    if(!detected_large_angle_difference && large_angle_diff_handling_phase_ == "none") {
        double commanded_steer_vel;
        for(int i = 0; i < num_modules_; i++) {
            commanded_steer_vel = (steer_angle_cmd[i] - prev_steer_angle_cmd_[i]) / loop_period_.count();
            if(std::abs(commanded_steer_vel) > params_.max_steer_velocity) {
                steer_angle_cmd[i] = prev_steer_angle_cmd_[i] + std::copysign(params_.max_steer_velocity, commanded_steer_vel) * loop_period_.count();
            }
        }
    }
}

void SwerveDriveController::apply_drive_command_limits(std::vector<double>& drive_vel_cmd) {
    double max_vel_diff = std::abs(drive_vel_cmd[0] - prev_drive_wheel_vel_cmd_[0]);
    double curr_vel_diff;
    for(int i = 0; i < num_modules_; i++) {
        curr_vel_diff = abs(drive_vel_cmd[i] - prev_drive_wheel_vel_cmd_[i]);
        if(curr_vel_diff > max_vel_diff) {
            max_vel_diff = curr_vel_diff;
        }
    }
    double proportional_accel, commanded_accel;
    for(int i = 0; i < num_modules_; i++) {
        proportional_accel = (prev_drive_wheel_vel_cmd_[i] / max_vel_diff) * params_.max_drive_acceleration;
        commanded_accel = (drive_vel_cmd[i] - prev_drive_wheel_vel_cmd_[i]) / loop_period_.count();
        if(std::abs(commanded_accel) > proportional_accel) {
            drive_vel_cmd[i] = prev_drive_wheel_vel_cmd_[i] + std::copysign(proportional_accel, commanded_accel) * loop_period_.count();
        }
    }
}

}  // namespace swerve_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  swerve_drive_controller::SwerveDriveController, controller_interface::ControllerInterface)