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

    num_modules_ = params_.swerve_modules_name.size();
    std::vector<std::pair<double, double>> module_positions;
    for(auto module_name : params_.swerve_modules_name) {
        double module_x_pos = params_.swerve_modules_name_map.at(module_name).wheel_position.x;
        double module_y_pos = params_.swerve_modules_name_map.at(module_name).wheel_position.y;
        module_positions.push_back({module_x_pos, module_y_pos});
    }

    kinematics_ = std::make_shared<SwerveDriveKinematics>(module_positions);
    odom_processor_ = std::make_shared<OdometryProcessor>();

    cmd_vel_expiry_time_ = std::chrono::time_point<std::chrono::system_clock>::now();
    cmd_velocity_ = Velocity();
    cmd_velocity_cp_ = Velocity();
    target_velocity_ = Velocity();
    prev_target_velocity_ = Velocity();
    curr_steer_wheel_angle_ = std::vector<double>(num_modules_, 0.0);
    curr_drive_wheel_angle_ = std::vector<double>(num_modules_, 0.0);
    cmd_vel_expired_ = true;
    base_at_zero_vel_ = true;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveDriveController::command_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::command_interface_configuration()");
    std::vector<std::string> conf_names;
    for (std::string module_name : params_.swerve_modules_name) {
        std::string steer_joint = params_.swerve_modules_name_map.at(module_name).steer_joint_name;
        std::string drive_joint = params_.swerve_modules_name_map.at(module_name).drive_joint_name;
        conf_names.push_back(steer_joint + "/position");
        conf_names.push_back(drive_joint + "/velocity");
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::state_interface_configuration()");
    std::vector<std::string> conf_names;
    for (std::string module_name : params_.swerve_modules_name) {
        std::string steer_joint = params_.swerve_modules_name_map.at(module_name).steer_joint_name;
        std::string drive_joint = params_.swerve_modules_name_map.at(module_name).drive_joint_name;
        conf_names.push_back(steer_joint + "/position");
        conf_names.push_back(drive_joint + "/position");
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn SwerveDriveController::on_configure(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController::on_configure()");
    for (std::string module_name : params_.swerve_modules_name) {
        std::string steer_joint = params_.swerve_modules_name_map.at(module_name).steer_joint_name;
        std::string drive_joint = params_.swerve_modules_name_map.at(module_name).drive_joint_name;

        required_command_interfaces_.push_back(steer_joint + "/position");
        required_command_interfaces_.push_back(drive_joint + "/velocity");
        
        required_state_interfaces_.push_back(steer_joint + "/position");
        required_state_interfaces_.push_back(drive_joint + "/position");
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
        10
    );

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
        cmd_vel_expiry_time_ = std::chrono::system_clock::now() + params_.cmd_vel_timeout;
    }
}

void SwerveDriveController::update_base_state_variables() {
    int i;
    i = 0;
    for(std::string module_name : params_.swerve_modules_name) {
        curr_steer_wheel_angle_[i] = loaned_state_interfaces_.at(params_.swerve_modules_name_map.at(module_name).steer_joint_name + "/position").get_value();
        curr_drive_wheel_angle_[i] = loaned_state_interfaces_.at(params_.swerve_modules_name_map.at(module_name).drive_joint_name + "/position").get_value();
        curr_drive_wheel_vel_[i] = loaned_state_interfaces_.at(params_.swerve_modules_name_map.at(module_name).drive_joint_name + "/velocity").get_value();
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
        double wheel_dist_diff = curr_drive_wheel_angle_[i] - prev_drive_wheel_angle_[i];
        swerve_modules_pos_diff_[i].first = wheel_dist_diff * std::cos(curr_steer_wheel_angle_[i]);
        swerve_modules_pos_diff_[i].second = wheel_dist_diff * std::sin(curr_steer_wheel_angle_[i]);
        swerve_modules_vel_[i].first = curr_drive_wheel_vel_[i] * std::cos(curr_steer_wheel_angle_[i]);
        swerve_modules_vel_[i].second = curr_drive_wheel_vel_[i] * std::sin(curr_steer_wheel_angle_[i]);
    }

    kinematics_->forward_kinematics(swerve_modules_pos_diff_, curr_base_pos_diff_);
    kinematics_->forward_kinematics(swerve_modules_vel_, curr_base_velocity_);

    odom_processor_->update_odometry(curr_base_pos_diff_, curr_base_velocity_);

    odom_pub_->publish(odom_processor_->odom_msg);

    if(curr_base_velocity_.is_zero()) {
        if(!base_at_zero_vel_) {
            RCLCPP_INFO(get_node()->get_logger(), "Drive reached at zero velocity");
        }
        base_at_zero_vel_ = true;
    }
    else {
        if(base_at_zero_vel_) {
            RCLCPP_INFO(get_node()->get_logger(), "Drive started moving again");
        }
        base_at_zero_vel_ = false;
    }

    prev_drive_wheel_angle_ = curr_drive_wheel_angle_;
}

void SwerveDriveController::handle_cmd_vel() {
    if(update_loop_first_pass_) {
        for(int i = 0; i < num_modules_; i++) {
            swerve_modules_vel_cmd_.emplace_back(0, 0);
        }
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
        target_velocity_.set_speed(0, 0, 0);
    }
    else {
        if(cmd_vel_expired_) {
            RCLCPP_INFO(get_node()->get_logger(), "Started receiving new command velocity");
        }
        cmd_vel_expired_ = false;
    }

    apply_kinematics_limits(target_velocity_);
    kinematics_->inverse_kinematics(target_velocity_, swerve_modules_vel_cmd_);
    for(int i = 0; i < num_modules_; i++) {
        steer_wheel_angle_cmd_[i] = std::atan2(swerve_modules_vel_cmd_[i].second, swerve_modules_vel_cmd_[i].first);
        drive_wheel_vel_cmd_[i] = 
    }

    prev_target_velocity_ = target_velocity_;
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

    commanded_acceleration = (vel_in.linear_x - prev_target_velocity_.linear_x) / loop_period_;
    if(std::abs(commanded_acceleration) > params_.linear_accel_x) {
        vel_in.linear_x = prev_target_velocity_.linear_x + std::copysign(params_.linear_accel_x, commanded_acceleration) * loop_period_;
    }
    commanded_acceleration = (vel_in.linear_y - prev_target_velocity_.linear_y) / loop_period_;
    if(std::abs(commanded_acceleration) > params_.linear_accel_y) {
        vel_in.linear_y = prev_target_velocity_.linear_y + std::copysign(params_.linear_accel_y, commanded_acceleration) * loop_period_;
    }
    commanded_acceleration = (vel_in.angular_z - prev_target_velocity_.angular_z) / loop_period_;
    if(std::abs(commanded_acceleration) > params_.angular_accel_z) {
        vel_in.angular_z = prev_target_velocity_.angular_z + std::copysign(params_.angular_accel_z, commanded_acceleration) * loop_period_;
    }
}

}  // namespace swerve_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  swerve_drive_controller::SwerveDriveController, controller_interface::ControllerInterface)