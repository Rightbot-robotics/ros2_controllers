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

    steer_fault_state_ = std::vector<bool>(num_modules_, false);
    prev_steer_fault_state_ = std::vector<bool>(num_modules_, false);
    steer_connection_break_state_ = std::vector<bool>(num_modules_, false);
    prev_steer_connection_break_state_ = std::vector<bool>(num_modules_, false);
    drive_fault_state_ = std::vector<bool>(num_modules_, false);
    prev_drive_fault_state_ = std::vector<bool>(num_modules_, false);
    drive_connection_break_state_ = std::vector<bool>(num_modules_, false);
    prev_drive_connection_break_state_ = std::vector<bool>(num_modules_, false);
    base_is_operational_ = true;

    steer_functional_state_ = std::vector<int>(num_modules_, 0);
    prev_steer_functional_state_ = std::vector<int>(num_modules_, 0);
    drive_functional_state_ = std::vector<int>(num_modules_, 0);
    prev_drive_functional_state_ = std::vector<int>(num_modules_, 0);

    executing_halt_task_ = false;
    halt_task_id_ = 0;

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
        
        conf_names.push_back(steer_joint + "/function_halt");
        conf_names.push_back(drive_joint + "/function_halt");
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
        
        conf_names.push_back(steer_joint + "/fault");
        conf_names.push_back(steer_joint + "/connection_break");
        conf_names.push_back(steer_joint + "/functional_state");
        
        conf_names.push_back(drive_joint + "/fault");
        conf_names.push_back(drive_joint + "/connection_break");
        conf_names.push_back(drive_joint + "/functional_state");
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
        required_command_interfaces_.push_back(drive_joint + "/function_halt");
        required_command_interfaces_.push_back(steer_joint + "/function_halt");
        
        required_state_interfaces_.push_back(steer_joint + "/position");
        required_state_interfaces_.push_back(drive_joint + "/position");
        required_state_interfaces_.push_back(drive_joint + "/velocity");
        
        required_state_interfaces_.push_back(steer_joint + "/fault");
        required_state_interfaces_.push_back(steer_joint + "/connection_break");
        required_state_interfaces_.push_back(steer_joint + "/functional_state");
        
        required_state_interfaces_.push_back(drive_joint + "/fault");
        required_state_interfaces_.push_back(drive_joint + "/connection_break");
        required_state_interfaces_.push_back(drive_joint + "/functional_state");
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

    halt_service_ = get_node()->create_service<rightbot_interfaces::srv::SetActuatorControlState>(
        "base_function_halt",
        std::bind(
            &SwerveDriveController::halt_service_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
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
    handle_base_faults();
    handle_halt_task();
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
        steer_fault_state_[i] = static_cast<bool>(loaned_state_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).steer_joint_name + "/fault").get().get_value() > epsilon_);
        drive_fault_state_[i] = static_cast<bool>(loaned_state_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).drive_joint_name + "/fault").get().get_value() > epsilon_);
        steer_connection_break_state_[i] = static_cast<bool>(loaned_state_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).steer_joint_name + "/connection_break").get().get_value() > epsilon_);
        drive_connection_break_state_[i] = static_cast<bool>(loaned_state_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).drive_joint_name + "/connection_break").get().get_value() > epsilon_);
        steer_functional_state_[i] = static_cast<int>(loaned_state_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).steer_joint_name + "/functional_state").get().get_value());
        drive_functional_state_[i] = static_cast<int>(loaned_state_interfaces_.at(params_.modules_info.swerve_modules_name_map.at(module_name).drive_joint_name + "/functional_state").get().get_value());
        i++;
    }
}

void SwerveDriveController::handle_base_faults() {
    bool abnormal_state_detected = false;
    
    for(int i = 0; i < num_modules_; i++) {
        if(steer_fault_state_[i] != prev_steer_fault_state_[i]) {
            if(steer_fault_state_[i]) {
                RCLCPP_ERROR(get_node()->get_logger(), "Steer joint of %s module went into fault state: %f", params_.swerve_modules_name[i].c_str(), steer_fault_state_[i]? 1.0 : 0.0);
                abnormal_state_detected = true;
            }
            else{
                RCLCPP_INFO(get_node()->get_logger(), "Steer joint of %s module changed to no fault state", params_.swerve_modules_name[i].c_str());
            }
        }
        if(drive_fault_state_[i] != prev_drive_fault_state_[i]) {
            if(steer_fault_state_[i]) {
                RCLCPP_ERROR(get_node()->get_logger(), "Drive joint of %s module went into fault state: %f", params_.swerve_modules_name[i].c_str(), drive_fault_state_[i]? 1.0 : 0.0);
                abnormal_state_detected = true;
            }
            else{
                RCLCPP_INFO(get_node()->get_logger(), "Drive joint of %s module changed to no fault state", params_.swerve_modules_name[i].c_str());
            }
        }
    }

    for(int i = 0; i < num_modules_; i++) {
        if(steer_connection_break_state_[i] != prev_steer_connection_break_state_[i]) {
            if(steer_connection_break_state_[i]) {
                RCLCPP_ERROR(get_node()->get_logger(), "Steer joint of %s module went into connection break state: %f", params_.swerve_modules_name[i].c_str(), steer_connection_break_state_[i]? 1.0 : 0.0);
                abnormal_state_detected = true;
            }
            else{
                RCLCPP_INFO(get_node()->get_logger(), "Steer joint of %s module changed to no connection break state", params_.swerve_modules_name[i].c_str());
            }
        }
        if(drive_connection_break_state_[i] != prev_drive_connection_break_state_[i]) {
            if(drive_connection_break_state_[i]) {
                RCLCPP_ERROR(get_node()->get_logger(), "Drive joint of %s module went into connection break state: %f", params_.swerve_modules_name[i].c_str(), drive_connection_break_state_[i]? 1.0 : 0.0);
                abnormal_state_detected = true;
            }
            else{
                RCLCPP_INFO(get_node()->get_logger(), "Drive joint of %s module changed to no connection break state", params_.swerve_modules_name[i].c_str());
            }
        }
    }

    for(int i = 0; i < num_modules_; i++) {
        if(steer_functional_state_[i] != prev_steer_functional_state_[i]) {
            if(steer_functional_state_[i] != halt_cmd_to_int_map_["OPERATIONAL"]) {
                RCLCPP_ERROR(get_node()->get_logger(), "Steer joint of %s module went into non-operational functional state: %d", params_.swerve_modules_name[i].c_str(), steer_functional_state_[i]);
                abnormal_state_detected = true;
            }
            else{
                RCLCPP_INFO(get_node()->get_logger(), "Steer joint of %s module changed to operational functional state", params_.swerve_modules_name[i].c_str());
            }
        }
        if(drive_functional_state_[i] != prev_drive_functional_state_[i]) {
            if(drive_functional_state_[i] != halt_cmd_to_int_map_["OPERATIONAL"]) {
                RCLCPP_ERROR(get_node()->get_logger(), "Drive joint of %s module went into non-operational functional state: %d", params_.swerve_modules_name[i].c_str(), drive_functional_state_[i]);
                abnormal_state_detected = true;
            }
            else{
                RCLCPP_INFO(get_node()->get_logger(), "Drive joint of %s module changed to operational functional state", params_.swerve_modules_name[i].c_str());
            }
        }
    }

    if(abnormal_state_detected && base_is_operational_) {
        RCLCPP_INFO(get_node()->get_logger(), "Sending hard stop to all motors");
        auto halt_task = get_new_halt_task();
        halt_task->type = "HARD_STOP";
        {
            std::lock_guard<std::mutex> lock(halt_tasks_mutex_);
            halt_tasks_.push(halt_task);
        }
        base_is_operational_ = false;
    }

    if(!abnormal_state_detected && !base_is_operational_) {
        RCLCPP_INFO(get_node()->get_logger(), "Setting bot state as normal");
        base_is_operational_ = true;
    }

    prev_steer_fault_state_ = steer_fault_state_;
    prev_drive_fault_state_ = drive_fault_state_;
    prev_steer_connection_break_state_ = steer_connection_break_state_;
    prev_drive_connection_break_state_ = drive_connection_break_state_;
    prev_steer_functional_state_ = steer_functional_state_;
    prev_drive_functional_state_ = drive_functional_state_;
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

    if(!base_is_operational_) {
        target_velocity_.set_speed(0.0, 0.0, 0.0);
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

    double accel_x, accel_y, accel_w;
    double time_x, time_y, time_w, max_time = 0.0;

    accel_x = (vel_in.linear_x - prev_target_velocity_.linear_x) / loop_period_.count();
    if(std::abs(accel_x) > params_.linear_accel_x) {
        accel_x = std::copysign(params_.linear_accel_x, accel_x);
    }
    if(std::abs(accel_x) < 1e-6) {
        time_x = 0.0;
    }
    else {
        time_x = std::abs((vel_in.linear_x - prev_target_velocity_.linear_x) / accel_x);
    }
    max_time = std::max(time_x, max_time);

    accel_y = (vel_in.linear_y - prev_target_velocity_.linear_y) / loop_period_.count();
    if(std::abs(accel_y) > params_.linear_accel_y) {
        accel_y = std::copysign(params_.linear_accel_y, accel_y);
    }
    if(std::abs(accel_y) < 1e-6) {
        time_y = 0.0;
    }
    else {
        time_y = std::abs((vel_in.linear_y - prev_target_velocity_.linear_y) / accel_y);
    }
    max_time = std::max(time_y, max_time);

    accel_w = (vel_in.angular_z - prev_target_velocity_.angular_z) / loop_period_.count();
    if(std::abs(accel_w) > params_.angular_accel_z) {
        accel_w = std::copysign(params_.angular_accel_z, accel_w);
    }
    if(std::abs(accel_w) < 1e-6) {
        time_w = 0.0;
    }
    else {
        time_w = std::abs((vel_in.angular_z - prev_target_velocity_.angular_z) / accel_w);
    }
    max_time = std::max(time_w, max_time);

    if(max_time > 1e-6) {
        vel_in.linear_x = prev_target_velocity_.linear_x + (time_x / max_time) * accel_x * loop_period_.count();
        vel_in.linear_y = prev_target_velocity_.linear_y + (time_y / max_time) * accel_y * loop_period_.count();
        vel_in.angular_z = prev_target_velocity_.angular_z + (time_w / max_time) * accel_w * loop_period_.count();
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
    apply_drive_command_limits(drive_vel_cmd);

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
        proportional_accel = (abs(drive_vel_cmd[i] - prev_drive_wheel_vel_cmd_[i]) / max_vel_diff) * params_.max_drive_acceleration;
        commanded_accel = (drive_vel_cmd[i] - prev_drive_wheel_vel_cmd_[i]) / loop_period_.count();
        if(std::abs(commanded_accel) > proportional_accel) {
            drive_vel_cmd[i] = prev_drive_wheel_vel_cmd_[i] + std::copysign(proportional_accel, commanded_accel) * loop_period_.count();
        }
    }
}

void SwerveDriveController::handle_halt_task() {
    if(!executing_halt_task_)
    {
        std::lock_guard lk(halt_tasks_mutex_);
        if(!halt_tasks_.empty()) {
            current_halt_task_.reset();
            current_halt_task_ = halt_tasks_.front();
            halt_tasks_.pop();
            executing_halt_task_ = true;
        }
    }

    if(current_halt_task_ == nullptr) {
        return;
    }

    if(!current_halt_task_->initialized) {
        RCLCPP_INFO(get_node()->get_logger(), "Executing halt task, task id: %d, task type: %s", current_halt_task_->id, current_halt_task_->type.c_str());
        RCLCPP_INFO(get_node()->get_logger(), "Setting halt command interface for all swerve modules");
        for(int i = 0; i < num_modules_; i++) {
            std::string steer_joint = params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).steer_joint_name;
            std::string drive_joint = params_.modules_info.swerve_modules_name_map.at(params_.swerve_modules_name[i]).drive_joint_name;
            if(!steer_connection_break_state_[i]) {
                loaned_command_interfaces_.at(steer_joint + "/function_halt").get().set_value(halt_cmd_to_int_map_.at(current_halt_task_->type));
            }
            if(!drive_connection_break_state_[i]) {
                loaned_command_interfaces_.at(drive_joint + "/function_halt").get().set_value(halt_cmd_to_int_map_.at(current_halt_task_->type));
            }
        }
        RCLCPP_INFO(get_node()->get_logger(), "Halt command interface set for all swerve modules");
        current_halt_task_->initialized = true;
        return;
    }

    bool halt_task_complete = true;
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *(get_node())->get_clock(), 1000, "Waiting for halt task to complete, task id: %d, task type: %s", current_halt_task_->id, current_halt_task_->type.c_str());
    for(int i = 0; i < num_modules_; i++) {
        if(!steer_connection_break_state_[i]) {
            if(steer_functional_state_[i] != halt_cmd_to_int_map_.at(current_halt_task_->type)) {
                halt_task_complete = false;
                break;
            }
        }

        if(!drive_connection_break_state_[i]) {
            if(drive_functional_state_[i] != halt_cmd_to_int_map_.at(current_halt_task_->type)) {
                halt_task_complete = false;
                break;
            }
        }
    }

    bool halt_task_timed_out = false;
    if(!halt_task_complete) {
        current_halt_task_->time_elapsed = std::chrono::duration<double>(std::chrono::system_clock::now() - current_halt_task_->task_start_time).count();
        if(current_halt_task_->time_elapsed > params_.halt_task_timeout) {
            halt_task_complete = true;
            halt_task_timed_out = true;
        }
    }

    if(halt_task_complete) {
        RCLCPP_INFO(get_node()->get_logger(), "Halt task complete, task id: %d, task type: %s", current_halt_task_->id, current_halt_task_->type.c_str());
        if(!halt_task_timed_out) {
            std::lock_guard lk(current_halt_task_->mutex);
            current_halt_task_->response_available = true;
            current_halt_task_->task_successful = true;
            current_halt_task_->response = "SUCCESSFUL";
        }
        else {
            std::lock_guard lk(current_halt_task_->mutex);
            current_halt_task_->response_available = true;
            current_halt_task_->task_successful = false;
            current_halt_task_->response = "TIMED_OUT";
        }
        current_halt_task_->cv.notify_all();
        current_halt_task_.reset();
        executing_halt_task_ = false;
    }
}

std::shared_ptr<HaltTask> SwerveDriveController::get_new_halt_task() {
    std::shared_ptr<HaltTask> new_halt_task = std::make_shared<HaltTask>();
    new_halt_task->id = halt_task_id_;
    halt_task_id_++;
    return new_halt_task;
}

void SwerveDriveController::halt_service_callback(
    rightbot_interfaces::srv::SetActuatorControlState::Request::SharedPtr req,
    rightbot_interfaces::srv::SetActuatorControlState::Response::SharedPtr resp
) {
    RCLCPP_INFO(get_node()->get_logger(), "[halt_service_callback] Received set actuator control state service request");
    
    if(req->actuator_state.size() == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "[halt_service_callback] No halt task specified");
        resp->success = false;
        resp->msg = "EMPTY_TASK";
        return;
    }

    if(halt_cmd_to_int_map_.count(req->actuator_state[0]) == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "[halt_service_callback] Invalid halt task specified");
        resp->success = false;
        resp->msg = "INVALID_TASK";
        return;
    }

    RCLCPP_INFO(get_node()->get_logger(), "[halt_service_callback] Validity check passed");
    auto halt_task = get_new_halt_task();
    halt_task->type = req->actuator_state[0];
    {
        std::lock_guard<std::mutex> lock(halt_tasks_mutex_);
        halt_tasks_.push(halt_task);
    }

    {
        std::unique_lock<std::mutex> lock(halt_task->mutex);
        halt_task->cv.wait_for(lock, std::chrono::duration<double>(params_.halt_task_timeout * 1.2), [&halt_task]() { return halt_task->response_available; });
        if(halt_task->response_available) {
            RCLCPP_INFO(get_node()->get_logger(), "[halt_service_callback] Halt task response received");
            RCLCPP_INFO(get_node()->get_logger(), "[halt_service_callback] Halt task successful: %s", halt_task->task_successful ? "true" : "false");
            RCLCPP_INFO(get_node()->get_logger(), "[halt_service_callback] Halt task response: %s", halt_task->response.c_str());
            resp->success = halt_task->task_successful;
            resp->msg = halt_task->response;
        }
        else {
            RCLCPP_ERROR(get_node()->get_logger(), "[halt_service_callback] Halt task response not received");
            resp->success = false;
            resp->msg = "TIMEOUT";
        }
    }
}

}  // namespace swerve_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  swerve_drive_controller::SwerveDriveController, controller_interface::ControllerInterface)