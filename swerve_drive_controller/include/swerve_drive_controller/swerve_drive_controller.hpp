#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_

#include <vector>
#include <utility>
#include <string>
#include <memory>
#include <mutex>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_ros/transform_broadcaster.h"

#include "controller_interface/controller_interface.hpp"

#include <swerve_drive_controller/types.hpp>
#include <swerve_drive_controller/kinematics.hpp>
#include <swerve_drive_controller/odometry.hpp>

#include "swerve_drive_controller_parameters.hpp"


namespace swerve_drive_controller
{

template <typename T>
bool get_loaned_interfaces(
    std::vector<T> & available_interfaces,
    const std::vector<std::string> & requested_interface_names,
    std::map<std::string, std::reference_wrapper<T>> & interface_map
) {
    bool all_interface_found = true;
    for(const auto & interface_name : requested_interface_names) {
        bool found_current_interface = false;
        for(auto & interface : available_interfaces) {
            if(interface.get_name() == interface_name) {
                interface_map.emplace(interface_name, std::ref(interface));
                found_current_interface = true;
            }
        }
        all_interface_found = all_interface_found && found_current_interface;
    }
    return all_interface_found;
}

class SwerveDriveController
    : public controller_interface::ControllerInterface
{
public:
    SwerveDriveController();
    
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    
    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    
    controller_interface::CallbackReturn on_init() override;

    
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void handle_odometry();
    void update_base_state_variables();
    void handle_cmd_vel();
    void apply_kinematics_limits(Velocity& vel_in);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool update_loop_first_pass_;
    std::chrono::time_point<std::chrono::system_clock> curr_loop_time_, prev_loop_time_;
    std::chrono::duration<double> loop_period_, max_loop_period_;
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    int num_modules_;

    std::shared_ptr<SwerveDriveKinematics> kinematics_;
    std::shared_ptr<OdometryProcessor> odom_processor_;

    std::string base_frame_id_, odom_frame_id_;

    std::vector<std::string> required_command_interfaces_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> loaned_command_interfaces_;
    std::vector<std::string> required_state_interfaces_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> loaned_state_interfaces_;

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;
    Velocity cmd_velocity_, cmd_velocity_cp_, target_velocity_, prev_target_velocity_;
    std::chrono::time_point<std::chrono::system_clock> cmd_vel_expiry_time_, cmd_vel_expiry_time_cp_;
    bool cmd_vel_expired_;
    std::mutex cmd_vel_mutex_;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
    std::vector<double> curr_steer_wheel_angle_;
    std::vector<double> curr_drive_wheel_angle_, prev_drive_wheel_angle_;
    std::vector<double> curr_drive_wheel_vel_;
    std::vector<std::pair<double, double>> swerve_modules_pos_diff_, swerve_modules_vel_;
    Velocity curr_base_pos_diff_, curr_base_velocity_;
    bool base_at_zero_vel_;


    std::vector<std::pair<double, double>> swerve_modules_vel_cmd_;
    std::vector<double> steer_wheel_angle_cmd_;
    std::vector<double> drive_wheel_vel_cmd_;
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_