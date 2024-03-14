#ifndef DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_
#define DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <condition_variable>
#include <mutex>
#include <cmath>
#include <chrono>


#include "controller_interface/controller_interface.hpp"
#include "dynamic_conveyor_controller/parameter_handler.hpp"
#include "rightbot_interfaces/srv/conveyor_command.hpp"
#include "rightbot_interfaces/msg/conveyor_state.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"


namespace dynamic_conveyor_controller
{

enum SystemStatus : uint32_t {
    CONVEYOR_OK = 1,
    BELT_RUNNING = 2
};

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

class DynamicConveyorController
    : public controller_interface::ControllerInterface
{
public:
    DynamicConveyorController();
    
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
    bool sanity_check();
    double get_equation_result(double x, std::vector<double> coeffs);
    double enc_angle_to_distance(double angle);
    void conveyor_command_service_callback(
        rightbot_interfaces::srv::ConveyorCommand::Request::SharedPtr req,
        rightbot_interfaces::srv::ConveyorCommand::Response::SharedPtr resp
    );
    bool wait_until_command_acknowledged();
    double get_travel_from_height(double height);
    double get_travel_from_angle(double angle);
    bool initial_values_valid();
    void imu_orientation_callback(const geometry_msgs::msg::Vector3Stamped msg);
    void update_conveyor_angle();


    Parameters params_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_interfaces_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_interfaces_;
    std::vector<double> height_cmd_inverse_kinematics_coeffs_ = {1.0, 0.0};
    std::vector<double> angle_cmd_inverse_kinematics_coeffs_ =  {1.0, 0.0};

    rclcpp::Service<rightbot_interfaces::srv::ConveyorCommand>::SharedPtr conveyor_commad_srv_;
    std::string response_string_ = "";
    std::mutex response_wait_mutex_;
    std::condition_variable response_wait_cv_;

    std::shared_ptr<rclcpp::Publisher<rightbot_interfaces::msg::ConveyorState>> state_pub_;
    rightbot_interfaces::msg::ConveyorState state_msg_;

    uint32_t system_status_ = 0;

    const double PI_ = 3.141592653589793;

    double left_encoder_angle_;
    double right_encoder_angle_;
    double left_encoder_distance_;
    double right_encoder_distance_;
    double left_gantry_raw_distance_;
    double right_gantry_raw_distance_;
    double left_gantry_initial_encoder_offset_;
    double right_gantry_initial_encoder_offset_;
    double left_gantry_distance_;
    double right_gantry_distance_;
    double left_gantry_target_diff_;
    double right_gantry_target_diff_;
    double target_belt_velocity_;
    double last_commanded_belt_velocity_ = 0.0;
    double current_belt_velocity_;
    double realign_difference_;
    double left_minus_right_travel_offset_;
    double relative_move_distance_;
    double left_final_moveback_distance_;
    double right_final_moveback_distance_;


    double gantry_travel_distance_;
    double left_gantry_target_distance_;
    double right_gantry_target_distance_;
    
    bool on_first_update_loop_ = true;
    bool gantry_move_request_available_ = false;
    bool executing_gantry_move_command_ = false;
    bool gantry_lift_request_available_ = false;
    bool belt_velocity_request_available_ = false;
    bool executing_belt_velocity_command_ = false;
    bool realign_left_ = false;
    bool realign_right_ = false;
    bool realign_request_available_ = false;
    bool commanded_stop_ = false;
    bool previously_in_belt_velocity_window_ = false;
    bool relative_move_request_available_ = false;
    bool is_sane_ = true;
    bool was_sane_ = true;
    bool check_sanity_ = true;
    bool command_available_ = false;
    bool command_acknowledged_ = false;

    rclcpp::Time lift_command_sent_time_;
    rclcpp::Time belt_command_sent_time_;
    rclcpp::Time belt_velocity_window_enter_time_;
    std::chrono::duration<double> gantry_target_timeout_;
    std::chrono::duration<double> belt_target_velocity_window_time_;
    std::chrono::duration<double> belt_command_timeout_;

    geometry_msgs::msg::Vector3Stamped imu_orientation_, imu_orientation_copy_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>> orientation_sub_;
    rclcpp::CallbackGroup::SharedPtr imu_msg_callback_group_;
    rclcpp::SubscriptionOptions imu_msg_subscription_options_;
    std::mutex orientation_mutex_;
    rclcpp::Time orientation_received_time_, orientation_received_time_copy_;
    std::chrono::duration<double> orientation_validity_;
    double conveyor_angle_;
};

}  // namespace dynamic_conveyor_controller

#endif  // DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_