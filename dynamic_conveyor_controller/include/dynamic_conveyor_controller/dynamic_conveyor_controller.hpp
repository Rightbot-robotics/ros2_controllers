#ifndef DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_
#define DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <condition_variable>
#include <mutex>
#include <cmath>


#include "controller_interface/controller_interface.hpp"
#include "dynamic_conveyor_controller/parameter_handler.hpp"
#include "rightbot_interfaces/srv/conveyor_command.hpp"
#include "rightbot_interfaces/msg/conveyor_status.hpp"


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
    Parameters params_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_interfaces_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_interfaces_;
    std::vector<double> height_cmd_inverse_kinematics_coeffs_;
    std::vector<double> angle_cmd_inverse_kinematics_coeffs_;
    double get_equation_result(double x, std::vector<double> coeffs);

    rclcpp::Service<rightbot_interfaces::srv::ConveyorCommand>::SharedPtr conveyor_commad_srv_;
    std::string response_string_ = "";
    std::mutex response_wait_mutex_;
    std::condition_variable response_wait_cv_;
    void conveyor_command_service_callback(
        rightbot_interfaces::srv::ConveyorCommand::Request::SharedPtr req,
        rightbot_interfaces::srv::ConveyorCommand::Response::SharedPtr resp
    );

    uint32_t system_status_ = 0;

    double left_gantry_initial_offset_ = 0.0;
    double right_gantry_initial_offset_ = 0.0;
    double left_gantry_target_distance_ = 0.0;
    double right_gantry_target_distance_ = 0.0;
    bool lift_request_available_ = false;

    double target_belt_velocity_ = 0.0;
    double last_commanded_belt_velocity_ = 0.0;
    double belt_velocity_request_available_ = false;

    bool realign_left_ = false;
    bool realign_right_ = false;
    bool realign_request_available_ = false;
};

}  // namespace dynamic_conveyor_controller

#endif  // DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_