#ifndef CONVEYOR_BELT_CONTROLLER__CONVEYOR_BELT_CONTROLLER_HPP_
#define CONVEYOR_BELT_CONTROLLER__CONVEYOR_BELT_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include "controller_interface/controller_interface.hpp"
#include "rightbot_interfaces/srv/set_actuator_control_state.hpp"

#include "conveyor_belt_controller_parameters.hpp"


namespace conveyor_belt_controller
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

class ConveyorBeltController
    : public controller_interface::ControllerInterface
{
public:
    ConveyorBeltController();
    
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
    
    void halt_service_callback(
        rightbot_interfaces::srv::SetActuatorControlState::Request::SharedPtr req,
        rightbot_interfaces::srv::SetActuatorControlState::Response::SharedPtr resp
    );

protected:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

private:
    void process_halt_service();

    std::vector<std::string> required_command_interfaces_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> loaned_command_interfaces_;
    std::vector<std::string> required_state_interfaces_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> loaned_state_interfaces_;

    rclcpp::Service<rightbot_interfaces::srv::SetActuatorControlState>::SharedPtr halt_service_;
    std::map<std::string, int> cmd_to_int_map_ = {
        {"OPERATIONAL", 0},
        {"SOFT_STOP", 1},
        {"HARD_STOP", 2}
    };
    std::map<std::string, std::string> actuator_halt_cmd_interface_map_;
    std::map<std::string, std::string> actuator_halt_state_interface_map_;
    struct {
        std::vector<std::string> actuator_name, actuator_state;
        bool command_available = false, received_command = false, response_available = false, service_result;
        std::string result_msg;
        std::mutex mutex;
        std::condition_variable cv;
    } halt_service_shared_data_;
    struct {
        std::vector<std::string> actuator_name, actuator_state;
        bool command_available = false, first_pass, service_result, state_changed;
        int curr_functional_state;
    } halt_service_process_data_;
    std::chrono::time_point<std::chrono::system_clock> halt_service_start_time_;
    rclcpp::CallbackGroup::SharedPtr halt_service_callback_group_;
    rclcpp::SubscriptionOptions halt_service_subscription_options_;
};

}  // namespace conveyor_belt_controller

#endif  // CONVEYOR_BELT_CONTROLLER__CONVEYOR_BELT_CONTROLLER_HPP_