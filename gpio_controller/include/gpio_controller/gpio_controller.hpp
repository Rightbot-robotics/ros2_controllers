#ifndef GPIO_CONTROLLER_HPP_
#define GPIO_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <algorithm>
#include "controller_interface/controller_interface.hpp"
#include "rightbot_interfaces/srv/gpio_command.hpp"
#include "gpio_controller_parameters.hpp"

namespace gpio_controller
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

class GPIOController
    : public controller_interface::ControllerInterface
{
public:
    GPIOController();
    
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
    void gpio_command_service_callback(
        rightbot_interfaces::srv::GpioCommand::Request::SharedPtr req, 
        rightbot_interfaces::srv::GpioCommand::Response::SharedPtr resp
    );

    rclcpp::Service<rightbot_interfaces::srv::GpioCommand>::SharedPtr gpio_command_srv_;

    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_interfaces_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_interfaces_;

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::vector<double> gpio_states_;
    std::vector<double> prev_gpio_states_;

    double gpio_command_;

    double prev_gpio_command_;

    double prev_bin_;

protected:
};

}  // namespace gpio_controller

#endif  // GPIO_CONTROLLER_HPP_