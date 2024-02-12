#ifndef DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_
#define DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <iostream>

#include "controller_interface/controller_interface.hpp"
#include "dynamic_conveyor_controller/parameter_handler.hpp"


namespace dynamic_conveyor_controller
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

protected:
    Parameters params_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_interfaces_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_interfaces_;
};

}  // namespace dynamic_conveyor_controller

#endif  // DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_