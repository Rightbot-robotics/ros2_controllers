#ifndef GPIO_CONTROLLER_HPP_
#define GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include <gpio_controller_parameters.hpp>

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

class GPIOController : public controller_interface::ControllerInterface
{
public:
  GPIOController();
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  CallbackReturn on_init() override;

private:
  std::vector<std::string> inputs_;
  std::vector<std::string> outputs_;

protected:
};
}  // namespace gpio_controller

#endif  // GPIO_CONTROLLER_HPP_
