#ifndef CONVEYOR_BELT_CONTROLLER__CONVEYOR_BELT_CONTROLLER_HPP_
#define CONVEYOR_BELT_CONTROLLER__CONVEYOR_BELT_CONTROLLER_HPP_

#include <vector>
#include <string>

#include "controller_interface/controller_interface.hpp"


namespace conveyor_belt_controller
{

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

protected:
};

}  // namespace conveyor_belt_controller

#endif  // CONVEYOR_BELT_CONTROLLER__CONVEYOR_BELT_CONTROLLER_HPP_