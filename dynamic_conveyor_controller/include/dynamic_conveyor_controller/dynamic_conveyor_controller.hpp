#ifndef DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_
#define DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_

#include <vector>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "dynamic_conveyor_controller/parameter_handler.hpp"


namespace dynamic_conveyor_controller
{

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
};

}  // namespace dynamic_conveyor_controller

#endif  // DYNAMIC_CONVEYOR_CONTROLLER__DYNAMIC_CONVEYOR_CONTROLLER_HPP_