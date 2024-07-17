#ifndef DYNAMIC_CONVEYOR_LIFT_TUNER_CONTROLLER_HPP_
#define DYNAMIC_CONVEYOR_LIFT_TUNER_CONTROLLER_HPP_

#include <vector>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rightbot_interfaces/srv/conveyor_lift_tuner_command.hpp"


namespace dynamic_conveyor_lift_tuner_controller
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

class DynamicConveyorLiftTunerController
    : public controller_interface::ControllerInterface
{
public:
    DynamicConveyorLiftTunerController();
    
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
    void conveyor_lift_tuner_command_service_callback(
        rightbot_interfaces::srv::ConveyorLiftTunerCommand::Request::SharedPtr req, 
        rightbot_interfaces::srv::ConveyorLiftTunerCommand::Response::SharedPtr resp
    );

    rclcpp::Service<rightbot_interfaces::srv::ConveyorLiftTunerCommand>::SharedPtr conveyor_lift_tuner_commad_srv_;

    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_interfaces_;
    std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_interfaces_;

    double position_command_left_;
    double position_command_right_;
    
    double position_command_all_;

    double position_kp_command_;
    double position_ki_command_;
    double position_kd_command_;

    double velocity_kp_command_;
    double velocity_ki_command_;
    double velocity_kd_command_;

    float left_lift_postion_; 
    float right_lift_position_;
    
    bool stop_command_all_;
    
    bool sync_motors_ = false;

    double initial_offset_;
    double current_offset_;
    double offet_threshold_;
    
    int test_counter_;

protected:
};

}  // namespace dynamic_conveyor_lift_tuner_controller

#endif  // DYNAMIC_CONVEYOR_LIFT_TUNER_CONTROLLER_HPP_