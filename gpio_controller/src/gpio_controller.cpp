#include <gpio_controller/gpio_controller.hpp>


namespace gpio_controller
{

GPIOController::GPIOController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn GPIOController::on_init() {
    RCLCPP_INFO(get_node()->get_logger(), "GPIOController::on_init()");
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    for (size_t i = 0; i < params_.outputs.size(); i++)
    {
        gpio_states_.push_back(0.0);
        prev_gpio_states_.push_back(0.0);
    }
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "GPIOController::command_interface_configuration()");
    std::vector<std::string> conf_names;
    
    for (size_t i = 0; i < params_.outputs.size(); i++)
    {
        conf_names.push_back(params_.output_mapping.outputs_map.at(params_.outputs.at(i)).interface.data());
    }

    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const {
    RCLCPP_INFO(get_node()->get_logger(), "GPIOController::state_interface_configuration()");
    std::vector<std::string> conf_names;

    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type GPIOController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    // RCLCPP_INFO(get_node()->get_logger(), "GPIOController::update()");

    for (size_t i = 0; i < params_.outputs.size(); i++)
    {
        if (prev_gpio_states_[i] != gpio_states_[i])
        {
            joint_command_interfaces_.at(params_.output_mapping.outputs_map.at(params_.outputs.at(i)).interface.data()).get().set_value(gpio_command_);
        }
            
    }

    prev_gpio_states_ = gpio_states_;
    prev_gpio_command_ = gpio_command_;

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GPIOController::on_configure(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "GPIOController::on_configure()");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPIOController::on_activate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "GPIOController::on_activate()");
    
    // Acquiring command interfaces
    std::vector<std::string> required_command_interfaces_name {};

    for (size_t i = 0; i < params_.outputs.size(); i++)
    {
        required_command_interfaces_name.push_back(params_.output_mapping.outputs_map.at(params_.outputs.at(i)).interface.data());
    }
    

    if(!get_loaned_interfaces(
        command_interfaces_,
        required_command_interfaces_name,
        joint_command_interfaces_
    )) {
        RCLCPP_ERROR(get_node()->get_logger(), "All the requested command interface is not found");
        return controller_interface::CallbackReturn::ERROR;
    }

    // Creating services
    gpio_command_srv_ = get_node()->create_service<rightbot_interfaces::srv::GpioCommand>(
        "gpio_command",
        std::bind(
            &GPIOController::gpio_command_service_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPIOController::on_deactivate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "GPIOController::on_deactivate()");
    gpio_command_srv_.reset();
    joint_command_interfaces_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}

int GPIOController::binaryToDecimal(int n)
{
    int num = n;
    int dec_value = 0;
 
    // Initializing base value to 1, i.e 2^0
    int base = 1;
 
    int temp = num;
    while (temp) {
        int last_digit = temp % 10;
        temp = temp / 10;
 
        dec_value += last_digit * base;
 
        base = base * 2;
    }
 
    return dec_value;
}

void GPIOController::gpio_command_service_callback(
    rightbot_interfaces::srv::GpioCommand::Request::SharedPtr req,
    rightbot_interfaces::srv::GpioCommand::Response::SharedPtr resp) {
    
    // RCLCPP_INFO(get_node()->get_logger(), "[GPIOController] gpio_command_service_callback");

        for (size_t i = 0; i < params_.outputs.size(); i++)
        {
            if(req->gpio_names[i] == params_.outputs.at(i)) {
                // RCLCPP_INFO(get_node()->get_logger(), "Output %s", params_.output_mapping.outputs_map.at(params_.outputs.at(i)).interface.data());
                // RCLCPP_INFO(get_node()->get_logger(), "Output %i", i);
                gpio_states_[i] = req->gpio_states[i];
            }
        }
        auto binary = std::accumulate( gpio_states_.begin(), gpio_states_.end(), 0, []( int l, int r ) {
            return l * 10 + r; 
        } );

        gpio_command_ = binaryToDecimal(binary);

        RCLCPP_INFO(get_node()->get_logger(), "Output %s", std::to_string(gpio_command_).c_str());

    resp->status = true;    
}  
} // namespace gpio_controller
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  gpio_controller::GPIOController, controller_interface::ControllerInterface)