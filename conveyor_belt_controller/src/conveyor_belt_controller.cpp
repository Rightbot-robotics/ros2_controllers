#include <conveyor_belt_controller/conveyor_belt_controller.hpp>


namespace conveyor_belt_controller
{

ConveyorBeltController::ConveyorBeltController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ConveyorBeltController::on_init() {
    RCLCPP_DEBUG(get_node()->get_logger(), "ConveyorBeltController::on_init()");
    try
    {
        // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    {
        std::lock_guard lk(halt_service_shared_data_.mutex);
        halt_service_shared_data_.command_available = false;
        halt_service_shared_data_.response_available = false;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ConveyorBeltController::command_interface_configuration() const {
    RCLCPP_DEBUG(get_node()->get_logger(), "ConveyorBeltController::command_interface_configuration()");
    std::vector<std::string> conf_names;
    for (size_t i = 0; i < params_.actuator_names.size(); i++) {
        std::string req_cmd_interface = params_.actuator_info.actuator_names_map.at(params_.actuator_names.at(i)).function_halt_command_interface.data();
        conf_names.push_back(req_cmd_interface);
        RCLCPP_DEBUG(get_node()->get_logger(), "requesting command interface: %s", req_cmd_interface.c_str());
    }
    for (size_t i = 0; i < params_.actuator_names.size(); i++) {
        std::string req_cmd_interface = params_.actuator_info.actuator_names_map.at(params_.actuator_names.at(i)).velocity_command_interface.data();
        conf_names.push_back(req_cmd_interface);
        RCLCPP_DEBUG(get_node()->get_logger(), "requesting command interface: %s", req_cmd_interface.c_str());
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration ConveyorBeltController::state_interface_configuration() const {
    RCLCPP_DEBUG(get_node()->get_logger(), "ConveyorBeltController::state_interface_configuration()");
    std::vector<std::string> conf_names;
    for (size_t i = 0; i < params_.actuator_names.size(); i++) {
        std::string req_state_interface = params_.actuator_info.actuator_names_map.at(params_.actuator_names.at(i)).functional_state_interface.data();
        conf_names.push_back(req_state_interface);
        RCLCPP_DEBUG(get_node()->get_logger(), "requesting state interface: %s", req_state_interface.c_str());
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn ConveyorBeltController::on_configure(
    const rclcpp_lifecycle::State &) {
    RCLCPP_DEBUG(get_node()->get_logger(), "ConveyorBeltController::on_configure()");
    for (size_t i = 0; i < params_.actuator_names.size(); i++) {
        std::string req_cmd_interface = params_.actuator_info.actuator_names_map.at(params_.actuator_names.at(i)).function_halt_command_interface.data();
        actuator_halt_cmd_interface_map_.emplace(params_.actuator_names.at(i), req_cmd_interface);
        required_command_interfaces_.push_back(req_cmd_interface);
        
        req_cmd_interface = params_.actuator_info.actuator_names_map.at(params_.actuator_names.at(i)).velocity_command_interface.data();
        actuator_velocity_interface_map_.emplace(params_.actuator_names.at(i), req_cmd_interface);
        required_command_interfaces_.push_back(req_cmd_interface);
        
        std::string req_state_interface = params_.actuator_info.actuator_names_map.at(params_.actuator_names.at(i)).functional_state_interface.data();
        actuator_halt_state_interface_map_.emplace(params_.actuator_names.at(i), req_state_interface);
        required_state_interfaces_.push_back(req_state_interface);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConveyorBeltController::on_activate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_DEBUG(get_node()->get_logger(), "ConveyorBeltController::on_activate()");

    if(!get_loaned_interfaces(
        command_interfaces_,
        required_command_interfaces_,
        loaned_command_interfaces_
    )) {
        RCLCPP_ERROR(get_node()->get_logger(), "All the requested command interface is not found");
        return controller_interface::CallbackReturn::ERROR;
    }
    if(!get_loaned_interfaces(
        state_interfaces_,
        required_state_interfaces_,
        loaned_state_interfaces_
    )) {
        RCLCPP_ERROR(get_node()->get_logger(), "All the requested state interface is not found");
        return controller_interface::CallbackReturn::ERROR;
    }

    halt_service_callback_group_ = get_node()->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );
    halt_service_subscription_options_ = rclcpp::SubscriptionOptions();
    halt_service_subscription_options_.callback_group = halt_service_callback_group_;
    
    halt_service_ = get_node()->create_service<rightbot_interfaces::srv::SetActuatorControlState>(
        "actuator_function_halt",
        std::bind(
            &ConveyorBeltController::halt_service_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    conveyor_service_ = get_node()->create_service<rightbot_interfaces::srv::ConveyorBeltCommand>(
        "conveyor_belt_command",
        std::bind(
            &ConveyorBeltController::conveyor_service_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ConveyorBeltController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    process_halt_service();
    process_conveyor_service();
    return controller_interface::return_type::OK;
}

void ConveyorBeltController::halt_service_callback(
    rightbot_interfaces::srv::SetActuatorControlState::Request::SharedPtr req,
    rightbot_interfaces::srv::SetActuatorControlState::Response::SharedPtr resp
) {
    RCLCPP_INFO(get_node()->get_logger(), "[halt_service_callback] Received set actuator control state service request");
    std::string requested_actuators, requested_states;

    for (size_t i = 0; i < req->actuator_name.size(); i++) {
        requested_actuators += req->actuator_name[i];
        requested_actuators += ", ";
        requested_states += req->actuator_state[i];
        requested_states += ", ";
    }

    RCLCPP_INFO(get_node()->get_logger(), "[halt_service_callback] Requested actuators: %s", requested_actuators.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "[halt_service_callback] Requested states: %s", requested_states.c_str());

    {
        std::lock_guard lk(halt_service_shared_data_.mutex);
        halt_service_shared_data_.actuator_name = req->actuator_name;
        halt_service_shared_data_.actuator_state = req->actuator_state;
        halt_service_shared_data_.response_available = false;
        halt_service_shared_data_.received_command = false;
        halt_service_shared_data_.command_available = true;
    }

    {
        std::unique_lock lk(halt_service_shared_data_.mutex);
        halt_service_shared_data_.cv.wait_for(lk, std::chrono::milliseconds((int)(params_.halt_service_timeout_sec * 1000 * 1.5)), [this](){return halt_service_shared_data_.response_available;});
        if (halt_service_shared_data_.response_available) {
            resp->success = halt_service_shared_data_.service_result;
            resp->msg = halt_service_shared_data_.result_msg;
        }
        else {
            RCLCPP_ERROR(get_node()->get_logger(), "[halt_service_callback] Halt service timed out waiting for processing response");
            resp->success = false;
            resp->msg = "TIMEOUT";
        }
        halt_service_shared_data_.command_available = false;
    }
}

void ConveyorBeltController::process_halt_service() {
    // This function should be non blocking as it is called in update loop
    {
        std::lock_guard lk(halt_service_shared_data_.mutex);
        halt_service_process_data_.command_available = halt_service_shared_data_.command_available;
        if (halt_service_shared_data_.command_available) {
            if (!halt_service_shared_data_.received_command) {
                halt_service_shared_data_.received_command = true;
                halt_service_process_data_.actuator_name = halt_service_shared_data_.actuator_name;
                halt_service_process_data_.actuator_state = halt_service_shared_data_.actuator_state;
                halt_service_process_data_.first_pass = true;
                halt_service_start_time_ = std::chrono::system_clock::now();
            }
        }
        else {
            return;
        }
    }

    // Check validity of command at first pass
    if (halt_service_process_data_.first_pass) {
        // Size check
        RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Running size check on halt service request");
        if (halt_service_process_data_.actuator_name.size() != halt_service_process_data_.actuator_state.size()) {
            RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Invalid service request: actuator_name and actuator_state size mismatch");
            {
                std::lock_guard lk(halt_service_shared_data_.mutex);
                halt_service_shared_data_.service_result = false;
                halt_service_shared_data_.result_msg = "INVALID REQUEST";
                halt_service_shared_data_.response_available = true;
                halt_service_shared_data_.command_available = false;
            }
            halt_service_shared_data_.cv.notify_all();
            return;
        }

        // Command type check
        RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Running command type check on halt service request");
        for (size_t i = 0; i < halt_service_process_data_.actuator_state.size(); i++) {
            if (cmd_to_int_map_.count(halt_service_process_data_.actuator_state.at(i)) == 0) {
                RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Invalid service request: invalid command type");
                {
                    std::lock_guard lk(halt_service_shared_data_.mutex);
                    halt_service_shared_data_.service_result = false;
                    halt_service_shared_data_.result_msg = "INVALID REQUEST";
                    halt_service_shared_data_.response_available = true;
                    halt_service_shared_data_.command_available = false;
                }
                halt_service_shared_data_.cv.notify_all();
                return;
            }
        }

        // Check the correctness of actuator names
        RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Running actuator name check on halt service request");
        for (size_t i = 0; i < halt_service_process_data_.actuator_state.size(); i++) {
            if (actuator_halt_cmd_interface_map_.count(halt_service_process_data_.actuator_name.at(i)) == 0) {
                RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Invalid service request: command interface for %s not found", halt_service_process_data_.actuator_name.at(i).c_str());
                {
                    std::lock_guard lk(halt_service_shared_data_.mutex);
                    halt_service_shared_data_.service_result = false;
                    halt_service_shared_data_.result_msg = "INVALID REQUEST";
                    halt_service_shared_data_.response_available = true;
                    halt_service_shared_data_.command_available = false;
                }
                halt_service_shared_data_.cv.notify_all();
                return;
            }
        }
        for (size_t i = 0; i < halt_service_process_data_.actuator_state.size(); i++) {
            if (actuator_halt_state_interface_map_.count(halt_service_process_data_.actuator_name.at(i)) == 0) {
                RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Invalid service request: command interface for %s not found", halt_service_process_data_.actuator_name.at(i).c_str());
                {
                    std::lock_guard lk(halt_service_shared_data_.mutex);
                    halt_service_shared_data_.service_result = false;
                    halt_service_shared_data_.result_msg = "INVALID REQUEST";
                    halt_service_shared_data_.response_available = true;
                    halt_service_shared_data_.command_available = false;
                }
                halt_service_shared_data_.cv.notify_all();
                return;
            }
        }
    }

    // Send command at first pass
    if (halt_service_process_data_.first_pass) {
        RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Processing halt service request");
        for (size_t i = 0; i < halt_service_process_data_.actuator_name.size(); i++) {
            loaned_command_interfaces_.at(
                actuator_halt_cmd_interface_map_.at(halt_service_process_data_.actuator_name.at(i))
            ).get().set_value((double)cmd_to_int_map_.at(halt_service_process_data_.actuator_state.at(i)));
            RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Sending %d to %s",
                cmd_to_int_map_.at(halt_service_process_data_.actuator_state.at(i)),
                halt_service_process_data_.actuator_name.at(i).c_str()
            );
        }
    }

    // Check if state changed
    {
        halt_service_process_data_.state_changed = true;
        for (size_t i = 0; i < halt_service_process_data_.actuator_name.size(); i++) {
            halt_service_process_data_.curr_functional_state = (int)loaned_state_interfaces_.at(actuator_halt_state_interface_map_.at(halt_service_process_data_.actuator_name.at(i))).get().get_value();
            if(
                halt_service_process_data_.curr_functional_state
                != cmd_to_int_map_.at(halt_service_process_data_.actuator_state.at(i))
            ) {
                halt_service_process_data_.state_changed = halt_service_process_data_.state_changed && false;
            }
        }
        if(halt_service_process_data_.state_changed) {
            RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Actuator functional states have been set successfully");
            {
                std::lock_guard lk(halt_service_shared_data_.mutex);
                halt_service_shared_data_.service_result = true;
                halt_service_shared_data_.result_msg = "SUCCESS";
                halt_service_shared_data_.command_available = false;
                halt_service_shared_data_.response_available = true;
            }
            halt_service_shared_data_.cv.notify_all();
            return;
        }
    }

    // Check if timeout
    {
        if (
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - halt_service_start_time_
            ).count() > (int)(params_.halt_service_timeout_sec * 1000)
        ) {
            RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Timeout waiting for state to change");
            {
                std::lock_guard lk(halt_service_shared_data_.mutex);
                halt_service_shared_data_.service_result = false;
                halt_service_shared_data_.result_msg = "TIMEOUT";
                halt_service_shared_data_.command_available = false;
                halt_service_shared_data_.response_available = true;
            }
            halt_service_shared_data_.cv.notify_all();
            return;
        }
    }

    halt_service_process_data_.first_pass = false;
}

void ConveyorBeltController::conveyor_service_callback(
    rightbot_interfaces::srv::ConveyorBeltCommand::Request::SharedPtr req,
    rightbot_interfaces::srv::ConveyorBeltCommand::Response::SharedPtr resp
) {
    RCLCPP_INFO(get_node()->get_logger(), "[conveyor_service_callback] Received set actuator velocity service request");
    std::string requested_actuators, requested_states;

    for (size_t i = 0; i < req->actuator_name.size(); i++) {
        requested_actuators += req->actuator_name[i];
        requested_actuators += ", ";
        requested_states += std::to_string(req->actuator_velocity[i]);
        requested_states += ", ";
    }

    RCLCPP_INFO(get_node()->get_logger(), "[conveyor_service_callback] Requested actuators: %s", requested_actuators.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "[conveyor_service_callback] Requested velocities: %s", requested_states.c_str());

    {
        std::lock_guard lk(conveyor_service_shared_data_.mutex);
        conveyor_service_shared_data_.actuator_name = req->actuator_name;
        conveyor_service_shared_data_.actuator_velocity = req->actuator_velocity;
        conveyor_service_shared_data_.response_available = false;
        conveyor_service_shared_data_.received_command = false;
        conveyor_service_shared_data_.command_available = true;
    }

    {
        std::unique_lock lk(conveyor_service_shared_data_.mutex);
        conveyor_service_shared_data_.cv.wait_for(lk, std::chrono::milliseconds((int)(params_.halt_service_timeout_sec * 1000 * 1.5)), [this](){return conveyor_service_shared_data_.response_available;});
        if (conveyor_service_shared_data_.response_available) {
            resp->success = conveyor_service_shared_data_.service_result;
            resp->msg = conveyor_service_shared_data_.result_msg;
        }
        else {
            RCLCPP_ERROR(get_node()->get_logger(), "[conveyor_service_callback] Conveyor service timed out waiting for processing response");
            resp->success = false;
            resp->msg = "TIMEOUT";
        }
        conveyor_service_shared_data_.command_available = false;
    }
}

void ConveyorBeltController::process_conveyor_service() {
    // This function should be non blocking as it is called in update loop
    {
        std::lock_guard lk(conveyor_service_shared_data_.mutex);
        conveyor_service_process_data_.command_available = conveyor_service_shared_data_.command_available;
        if (conveyor_service_shared_data_.command_available) {
            if (!conveyor_service_shared_data_.received_command) {
                conveyor_service_shared_data_.received_command = true;
                conveyor_service_process_data_.actuator_name = conveyor_service_shared_data_.actuator_name;
                conveyor_service_process_data_.actuator_velocity = conveyor_service_shared_data_.actuator_velocity;
                conveyor_service_process_data_.first_pass = true;
                halt_service_start_time_ = std::chrono::system_clock::now();
            }
        }
        else {
            return;
        }
    }

    // Check validity of command at first pass
    if (conveyor_service_process_data_.first_pass) {
        // Size check
        RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Running size check on halt service request");
        if (conveyor_service_process_data_.actuator_name.size() != conveyor_service_process_data_.actuator_velocity.size()) {
            RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Invalid service request: actuator_name and actuator_state size mismatch");
            {
                std::lock_guard lk(conveyor_service_shared_data_.mutex);
                conveyor_service_shared_data_.service_result = false;
                conveyor_service_shared_data_.result_msg = "INVALID REQUEST";
                conveyor_service_shared_data_.response_available = true;
                conveyor_service_shared_data_.command_available = false;
            }
            conveyor_service_shared_data_.cv.notify_all();
            return;
        }

        // Command type check
        // RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Running command type check on halt service request");
        // for (size_t i = 0; i < conveyor_service_process_data_.actuator_velocity.size(); i++) {
        //     if (conveyor_service_process_data_.actuator_velocity.at(i) < 0) {
        //         RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Invalid service request: Reverse velocity not allowed");
        //         {
        //             std::lock_guard lk(conveyor_service_shared_data_.mutex);
        //             conveyor_service_shared_data_.service_result = false;
        //             conveyor_service_shared_data_.result_msg = "INVALID REQUEST";
        //             conveyor_service_shared_data_.response_available = true;
        //             conveyor_service_shared_data_.command_available = false;
        //         }
        //         conveyor_service_shared_data_.cv.notify_all();
        //         return;
        //     }
        // }

        // Check the correctness of actuator names
        RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Running actuator name check on halt service request");
        for (size_t i = 0; i < conveyor_service_process_data_.actuator_velocity.size(); i++) {
            if (actuator_halt_cmd_interface_map_.count(conveyor_service_process_data_.actuator_name.at(i)) == 0) {
                RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Invalid service request: command interface for %s not found", conveyor_service_process_data_.actuator_name.at(i).c_str());
                {
                    std::lock_guard lk(conveyor_service_shared_data_.mutex);
                    conveyor_service_shared_data_.service_result = false;
                    conveyor_service_shared_data_.result_msg = "INVALID REQUEST";
                    conveyor_service_shared_data_.response_available = true;
                    conveyor_service_shared_data_.command_available = false;
                }
                conveyor_service_shared_data_.cv.notify_all();
                return;
            }
        }
        for (size_t i = 0; i < conveyor_service_process_data_.actuator_velocity.size(); i++) {
            if (actuator_halt_state_interface_map_.count(conveyor_service_process_data_.actuator_name.at(i)) == 0) {
                RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Invalid service request: command interface for %s not found", conveyor_service_process_data_.actuator_name.at(i).c_str());
                {
                    std::lock_guard lk(conveyor_service_shared_data_.mutex);
                    conveyor_service_shared_data_.service_result = false;
                    conveyor_service_shared_data_.result_msg = "INVALID REQUEST";
                    conveyor_service_shared_data_.response_available = true;
                    conveyor_service_shared_data_.command_available = false;
                }
                conveyor_service_shared_data_.cv.notify_all();
                return;
            }
        }
    }

    // Send command at first pass
    if (conveyor_service_process_data_.first_pass) {
        RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Processing halt service request");
        for (size_t i = 0; i < conveyor_service_process_data_.actuator_name.size(); i++) {
            loaned_command_interfaces_.at(
                actuator_velocity_interface_map_.at(conveyor_service_process_data_.actuator_name.at(i))
            ).get().set_value(conveyor_service_process_data_.actuator_velocity.at(i));
            RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Sending %f to %s",
                conveyor_service_process_data_.actuator_velocity.at(i),
                conveyor_service_process_data_.actuator_name.at(i).c_str()
            );
        }
    }

    // Check if state changed
    {
        conveyor_service_process_data_.state_changed = true;
        // for (size_t i = 0; i < halt_service_process_data_.actuator_name.size(); i++) {
        //     halt_service_process_data_.curr_functional_state = (int)loaned_state_interfaces_.at(actuator_halt_state_interface_map_.at(halt_service_process_data_.actuator_name.at(i))).get().get_value();
        //     if(
        //         halt_service_process_data_.curr_functional_state
        //         != cmd_to_int_map_.at(halt_service_process_data_.actuator_state.at(i))
        //     ) {
        //         halt_service_process_data_.state_changed = halt_service_process_data_.state_changed && false;
        //     }
        // }
        if(conveyor_service_process_data_.state_changed) {
            RCLCPP_INFO(get_node()->get_logger(), "[process_halt_service] Actuator velocity states have been set successfully");
            {
                std::lock_guard lk(conveyor_service_shared_data_.mutex);
                conveyor_service_shared_data_.service_result = true;
                conveyor_service_shared_data_.result_msg = "SUCCESS";
                conveyor_service_shared_data_.command_available = false;
                conveyor_service_shared_data_.response_available = true;
            }
            conveyor_service_shared_data_.cv.notify_all();
            return;
        }
    }

    // Check if timeout
    // {
    //     if (
    //         std::chrono::duration_cast<std::chrono::milliseconds>(
    //             std::chrono::system_clock::now() - halt_service_start_time_
    //         ).count() > (int)(params_.halt_service_timeout_sec * 1000)
    //     ) {
    //         RCLCPP_ERROR(get_node()->get_logger(), "[process_halt_service] Timeout waiting for state to change");
    //         {
    //             std::lock_guard lk(halt_service_shared_data_.mutex);
    //             halt_service_shared_data_.service_result = false;
    //             halt_service_shared_data_.result_msg = "TIMEOUT";
    //             halt_service_shared_data_.command_available = false;
    //             halt_service_shared_data_.response_available = true;
    //         }
    //         halt_service_shared_data_.cv.notify_all();
    //         return;
    //     }
    // }

    conveyor_service_process_data_.first_pass = false;
}

controller_interface::CallbackReturn ConveyorBeltController::on_deactivate(
    const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_node()->get_logger(), "ConveyorBeltController::on_deactivate()");
    return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace conveyor_belt_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  conveyor_belt_controller::ConveyorBeltController, controller_interface::ControllerInterface)