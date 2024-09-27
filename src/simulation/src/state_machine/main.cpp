#include "state_machine/StateMachine.h"
#include "LocobotControl.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <string>
#include <iostream>
#include <thread>

std::string state_to_string(States state)
{
    switch (state)
    {
    case States::ERROR:
        return "ERROR";
    case States::IDLE:
        return "IDLE";
    case States::CLEAR:
        return "CLEAR";
    case States::SECURE_ARM:
        return "SECURE_ARM";
    case States::WAIT_ARM_SECURING:
        return "WAIT_ARM_SECURING";
    case States::SEND_NAV_GOAL:
        return "SEND_NAV_GOAL";
    case States::WAIT_NAVIGATION:
        return "WAIT_NAVIGATION";
    case States::NAVIGATION_COMPLETED:
        return "NAVIGATION_COMPLETED";
    case States::WAIT_ARM_EXTENDING:
        return "WAIT_ARM_EXTENDING";
    case States::ARM_EXTENDED:
        return "ARM_EXTENDED";
    case States::WAIT_GRIPPER_OPENING:
        return "WAIT_GRIPPER_OPENING";
    case States::GRIPPER_OPENED:
        return "GRIPPER_OPENED";
    case States::GRIPPER_CLOSING:
        return "GRIPPER_CLOSING";
    case States::WAIT_GRIPPER_CLOSING:
        return "WAIT_GRIPPER_CLOSING";
    case States::GRIPPER_CLOSED:
        return "GRIPPER_CLOSED";
    case States::WAIT_ARM_RETRACTING:
        return "WAIT_ARM_RETRACTING";
    case States::COMPLETED:
        return "COMPLETED";
    case States::ABORTING:
        return "ABORTING";
    case States::ABORTED:
        return "ABORTED";
    default:
        return "UNKNOWN";
    }
}

std::string result_to_string(Result result)
{
    switch (result)
    {
    case Result::SUCCESS:
        return "SUCCESS";
    case Result::FAILURE:
        return "FAILURE";
    case Result::RUNNING:
        return "RUNNING";
    case Result::ABORTING:
        return "ABORTING";
    case Result::ABORTED:
        return "ABORTED";
    case Result::INITIALIZED:
        return "INITIALIZED";
    default:
        return "UNKNOWN";
    }
}

int main(int argc, char * argv[])
{
    // Initialize rclcpp
    rclcpp::init(argc, argv);

    //Component manager
    // Create the component manager
    auto component_manager = std::make_shared<rclcpp_components::ComponentManager>(rclcpp::NodeOptions());
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create the StateMachine object
    machine->configure(locobot);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    executor->add_node(machine);
    executor->add_node(locobot);
    executor->add_node(component_manager);

    // Load the StateMachine component
    auto load_component_future = component_manager->load_component("StateMachine", "simulation::StateMachine");

    std::thread spin_thread = std::thread([executor]() {
        while (rclcpp::ok())
        {
            executor->spin();
        }
        rclcpp::shutdown();
    });

    //spin_thread.detach(); // Detach the thread
    

    
    std::cout << "State machine after initializing: " << state_to_string(machine->getMachineState()) << std::endl;
     
    // Main loop

    while (machine->getMachineState() != States::COMPLETED && 
            machine->getMachineState() != States::ERROR && 
            machine->getMachineState() != States::ABORTED)
    {
        machine->nextState();
        std::cout << "State machine after nextState: " << state_to_string(machine->getMachineState()) << std::endl;
    }

    // Print the final state of the machine
    std::cout << "Final state of the machine: " << state_to_string(machine->getMachineState()) << std::endl;
    
 
    // Shutdown rclcpp
    rclcpp::shutdown();
    return 0;
}