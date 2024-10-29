#include "LocobotControl.cpp"


int main(int argc, char * argv[])
{
    // Send the robot's arm to sleep position
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<LocobotControl>();
    control_node->SetArmPose(ArmPose::SLEEP);
    rclcpp::spin_some(control_node);
    // Wait for the arm to stop moving before shutting down the node
    while (control_node->isArmMoving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    rclcpp::shutdown();
  return 0;
}