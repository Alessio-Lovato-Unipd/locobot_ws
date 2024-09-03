#include "LocobotControl.cpp"


int main(int argc, char * argv[])
{
    // Send the robot's arm to sleep position
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<LocobotControl>();
    control_node->SetArmPose(ArmPose::SLEEP);
    rclcpp::spin_some(control_node);
    rclcpp::shutdown();
  return 0;
}