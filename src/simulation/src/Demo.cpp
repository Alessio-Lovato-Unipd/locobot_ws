#include "LocobotControl.cpp"


int main(int argc, char * argv[])
{
  // Crate the MoveArm Node
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<LocobotControl>();
    auto pose = coordinates_to_pose(0, 1.5, 0,  0, 0, 0.7071068, 0.7071068, "map");
    control_node->MoveBaseTo(pose);
    control_node->SetGripper(GripperState::RELEASED);
    control_node->SetArmPose(ArmPose::SLEEP);
    rclcpp::spin_some(control_node);
    rclcpp::shutdown();
  return 0;
}