#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_controller",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);
  move_group.setEndEffectorLink("evcharger_tcp");
  
  move_group.setPoseReferenceFrame("world");

  rclcpp::sleep_for(std::chrono::seconds(2));
  move_group.setStartStateToCurrentState();

  // Relax constraints
  move_group.setGoalPositionTolerance(0.005);
  move_group.setGoalOrientationTolerance(0.05);

    RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
                move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector: %s",
                move_group.getEndEffectorLink().c_str());

    geometry_msgs::msg::Pose target_pose;
 

  target_pose.position.x = -0.024;
  target_pose.position.y =  0.612;
  target_pose.position.z =  1.063;

  target_pose.orientation.x = -0.764;
  target_pose.orientation.y =  0.015;
  target_pose.orientation.z =  0.047;
  target_pose.orientation.w =  0.643;
 

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Plan successful. Executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
