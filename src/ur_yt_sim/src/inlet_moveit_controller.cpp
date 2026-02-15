#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <atomic>
#include <cmath>
#include <algorithm>

// Define a simple State Machine
enum class State {
  IDLE,
  APPROACHING, // Moving to "hover" point
  INSERTING,   // Moving into the socket
  RETRACTING   // Pulling out
};

class InletMoveitController : public rclcpp::Node
{
public:
  InletMoveitController()
  : Node("inlet_moveit_controller",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    current_state_(State::IDLE)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/inlet/pose", 10,
      std::bind(&InletMoveitController::targetCallback, this, std::placeholders::_1),
      opts);

    last_plan_time_ = now();
    RCLCPP_INFO(get_logger(), "Inlet Controller Ready. Waiting for target...");
  }

  void initMoveIt()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "manipulator");

    move_group_->setPlanningTime(5.0);
    move_group_->setEndEffectorLink("tool0");
    move_group_->setMaxVelocityScalingFactor(0.5); 
    move_group_->setMaxAccelerationScalingFactor(0.5);
    
    planning_frame_ = move_group_->getPlanningFrame();
  }

private:
  void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Don't interrupt if we are already moving
    if (current_state_ != State::IDLE) return;

    // Rate limit (don't replan every 30ms)
    if ((now() - last_plan_time_).seconds() < 2.0) return;

    RCLCPP_INFO(get_logger(), "Target received. Processing...");

    // 1. Transform Target to Base Frame
    geometry_msgs::msg::PoseStamped inlet_pose;
    try {
      inlet_pose = tf_buffer_->transform(
        *msg, planning_frame_, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF Error: %s", ex.what());
      return;
    }

    // 2. Define "Pre-Approach" (Hover Point)
    // Back off 25cm along the inlet's Z-axis (normal)
    tf2::Transform inlet_tf;
    tf2::fromMsg(inlet_pose.pose, inlet_tf);
    
    // Assuming Z is pointing INTO the socket (as discussed in previous node)
    // We want to move -Z (backwards) relative to the target
    tf2::Vector3 approach_vec = inlet_tf.getBasis() * tf2::Vector3(0, 0, -0.25);

    geometry_msgs::msg::Pose pre_pose = inlet_pose.pose;
    pre_pose.position.x += approach_vec.x();
    pre_pose.position.y += approach_vec.y();
    pre_pose.position.z += approach_vec.z();

    // 3. EXECUTE PHASE 1: Go to Hover Point (OMPL)
    current_state_ = State::APPROACHING;
    
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(pre_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(get_logger(), "Moving to Pre-Approach...");
      move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(get_logger(), "Planning failed");
      current_state_ = State::IDLE;
      return;
    }

    // 4. EXECUTE PHASE 2: Cartesian Insertion
    // Now we are at the hover point. We need to drive straight in.
    current_state_ = State::INSERTING;
    RCLCPP_INFO(get_logger(), "Aligning for Insertion...");

    // Slow down for precision!
    move_group_->setMaxVelocityScalingFactor(0.05); // 5% speed
    move_group_->setMaxAccelerationScalingFactor(0.05);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(inlet_pose.pose); // Linear path from current to target

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_->computeCartesianPath(
        waypoints,
        0.005,  // eef_step (5mm resolution)
        0.0,    // jump_threshold (0 = disable check, prone to failures but aggressive)
        traj
    );

    if (fraction > 0.90) {
      RCLCPP_INFO(get_logger(), "Cartesian Path computed (%.2f%%). INSERTING.", fraction * 100);
      move_group_->execute(traj);
      RCLCPP_INFO(get_logger(), "Insertion Complete.");
    } else {
      RCLCPP_WARN(get_logger(), "Cartesian path failed (Only %.2f%% feasible). Aborting.", fraction * 100);
    }

    // Reset speed and state
    move_group_->setMaxVelocityScalingFactor(0.5);
    last_plan_time_ = now();
    current_state_ = State::IDLE;
  }

  // State
  State current_state_;
  rclcpp::Time last_plan_time_;

  // ROS & MoveIt
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string planning_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InletMoveitController>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  node->initMoveIt();
  exec.spin();
  rclcpp::shutdown();
  return 0;
}