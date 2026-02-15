#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <cmath>
#include <algorithm>

class InletPreinsertController : public rclcpp::Node
{
public:
  InletPreinsertController()
  : Node("preinsert_controller",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Params (tune these)
    planning_group_   = this->declare_parameter<std::string>("planning_group", "manipulator");
    eef_link_         = this->declare_parameter<std::string>("eef_link", "evcharger_tcp"); // or "tool0"
    target_topic_     = this->declare_parameter<std::string>("target_topic", "/inlet/preinsert_pose");
    preinsert_topic_  = this->declare_parameter<std::string>("preinsert_topic", "/inlet/preinsertion_pose");

    stand_off_        = this->declare_parameter<double>("stand_off", 0.20);     // meters away from inlet
    below_offset_     = this->declare_parameter<double>("below_offset", 0.07);  // meters BELOW inlet (negative Z)
    min_replan_sec_   = this->declare_parameter<double>("min_replan_sec", 1.5);

    max_vel_          = this->declare_parameter<double>("max_vel", 0.4);
    max_acc_          = this->declare_parameter<double>("max_acc", 0.4);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_, 10,
      std::bind(&InletPreinsertController::targetCallback, this, std::placeholders::_1),
      opts);

    pub_pre_ = create_publisher<geometry_msgs::msg::PoseStamped>(preinsert_topic_, 10);

    last_plan_time_ = now();

    RCLCPP_INFO(get_logger(), "Pre-insertion controller ready. Waiting for /inlet/preinsert_pose ...");
  }

  void initMoveIt()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);

    move_group_->setPlanningTime(5.0);
    move_group_->setEndEffectorLink(eef_link_);
    move_group_->setMaxVelocityScalingFactor(max_vel_);
    move_group_->setMaxAccelerationScalingFactor(max_acc_);

    planning_frame_ = move_group_->getPlanningFrame();

    RCLCPP_INFO(get_logger(),
      "MoveIt initialized. planning_frame=%s, eef_link=%s",
      planning_frame_.c_str(), eef_link_.c_str());
  }

private:
  void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // rate limit
    if ((now() - last_plan_time_).seconds() < min_replan_sec_) return;

    if (!move_group_) {
      RCLCPP_WARN(get_logger(), "MoveGroup not initialized yet.");
      return;
    }

    // 1) Transform inlet pose into planning frame
    geometry_msgs::msg::PoseStamped inlet_pose;
    try {
      inlet_pose = tf_buffer_->transform(*msg, planning_frame_, tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
      return;
    }

    // 2) Current TCP pose (in planning frame)
    geometry_msgs::msg::PoseStamped tcp_now = move_group_->getCurrentPose(eef_link_);

    // 3) Compute direction from TCP -> inlet (then step back by stand_off_)
    const double dx = inlet_pose.pose.position.x - tcp_now.pose.position.x;
    const double dy = inlet_pose.pose.position.y - tcp_now.pose.position.y;
    const double dz = inlet_pose.pose.position.z - tcp_now.pose.position.z;

    double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (norm < 1e-6) {
      RCLCPP_WARN(get_logger(), "TCP is at (almost) same position as inlet. Skipping.");
      return;
    }

    const double ux = dx / norm;
    const double uy = dy / norm;
    const double uz = dz / norm;

    geometry_msgs::msg::PoseStamped pre;
    pre.header.frame_id = planning_frame_;
    pre.header.stamp = now();

    // Stand-off: move opposite of (TCP->inlet) so we stop "before" the inlet
    pre.pose.position.x = inlet_pose.pose.position.x - stand_off_ * ux;
    pre.pose.position.y = inlet_pose.pose.position.y - stand_off_ * uy;
    pre.pose.position.z = inlet_pose.pose.position.z - stand_off_ * uz;

    // From-below: drop in -Z (planning frame). This makes the arm approach from below so camera sees inlet.
    pre.pose.position.z -= below_offset_;

    // Keep current orientation (no rotations until keypoint refinement)
    pre.pose.orientation = tcp_now.pose.orientation;

    // Publish pre-insertion pose (useful for debugging / next node)
    pub_pre_->publish(pre);

    RCLCPP_INFO(get_logger(),
      "Pre-insertion target: [%.3f, %.3f, %.3f] in %s (stand_off=%.2f, below=%.2f)",
      pre.pose.position.x, pre.pose.position.y, pre.pose.position.z,
      planning_frame_.c_str(), stand_off_, below_offset_);

    // 4) Plan + execute ONLY to pre-insertion
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(pre.pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(get_logger(), "Planning to pre-insertion failed.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Executing pre-insertion...");
    move_group_->execute(plan);

    last_plan_time_ = now();
  }

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pre_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Time last_plan_time_;

  // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string planning_group_;
  std::string eef_link_;
  std::string planning_frame_;
  std::string target_topic_;
  std::string preinsert_topic_;

  double stand_off_;
  double below_offset_;
  double min_replan_sec_;
  double max_vel_;
  double max_acc_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InletPreinsertController>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  node->initMoveIt();
  exec.spin();

  rclcpp::shutdown();
  return 0;
}