#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>

using namespace std::chrono_literals;

class InletTracker : public rclcpp::Node
{
public:
  InletTracker() : Node("inlet_tracker")
  {
    subscription_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
      "/gazebo/link_states", 10,
      [this](const gazebo_msgs::msg::LinkStates::SharedPtr msg) {
        this->link_states_callback(msg);
      });
    
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/inlet_pose", 10);
    
    RCLCPP_INFO(this->get_logger(), "Inlet tracker started");
  }

private:
  void link_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
  {
    static bool first_time = true;
    bool found = false;
    
    if (first_time) {
      RCLCPP_INFO(this->get_logger(), "=== AVAILABLE GAZEBO LINKS ===");
      for (size_t i = 0; i < msg->name.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Link %zu: %s", i, msg->name[i].c_str());
      }
      RCLCPP_INFO(this->get_logger(), "=== END OF LINKS ===");
      first_time = false;
    }
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "inlet") {
        found = true;
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "world";
        pose_msg.pose = msg->pose[i];
        
        pose_publisher_->publish(pose_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Inlet Position: x=%.3f, y=%.3f, z=%.3f",
          msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z);
        break;
      }
    }
    
    if (!found) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "Inlet link not found.");
    }
  }

  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InletTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}