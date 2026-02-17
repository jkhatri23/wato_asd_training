#include <chrono>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control"),
      control_(robot::ControlCore(this->get_logger()))
{
  // Subscribers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer at 10 Hz (100ms)
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControlNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "ControlNode initialized");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_odom_ = msg;
}

void ControlNode::controlLoop()
{
  // Skip if no path or odometry data available
  if (!current_path_ || !robot_odom_)
  {
    return;
  }

  // Skip if path is empty
  if (current_path_->poses.empty())
  {
    return;
  }

  // Extract robot position and yaw
  double robot_x = robot_odom_->pose.pose.position.x;
  double robot_y = robot_odom_->pose.pose.position.y;
  double robot_yaw = control_.extractYaw(robot_odom_->pose.pose.orientation);

  // Check if we've reached the final goal
  const auto &goal_pose = current_path_->poses.back().pose.position;
  if (control_.computeDistance(robot_x, robot_y, goal_pose.x, goal_pose.y) < control_.getGoalTolerance())
  {
    // Stop the robot
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    RCLCPP_INFO(this->get_logger(), "Goal reached, stopping.");
    return;
  }

  // Find the lookahead point
  auto lookahead_point = control_.findLookaheadPoint(
      *current_path_, robot_x, robot_y);

  if (!lookahead_point)
  {
    // No valid lookahead point; stop
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  // Compute and publish velocity command
  geometry_msgs::msg::Twist cmd_vel = control_.computeVelocity(
      *lookahead_point, robot_x, robot_y, robot_yaw);

  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}