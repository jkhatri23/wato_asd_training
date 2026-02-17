#include <chrono>
#include <memory>
#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner"),
      planner_(robot::PlannerCore(this->get_logger())),
      state_(State::WAITING_FOR_GOAL),
      goal_received_(false),
      map_received_(false),
      goal_tolerance_(0.5),
      timer_period_ms_(500.0)
{
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(timer_period_ms_)),
      std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "PlannerNode initialized, waiting for goal...");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  map_received_ = true;

  // Replan whenever the map updates and we have an active goal
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)",
              goal_.point.x, goal_.point.y);

  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    if (goalReached())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      goal_received_ = false;
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "Still navigating to goal...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() const
{
  if (!goal_received_)
    return false;

  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

void PlannerNode::planPath()
{
  if (!goal_received_ || !map_received_)
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: missing map or goal");
    return;
  }

  if (current_map_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: map data is empty");
    return;
  }

  nav_msgs::msg::Path path = planner_.planPath(
      current_map_,
      robot_pose_.position.x, robot_pose_.position.y,
      goal_.point.x, goal_.point.y);

  path.header.stamp = this->get_clock()->now();
  path_pub_->publish(path);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}