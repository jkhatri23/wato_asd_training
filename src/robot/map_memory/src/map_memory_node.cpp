#include <chrono>
#include <memory>
#include <cmath>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(robot::MapMemoryCore(this->get_logger())),
      robot_x_(0.0), robot_y_(0.0), robot_yaw_(0.0),
      last_update_x_(0.0), last_update_y_(0.0),
      costmap_received_(false), should_update_map_(false)
{
  // Initialize global map: 2000x2000 at 0.1m = 200m x 200m
  map_memory_.initializeGlobalMap(2000, 2000, 0.1);

  // Subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Timer — check every 1 second
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapMemoryNode::timerCallback, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Extract yaw from quaternion
  auto &q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Check if robot has moved enough since last update
  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance >= DISTANCE_THRESHOLD)
  {
    should_update_map_ = true;
  }
}

void MapMemoryNode::timerCallback()
{
  if (should_update_map_ && costmap_received_)
  {
    // Integrate the latest costmap into the global map
    map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);

    // Publish the updated global map
    auto global_map = map_memory_.getGlobalMap();
    global_map.header.stamp = this->now();
    map_pub_->publish(global_map);

    // Reset state
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    should_update_map_ = false;

    RCLCPP_INFO(this->get_logger(), "Map updated at (%.2f, %.2f)", robot_x_, robot_y_);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}