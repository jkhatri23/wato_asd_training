#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node
{
public:
  MapMemoryNode();

private:
  // Callbacks
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // Core logic
  robot::MapMemoryCore map_memory_;

  // ROS constructs
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
  double last_update_x_;
  double last_update_y_;
  bool costmap_received_;
  bool should_update_map_;

  // Parameters
  static constexpr double DISTANCE_THRESHOLD = 1.5; // meters
};

#endif