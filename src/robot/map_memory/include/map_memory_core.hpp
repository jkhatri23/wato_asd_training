#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot
{

  class MapMemoryCore
  {
  public:
    explicit MapMemoryCore(const rclcpp::Logger &logger);

    // Initialize the global map with given size and resolution
    void initializeGlobalMap(int width, int height, double resolution);

    // Integrate a local costmap into the global map given the robot's pose
    void integrateCostmap(
        const nav_msgs::msg::OccupancyGrid &costmap,
        double robot_x, double robot_y, double robot_yaw);

    // Get the global map as an OccupancyGrid message
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
    bool initialized_;
  };

}

#endif
