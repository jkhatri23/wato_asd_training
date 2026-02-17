#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // 1. Initialize costmap (pick reasonable size, e.g. 200x200 at 0.1m resolution = 20m x 20m)
  int grid_size = 200;
  double resolution = 0.1;
  costmap_.initializeCostmap(grid_size, grid_size, resolution);

  // 2. Convert each laser reading to grid coordinates and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    double range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max)
      continue;

    double angle = scan->angle_min + i * scan->angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Convert to grid indices (origin at center of grid)
    int x_grid = static_cast<int>(x / resolution) + grid_size / 2;
    int y_grid = static_cast<int>(y / resolution) + grid_size / 2;

    costmap_.markObstacle(x_grid, y_grid);
  }

  // 3. Inflate obstacles
  costmap_.inflateObstacles(0.5); // 0.5m inflation radius

  // 4. Publish
  auto msg = nav_msgs::msg::OccupancyGrid();
  msg.header.stamp = this->now();
  msg.header.frame_id = "sim_world";
  msg.info.resolution = costmap_.getResolution();
  msg.info.width = costmap_.getWidth();
  msg.info.height = costmap_.getHeight();
  msg.info.origin.position.x = -(grid_size / 2.0) * resolution;
  msg.info.origin.position.y = -(grid_size / 2.0) * resolution;
  msg.data = costmap_.getData();

  costmap_pub_->publish(msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}