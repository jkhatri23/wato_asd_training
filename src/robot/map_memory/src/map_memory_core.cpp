#include "map_memory_core.hpp"

namespace robot
{

  MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger)
      : logger_(logger), initialized_(false) {}

  void MapMemoryCore::initializeGlobalMap(int width, int height, double resolution)
  {
    global_map_.info.width = width;
    global_map_.info.height = height;
    global_map_.info.resolution = resolution;
    global_map_.header.frame_id = "sim_world";

    // Center the map origin so (0,0) world is at the center of the grid
    global_map_.info.origin.position.x = -(width / 2.0) * resolution;
    global_map_.info.origin.position.y = -(height / 2.0) * resolution;
    global_map_.info.origin.orientation.w = 1.0;

    // Initialize all cells to -1 (unknown)
    global_map_.data.assign(width * height, -1);
    initialized_ = true;

    RCLCPP_INFO(logger_, "Global map initialized: %dx%d at %.2fm resolution", width, height, resolution);
  }

  void MapMemoryCore::integrateCostmap(
      const nav_msgs::msg::OccupancyGrid &costmap,
      double robot_x, double robot_y, double robot_yaw)
  {
    if (!initialized_)
    {
      RCLCPP_WARN(logger_, "Global map not initialized, skipping integration");
      return;
    }

    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);

    double local_origin_x = costmap.info.origin.position.x;
    double local_origin_y = costmap.info.origin.position.y;
    double local_res = costmap.info.resolution;

    double global_origin_x = global_map_.info.origin.position.x;
    double global_origin_y = global_map_.info.origin.position.y;
    double global_res = global_map_.info.resolution;

    int global_w = static_cast<int>(global_map_.info.width);
    int global_h = static_cast<int>(global_map_.info.height);

    for (unsigned int ly = 0; ly < costmap.info.height; ++ly)
    {
      for (unsigned int lx = 0; lx < costmap.info.width; ++lx)
      {
        int idx = ly * costmap.info.width + lx;
        int8_t cell_value = costmap.data[idx];

        // Skip unknown cells (-1) — retain previous global map value
        if (cell_value < 0)
        {
          continue;
        }

        // Local costmap position (in the robot's frame)
        double local_x = local_origin_x + (lx + 0.5) * local_res;
        double local_y = local_origin_y + (ly + 0.5) * local_res;

        // Transform to global frame using robot pose
        double world_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
        double world_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

        // Convert to global grid indices
        int gx = static_cast<int>((world_x - global_origin_x) / global_res);
        int gy = static_cast<int>((world_y - global_origin_y) / global_res);

        // Bounds check
        if (gx < 0 || gx >= global_w || gy < 0 || gy >= global_h)
        {
          continue;
        }

        int global_idx = gy * global_w + gx;
        global_map_.data[global_idx] = cell_value;
      }
    }
  }

  nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const
  {
    return global_map_;
  }

}