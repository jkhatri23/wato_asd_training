#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>

namespace robot
{

  class CostmapCore
  {
  public:
    explicit CostmapCore(const rclcpp::Logger &logger);

    // Initialize/reset the grid to all zeros
    void initializeCostmap(int width, int height, double resolution);

    // Convert a polar reading (range, angle) to grid coords and mark it
    void markObstacle(int x, int y);

    // Spread cost around obstacle cells
    void inflateObstacles(double inflation_radius);

    // Return the grid as a flattened 1D vector (row-major) for OccupancyGrid.data
    std::vector<int8_t> getData() const;

    // Getters for publishing the OccupancyGrid metadata
    int getWidth() const;
    int getHeight() const;
    double getResolution() const;

  private:
    rclcpp::Logger logger_;
    std::vector<std::vector<int8_t>> grid_;
    int width_;
    int height_;
    double resolution_;
  };

}

#endif