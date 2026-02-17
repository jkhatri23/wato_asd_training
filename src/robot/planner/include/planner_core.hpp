#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace robot
{

  // ------------------- Supporting Structures -------------------

  struct CellIndex
  {
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const
    {
      return (x == other.x && y == other.y);
    }

    bool operator!=(const CellIndex &other) const
    {
      return (x != other.x || y != other.y);
    }
  };

  struct CellIndexHash
  {
    std::size_t operator()(const CellIndex &idx) const
    {
      return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
  };

  struct AStarNode
  {
    CellIndex index;
    double f_score;

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
  };

  struct CompareF
  {
    bool operator()(const AStarNode &a, const AStarNode &b)
    {
      return a.f_score > b.f_score;
    }
  };

  // ------------------- Planner Core -------------------

  class PlannerCore
  {
  public:
    explicit PlannerCore(const rclcpp::Logger &logger);

    // Run A* on the occupancy grid from start to goal (in world coordinates).
    // Returns a nav_msgs::msg::Path with waypoints in the "map" frame.
    nav_msgs::msg::Path planPath(
        const nav_msgs::msg::OccupancyGrid &map,
        double start_x, double start_y,
        double goal_x, double goal_y);

  private:
    rclcpp::Logger logger_;

    // Convert world coordinates to grid cell indices
    CellIndex worldToGrid(double wx, double wy,
                          const nav_msgs::msg::OccupancyGrid &map) const;

    // Convert grid cell indices to world coordinates (center of cell)
    std::pair<double, double> gridToWorld(const CellIndex &cell,
                                          const nav_msgs::msg::OccupancyGrid &map) const;

    // Check if a cell is within bounds and traversable
    bool isValid(const CellIndex &cell,
                 const nav_msgs::msg::OccupancyGrid &map) const;

    // Heuristic: Euclidean distance in grid cells
    double heuristic(const CellIndex &a, const CellIndex &b) const;

    // Cost threshold: cells with occupancy above this are considered obstacles
    static constexpr int8_t OBSTACLE_THRESHOLD = 50;
  };

}

#endif