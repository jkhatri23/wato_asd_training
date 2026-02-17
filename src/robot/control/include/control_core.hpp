#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cmath>
#include <optional>

namespace robot
{

  class ControlCore
  {
  public:
    explicit ControlCore(const rclcpp::Logger &logger);

    // Find the lookahead point on the path closest to lookahead_distance_ ahead
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path &path,
        double robot_x, double robot_y);

    // Compute the Twist command to steer toward the target point
    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::PoseStamped &target,
        double robot_x, double robot_y, double robot_yaw);

    // Euclidean distance between two 2D points
    double computeDistance(double x1, double y1, double x2, double y2) const;

    // Convert quaternion to yaw angle (radians)
    double extractYaw(const geometry_msgs::msg::Quaternion &quat) const;

    // Accessor for goal tolerance
    double getGoalTolerance() const { return goal_tolerance_; }

  private:
    rclcpp::Logger logger_;

    // Tuning parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
  };

} // namespace robot

#endif