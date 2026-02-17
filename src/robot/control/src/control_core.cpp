#include "control_core.hpp"

namespace robot
{

  ControlCore::ControlCore(const rclcpp::Logger &logger)
      : logger_(logger),
        lookahead_distance_(1.0),
        goal_tolerance_(0.1),
        linear_speed_(0.5)
  {
  }

  double ControlCore::computeDistance(double x1, double y1, double x2, double y2) const
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }

  double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) const
  {
    // Standard quaternion-to-yaw extraction (only valid when roll/pitch ~ 0)
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
      const nav_msgs::msg::Path &path,
      double robot_x, double robot_y)
  {
    // Walk through the path and find the first point that is at least
    // lookahead_distance_ away from the robot
    for (const auto &pose_stamped : path.poses)
    {
      double dist = computeDistance(
          robot_x, robot_y,
          pose_stamped.pose.position.x,
          pose_stamped.pose.position.y);

      if (dist >= lookahead_distance_)
      {
        return pose_stamped;
      }
    }

    // If no point is far enough, use the last point on the path
    // (this handles the "approaching goal" case)
    if (!path.poses.empty())
    {
      return path.poses.back();
    }

    return std::nullopt;
  }

  geometry_msgs::msg::Twist ControlCore::computeVelocity(
      const geometry_msgs::msg::PoseStamped &target,
      double robot_x, double robot_y, double robot_yaw)
  {
    geometry_msgs::msg::Twist cmd_vel;

    // Vector from robot to lookahead point
    double dx = target.pose.position.x - robot_x;
    double dy = target.pose.position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < 1e-6)
    {
      // Already at target, don't divide by zero
      return cmd_vel;
    }

    // Transform the lookahead point into the robot's local frame
    // alpha = angle from the robot's heading to the lookahead point
    double angle_to_target = std::atan2(dy, dx);
    double alpha = angle_to_target - robot_yaw;

    // Normalize alpha to [-pi, pi]
    while (alpha > M_PI)
      alpha -= 2.0 * M_PI;
    while (alpha < -M_PI)
      alpha += 2.0 * M_PI;

    // Pure pursuit curvature: kappa = 2 * sin(alpha) / lookahead_distance
    // Angular velocity: omega = linear_speed * kappa
    double curvature = 2.0 * std::sin(alpha) / distance;

    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = linear_speed_ * curvature;

    return cmd_vel;
  }

} // namespace robot