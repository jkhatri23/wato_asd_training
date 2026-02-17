#include "planner_core.hpp"

namespace robot
{

    PlannerCore::PlannerCore(const rclcpp::Logger &logger)
        : logger_(logger) {}

    CellIndex PlannerCore::worldToGrid(double wx, double wy,
                                       const nav_msgs::msg::OccupancyGrid &map) const
    {
        double origin_x = map.info.origin.position.x;
        double origin_y = map.info.origin.position.y;
        double resolution = map.info.resolution;

        int gx = static_cast<int>((wx - origin_x) / resolution);
        int gy = static_cast<int>((wy - origin_y) / resolution);
        return CellIndex(gx, gy);
    }

    std::pair<double, double> PlannerCore::gridToWorld(const CellIndex &cell,
                                                       const nav_msgs::msg::OccupancyGrid &map) const
    {
        double origin_x = map.info.origin.position.x;
        double origin_y = map.info.origin.position.y;
        double resolution = map.info.resolution;

        double wx = origin_x + (cell.x + 0.5) * resolution;
        double wy = origin_y + (cell.y + 0.5) * resolution;
        return {wx, wy};
    }

    bool PlannerCore::isValid(const CellIndex &cell,
                              const nav_msgs::msg::OccupancyGrid &map) const
    {
        int width = static_cast<int>(map.info.width);
        int height = static_cast<int>(map.info.height);

        if (cell.x < 0 || cell.x >= width || cell.y < 0 || cell.y >= height)
            return false;

        int idx = cell.y * width + cell.x;
        int8_t value = map.data[idx];

        // Treat unknown (-1) as traversable, obstacles (>= threshold) as blocked
        if (value >= OBSTACLE_THRESHOLD)
            return false;

        return true;
    }

    double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) const
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    nav_msgs::msg::Path PlannerCore::planPath(
        const nav_msgs::msg::OccupancyGrid &map,
        double start_x, double start_y,
        double goal_x, double goal_y)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "sim_world";

        CellIndex start = worldToGrid(start_x, start_y, map);
        CellIndex goal = worldToGrid(goal_x, goal_y, map);

        // Validate start and goal
        if (!isValid(start, map))
        {
            RCLCPP_WARN(logger_, "Start cell (%d, %d) is invalid or in obstacle", start.x, start.y);
            return path;
        }
        if (!isValid(goal, map))
        {
            RCLCPP_WARN(logger_, "Goal cell (%d, %d) is invalid or in obstacle", goal.x, goal.y);
            return path;
        }

        // A* algorithm
        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
        std::unordered_map<CellIndex, double, CellIndexHash> g_score;
        std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

        g_score[start] = 0.0;
        open_list.push(AStarNode(start, heuristic(start, goal)));

        // 8-connected neighbors: dx, dy pairs
        const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const double move_cost[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

        bool found = false;

        while (!open_list.empty())
        {
            AStarNode current = open_list.top();
            open_list.pop();

            if (current.index == goal)
            {
                found = true;
                break;
            }

            // Skip if we've already found a better path to this node
            if (g_score.count(current.index) &&
                current.f_score > g_score[current.index] + heuristic(current.index, goal) + 1e-6)
            {
                continue;
            }

            for (int i = 0; i < 8; ++i)
            {
                CellIndex neighbor(current.index.x + dx[i], current.index.y + dy[i]);

                if (!isValid(neighbor, map))
                    continue;

                double tentative_g = g_score[current.index] + move_cost[i];

                if (!g_score.count(neighbor) || tentative_g < g_score[neighbor])
                {
                    g_score[neighbor] = tentative_g;
                    double f = tentative_g + heuristic(neighbor, goal);
                    open_list.push(AStarNode(neighbor, f));
                    came_from[neighbor] = current.index;
                }
            }
        }

        if (!found)
        {
            RCLCPP_WARN(logger_, "A* could not find a path from (%d,%d) to (%d,%d)",
                        start.x, start.y, goal.x, goal.y);
            return path;
        }

        // Reconstruct path from goal to start
        std::vector<CellIndex> cell_path;
        CellIndex current = goal;
        while (current != start)
        {
            cell_path.push_back(current);
            current = came_from[current];
        }
        cell_path.push_back(start);

        // Reverse so path goes from start to goal
        std::reverse(cell_path.begin(), cell_path.end());

        // Convert cell indices to PoseStamped waypoints
        for (const auto &cell : cell_path)
        {
            auto [wx, wy] = gridToWorld(cell, map);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "sim_world";
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        RCLCPP_INFO(logger_, "A* path found with %zu waypoints", path.poses.size());
        return path;
    }

} // namespace robot