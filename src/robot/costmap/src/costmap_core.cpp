#include "costmap_core.hpp"

namespace robot
{

    CostmapCore::CostmapCore(const rclcpp::Logger &logger)
        : logger_(logger), width_(0), height_(0), resolution_(0.0) {}

    void CostmapCore::initializeCostmap(int width, int height, double resolution)
    {
        width_ = width;
        height_ = height;
        resolution_ = resolution;
        // Reset grid to all zeros (free space)
        grid_.assign(height_, std::vector<int8_t>(width_, 0));
    }

    void CostmapCore::markObstacle(int x, int y)
    {
        // Bounds check, then set to 100 (occupied)
        if (x >= 0 && x < width_ && y >= 0 && y < height_)
        {
            grid_[y][x] = 100;
        }
    }

    void CostmapCore::inflateObstacles(double inflation_radius)
    {
        int radius_cells = static_cast<int>(inflation_radius / resolution_);

        // Make a copy so we don't inflate based on already-inflated values
        auto original = grid_;

        for (int row = 0; row < height_; ++row)
        {
            for (int col = 0; col < width_; ++col)
            {
                if (original[row][col] != 100)
                    continue; // only inflate around obstacles

                // Iterate over surrounding cells within the inflation radius
                for (int dy = -radius_cells; dy <= radius_cells; ++dy)
                {
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
                    {
                        int nx = col + dx;
                        int ny = row + dy;
                        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
                            continue;

                        double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                        if (distance > inflation_radius)
                            continue;

                        int8_t cost = static_cast<int8_t>(100.0 * (1.0 - distance / inflation_radius));
                        // Only assign if higher than current value
                        if (cost > grid_[ny][nx])
                        {
                            grid_[ny][nx] = cost;
                        }
                    }
                }
            }
        }
    }

    std::vector<int8_t> CostmapCore::getData() const
    {
        // Flatten the 2D grid into 1D (row-major)
        std::vector<int8_t> data;
        data.reserve(width_ * height_);
        for (const auto &row : grid_)
        {
            data.insert(data.end(), row.begin(), row.end());
        }
        return data;
    }

    int CostmapCore::getWidth() const { return width_; }
    int CostmapCore::getHeight() const { return height_; }
    double CostmapCore::getResolution() const { return resolution_; }

}