/*
Copyright 2026 Vega Innovations (Pvt) Ltd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "nova_exploration/sparse_coverage.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace nova_exploration
{
namespace
{

enum CellClass : uint8_t
{
  kFree = 0,
  kUnknown = 1,
  kOccupied = 2,
};

// Summed-area table with a one-cell zero border: sum over [x0,x1]x[y0,y1] in O(1).
class IntegralImage
{
public:
  IntegralImage(const std::vector<uint8_t> & mask, int width, int height)
  : width_(width), sums_(static_cast<std::size_t>(width + 1) * static_cast<std::size_t>(height + 1), 0)
  {
    for (int y = 0; y < height; ++y) {
      int32_t row_sum = 0;
      for (int x = 0; x < width; ++x) {
        row_sum += mask[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(x)];
        at(x + 1, y + 1) = at(x + 1, y) + row_sum;
      }
    }
  }

  [[nodiscard]] int32_t sum(int x0, int y0, int x1, int y1) const
  {
    return at(x1 + 1, y1 + 1) - at(x0, y1 + 1) - at(x1 + 1, y0) + at(x0, y0);
  }

private:
  int32_t & at(int x, int y)
  {
    return sums_[static_cast<std::size_t>(y) * static_cast<std::size_t>(width_ + 1) +
             static_cast<std::size_t>(x)];
  }

  [[nodiscard]] int32_t at(int x, int y) const
  {
    return sums_[static_cast<std::size_t>(y) * static_cast<std::size_t>(width_ + 1) +
             static_cast<std::size_t>(x)];
  }

  int width_;
  std::vector<int32_t> sums_;
};

// Labels 8-connected unknown components and keeps the ones that look like ray gaps.
std::vector<uint8_t> find_hole_cells(
  const std::vector<uint8_t> & classes,
  int width,
  int height,
  const SparseCoverageConfig & config)
{
  const std::size_t cell_count = classes.size();
  std::vector<uint8_t> holes(cell_count, 0);
  std::vector<uint8_t> visited(cell_count, 0);
  std::vector<std::size_t> component;
  std::vector<std::size_t> stack;

  for (std::size_t seed = 0; seed < cell_count; ++seed) {
    if (classes[seed] != kUnknown || visited[seed] != 0) {
      continue;
    }

    component.clear();
    stack.clear();
    stack.push_back(seed);
    visited[seed] = 1;
    bool touches_edge = false;
    std::size_t free_border = 0;
    std::size_t occupied_border = 0;

    while (!stack.empty()) {
      const std::size_t idx = stack.back();
      stack.pop_back();
      component.push_back(idx);
      const int x = static_cast<int>(idx % static_cast<std::size_t>(width));
      const int y = static_cast<int>(idx / static_cast<std::size_t>(width));
      if (x == 0 || y == 0 || x == width - 1 || y == height - 1) {
        touches_edge = true;
      }

      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const std::size_t nidx =
            static_cast<std::size_t>(ny) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(nx);
          const uint8_t neighbor_class = classes[nidx];
          if (neighbor_class == kUnknown) {
            if (visited[nidx] == 0) {
              visited[nidx] = 1;
              stack.push_back(nidx);
            }
          } else if (neighbor_class == kOccupied) {
            // 8-neighbours: a shadow often touches its obstacle only diagonally.
            ++occupied_border;
          } else {
            ++free_border;
          }
        }
      }
    }

    const std::size_t border = free_border + occupied_border;
    const bool is_hole =
      !touches_edge &&
      component.size() <= static_cast<std::size_t>(std::max(0, config.max_hole_cells)) &&
      border > 0 &&
      static_cast<double>(occupied_border) <=
      config.max_hole_occupied_border_ratio * static_cast<double>(border);
    if (is_hole) {
      for (const auto idx : component) {
        holes[idx] = 1;
      }
    }
  }

  return holes;
}

}  // namespace

std::size_t apply_sparse_coverage(
  const nav_msgs::msg::OccupancyGrid & raw_map,
  nav_msgs::msg::OccupancyGrid & decision_map,
  const std::vector<std::pair<double, double>> & checked_points,
  const SparseCoverageConfig & config)
{
  const int width = static_cast<int>(raw_map.info.width);
  const int height = static_cast<int>(raw_map.info.height);
  const double resolution = raw_map.info.resolution;
  const std::size_t cell_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (
    width <= 0 || height <= 0 || resolution <= 0.0 ||
    raw_map.data.size() != cell_count ||
    decision_map.data.size() != cell_count ||
    decision_map.info.width != raw_map.info.width ||
    decision_map.info.height != raw_map.info.height)
  {
    return 0U;
  }

  std::vector<uint8_t> classes(cell_count, kFree);
  std::vector<uint8_t> free_mask(cell_count, 0);
  for (std::size_t i = 0; i < cell_count; ++i) {
    const int value = raw_map.data[i];
    if (value < 0) {
      classes[i] = kUnknown;
    } else if (value >= config.occ_threshold) {
      classes[i] = kOccupied;
    } else {
      free_mask[i] = 1;
    }
  }

  const std::vector<uint8_t> holes = find_hole_cells(classes, width, height, config);
  if (std::none_of(holes.begin(), holes.end(), [](uint8_t v) {return v != 0;})) {
    return 0U;
  }

  // Cells next to where the robot has already been (or failed to go) are left alone.
  std::vector<uint8_t> checked(cell_count, 0);
  const int checked_radius = static_cast<int>(std::ceil(config.checked_radius_m / resolution));
  const int checked_radius_sq = checked_radius * checked_radius;
  for (const auto & point : checked_points) {
    const int cx = static_cast<int>(std::floor((point.first - raw_map.info.origin.position.x) / resolution));
    const int cy = static_cast<int>(std::floor((point.second - raw_map.info.origin.position.y) / resolution));
    for (int y = std::max(0, cy - checked_radius); y <= std::min(height - 1, cy + checked_radius); ++y) {
      for (int x = std::max(0, cx - checked_radius); x <= std::min(width - 1, cx + checked_radius); ++x) {
        if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= checked_radius_sq) {
          checked[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x)] = 1;
        }
      }
    }
  }

  const IntegralImage hole_sums(holes, width, height);
  const IntegralImage free_sums(free_mask, width, height);
  const int window = std::max(1, static_cast<int>(std::lround(config.window_radius_m / resolution)));

  std::size_t remarked = 0;
  for (int y = 0; y < height; ++y) {
    const int y0 = std::max(0, y - window);
    const int y1 = std::min(height - 1, y + window);
    for (int x = 0; x < width; ++x) {
      const std::size_t idx =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
      // Only cells the decision map calls free can turn unknown; raw obstacles stay obstacles.
      if (decision_map.data[idx] != 0 || classes[idx] == kOccupied || checked[idx] != 0) {
        continue;
      }
      const int x0 = std::max(0, x - window);
      const int x1 = std::min(width - 1, x + window);
      const int32_t hole_count = hole_sums.sum(x0, y0, x1, y1);
      if (hole_count == 0) {
        continue;
      }
      const int32_t observed = hole_count + free_sums.sum(x0, y0, x1, y1);
      if (static_cast<double>(hole_count) >= config.min_unknown_ratio * static_cast<double>(observed)) {
        decision_map.data[idx] = -1;
        ++remarked;
      }
    }
  }

  return remarked;
}

}  // namespace nova_exploration
