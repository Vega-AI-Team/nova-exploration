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

#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "nova_exploration/frontier_types.hpp"

namespace nova_exploration
{

// Sparse-coverage pass: puts barely-observed free space back on the frontier list.
//
// Free space seen only from far away, at a grazing angle, comes out of SLAM full of small
// unknown holes (ray gaps). Each hole is below min_frontier_size_cells, and the decision map's
// filter + dilation fills them all in, so the area reads as explored although it was never
// looked at properly. This pass finds free cells whose neighbourhood is dense with such holes
// and marks them unknown in the decision map, so the patch's boundary becomes one ordinary
// frontier the robot drives to; seen up close the holes fill and the patch disappears.
//
// A hole is a small unknown component that does not touch the map edge and is bordered mostly
// by free cells. Shadows behind furniture and pockets against walls border occupied cells and
// never count, so clutter cannot keep exploration alive forever.
//
// Cells within checked_radius_m of a checked point are never re-marked. The explorer records
// the robot's trail and failed goals there: a patch that is still sparse after the robot has
// been next to it, or that it cannot reach, will not get better by trying again.
struct SparseCoverageConfig
{
  int occ_threshold{OCC_THRESHOLD};
  // Radius of the square window the hole density is measured over.
  double window_radius_m{0.30};
  // Hole cells / (hole + free cells) in the window at or above which a free cell is re-marked.
  double min_unknown_ratio{0.15};
  // Unknown components larger than this are real unexplored space, not ray gaps.
  int max_hole_cells{150};
  // Fraction of a component's 8-neighbour border allowed on obstacles; 0 = any contact is a shadow.
  double max_hole_occupied_border_ratio{0.0};
  double checked_radius_m{1.0};
};

// Re-marks sparsely observed free cells of decision_map as unknown. raw_map and decision_map
// must share geometry (the decision map is built from raw_map). Returns the number of cells
// re-marked; 0 also when the geometries disagree.
std::size_t apply_sparse_coverage(
  const nav_msgs::msg::OccupancyGrid & raw_map,
  nav_msgs::msg::OccupancyGrid & decision_map,
  const std::vector<std::pair<double, double>> & checked_points,
  const SparseCoverageConfig & config);

}  // namespace nova_exploration
