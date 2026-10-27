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

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nova_exploration/decision_map.hpp"
#include "nova_exploration/frontier_explorer_core.hpp"
#include "nova_exploration/frontier_search.hpp"
#include "nova_exploration/sparse_coverage.hpp"

namespace nova_exploration
{
namespace
{

// A walled 6 m x 3 m room at SLAM resolution. Everything is observed except a 2 m x 1 m patch
// in the far end crossed by short 1-cell unknown streaks: what a corridor looks like when the
// lidar only saw it from far away at a grazing angle.
constexpr double kResolution = 0.05;
constexpr int kWidth = 120;
constexpr int kHeight = 60;

geometry_msgs::msg::Pose make_pose(double x, double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::msg::OccupancyGrid build_grid(int default_value)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = static_cast<uint32_t>(kWidth);
  msg.info.height = static_cast<uint32_t>(kHeight);
  msg.info.resolution = kResolution;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<std::size_t>(kWidth * kHeight), static_cast<int8_t>(default_value));
  return msg;
}

void set_cell(nav_msgs::msg::OccupancyGrid & msg, int x, int y, int value)
{
  msg.data[static_cast<std::size_t>(y * kWidth + x)] = static_cast<int8_t>(value);
}

int cell(const nav_msgs::msg::OccupancyGrid & msg, int x, int y)
{
  return msg.data[static_cast<std::size_t>(y * kWidth + x)];
}

nav_msgs::msg::OccupancyGrid walled_room()
{
  auto msg = build_grid(0);
  for (int x = 0; x < kWidth; ++x) {
    set_cell(msg, x, 0, 100);
    set_cell(msg, x, kHeight - 1, 100);
  }
  for (int y = 0; y < kHeight; ++y) {
    set_cell(msg, 0, y, 100);
    set_cell(msg, kWidth - 1, y, 100);
  }
  return msg;
}

// Streaks are 8 cells long with 2-cell gaps, one every 4 rows: every unknown component is
// smaller than min_frontier_size_cells, so on its own none of them is a frontier.
void add_streak_patch(nav_msgs::msg::OccupancyGrid & msg, int x0, int y0, int x1, int y1)
{
  for (int y = y0; y <= y1; y += 4) {
    for (int x = x0; x <= x1; ++x) {
      if ((x - x0) % 10 < 8) {
        set_cell(msg, x, y, -1);
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid sparse_corridor_map()
{
  auto msg = walled_room();
  add_streak_patch(msg, 70, 20, 109, 40);
  return msg;
}

DecisionMapConfig production_decision_map_config()
{
  DecisionMapConfig config;
  config.optimization_enabled = true;
  config.occ_threshold = 99;
  config.sigma_s = 2.0;
  config.sigma_r = 30.0;
  config.dilation_kernel_radius_cells = 1;
  return config;
}

FrontierSearchOptions production_search_options()
{
  FrontierSearchOptions options;
  options.occ_threshold = 99;
  options.min_frontier_size_cells = 12;
  return options;
}

SparseCoverageConfig production_sparse_config()
{
  SparseCoverageConfig config;
  config.occ_threshold = 99;
  return config;
}

FrontierSearchResult search(const nav_msgs::msg::OccupancyGrid & decision_msg)
{
  const auto costmap_msg = walled_room();
  return get_frontier(
    make_pose(0.5, 1.5),
    OccupancyGrid2d(decision_msg),
    OccupancyGrid2d(costmap_msg),
    std::nullopt,
    0.0,
    false,
    production_search_options());
}

TEST(SparseCoverageTests, DecisionMapAloneHidesSparsePatch)
{
  // The bug this pass exists for: filter + dilation erase every streak, nothing is left to explore.
  const auto decision = build_decision_map(
    OccupancyGrid2d(sparse_corridor_map()), production_decision_map_config());
  EXPECT_TRUE(search(decision.optimized_map_msg).frontiers.empty());
}

TEST(SparseCoverageTests, SparsePatchBecomesFrontier)
{
  const auto raw = sparse_corridor_map();
  auto decision = build_decision_map(OccupancyGrid2d(raw), production_decision_map_config())
    .optimized_map_msg;

  const std::size_t remarked = apply_sparse_coverage(raw, decision, {}, production_sparse_config());
  EXPECT_GT(remarked, 0U);
  EXPECT_EQ(cell(decision, 90, 30), -1);
  // Well-observed space away from the patch is untouched, and so are the walls.
  EXPECT_EQ(cell(decision, 20, 30), 0);
  EXPECT_EQ(cell(decision, kWidth - 1, 30), 100);

  const auto result = search(decision);
  ASSERT_FALSE(result.frontiers.empty());
  for (const auto & frontier : result.frontiers) {
    // Every frontier belongs to the patch (x 3.5-5.5 m, y 1.0-2.0 m) plus the window margin.
    EXPECT_GE(frontier.centroid.first, 3.0);
    EXPECT_GE(frontier.centroid.second, 0.6);
    EXPECT_LE(frontier.centroid.second, 2.4);
  }
}

TEST(SparseCoverageTests, SingleStreakInObservedSpaceIsIgnored)
{
  // One ray gap in otherwise dense coverage is noise, which the decision map is right to drop.
  auto raw = walled_room();
  for (int x = 60; x < 68; ++x) {
    set_cell(raw, x, 30, -1);
  }
  auto decision = build_decision_map(OccupancyGrid2d(raw), production_decision_map_config())
    .optimized_map_msg;
  EXPECT_EQ(apply_sparse_coverage(raw, decision, {}, production_sparse_config()), 0U);
}

TEST(SparseCoverageTests, ShadowsBehindObstaclesAreNotHoles)
{
  // Unknown pockets against obstacles (chair legs, furniture shadows) never fill from closer
  // up either, so they must not keep exploration alive.
  auto raw = walled_room();
  for (int y = 20; y <= 40; y += 4) {
    for (int x = 70; x < 110; x += 5) {
      set_cell(raw, x, y, 100);
      set_cell(raw, x + 1, y, -1);
      set_cell(raw, x + 2, y, -1);
    }
  }
  auto decision = build_decision_map(OccupancyGrid2d(raw), production_decision_map_config())
    .optimized_map_msg;
  EXPECT_EQ(apply_sparse_coverage(raw, decision, {}, production_sparse_config()), 0U);
}

TEST(SparseCoverageTests, CheckedPointsSuppressRemarking)
{
  const auto raw = sparse_corridor_map();
  auto decision = build_decision_map(OccupancyGrid2d(raw), production_decision_map_config())
    .optimized_map_msg;
  auto config = production_sparse_config();
  config.checked_radius_m = 1.5;

  // The robot has stood in the middle of the patch: what is still sparse stays explored.
  const std::vector<std::pair<double, double>> checked{{4.5, 1.5}};
  EXPECT_EQ(apply_sparse_coverage(raw, decision, checked, config), 0U);
  EXPECT_TRUE(search(decision).frontiers.empty());
}

TEST(SparseCoverageTests, GeometryMismatchIsANoOp)
{
  const auto raw = sparse_corridor_map();
  auto decision = raw;
  decision.info.width = kWidth - 1;
  EXPECT_EQ(apply_sparse_coverage(raw, decision, {}, production_sparse_config()), 0U);
}

std::unique_ptr<FrontierExplorerCore> make_sparse_core()
{
  FrontierExplorerCoreParams params;
  params.occ_threshold = 99;
  params.min_frontier_size_cells = 12;
  params.sparse_coverage_enabled = true;
  FrontierExplorerCoreCallbacks callbacks;
  auto now_ns = std::make_shared<int64_t>(10'000'000'000);
  callbacks.now_ns = [now_ns]() {
      *now_ns += 1'000'000;
      return *now_ns;
    };
  callbacks.debug_outputs_enabled = []() {return false;};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose & pose,
    const OccupancyGrid2d & map,
    const OccupancyGrid2d & costmap,
    const std::optional<OccupancyGrid2d> & local_costmap,
    double min_goal_distance,
    bool return_robot_cell) {
      return get_frontier(
        pose, map, costmap, local_costmap, min_goal_distance, return_robot_cell,
        production_search_options());
    };

  auto core = std::make_unique<FrontierExplorerCore>(params, callbacks);
  core->map = OccupancyGrid2d(sparse_corridor_map());
  core->costmap = OccupancyGrid2d(walled_room());
  core->map_generation = 1;
  core->costmap_generation = 1;
  core->decision_map_dirty = true;
  return core;
}

TEST(SparseCoverageCoreTests, SnapshotOffersPatchUntilRobotHasBeenThere)
{
  auto core = make_sparse_core();

  const auto far = core->get_frontier_snapshot(make_pose(0.5, 1.5), 0.0);
  EXPECT_FALSE(far.frontiers.empty());
  EXPECT_GT(core->sparse_coverage_remarked_cells, 0U);
  ASSERT_EQ(core->sparse_coverage_checked_points.size(), 1U);

  // Walking the patch records a trail; the patch never filled in, so it is given up on.
  for (double x = 3.0; x <= 5.75; x += 0.25) {
    core->get_frontier_snapshot(make_pose(x, 1.5), 0.0);
  }
  const auto after = core->get_frontier_snapshot(make_pose(5.5, 1.5), 0.0);
  EXPECT_TRUE(after.frontiers.empty());
  EXPECT_EQ(core->sparse_coverage_remarked_cells, 0U);
}

TEST(SparseCoverageCoreTests, FailedGoalIsGivenUpAndSessionResetClearsTrail)
{
  auto core = make_sparse_core();
  core->note_sparse_coverage_checked_point(1.0, 1.0, false);
  // Inside the trail spacing: not recorded, unless forced (failed goal).
  core->note_sparse_coverage_checked_point(1.1, 1.0, false);
  EXPECT_EQ(core->sparse_coverage_checked_points.size(), 1U);
  core->note_sparse_coverage_checked_point(1.1, 1.0, true);
  EXPECT_EQ(core->sparse_coverage_checked_points.size(), 2U);

  core->start_exploration_session();
  EXPECT_TRUE(core->sparse_coverage_checked_points.empty());
}

TEST(SparseCoverageCoreTests, DisabledCoreRecordsNothing)
{
  auto core = make_sparse_core();
  core->params.sparse_coverage_enabled = false;
  const auto snapshot = core->get_frontier_snapshot(make_pose(0.5, 1.5), 0.0);
  EXPECT_TRUE(snapshot.frontiers.empty());
  EXPECT_TRUE(core->sparse_coverage_checked_points.empty());
}

}  // namespace
}  // namespace nova_exploration
