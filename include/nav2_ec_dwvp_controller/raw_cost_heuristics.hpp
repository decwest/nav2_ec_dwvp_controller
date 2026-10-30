// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2026 Fumiya Ohnishi
//
// SPDX-License-Identifier: Apache-2.0
// Adapted from nav2_raw_cost_rpp_controller's raw_cost_heuristics (3a622a6).

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "std_msgs/msg/header.hpp"

namespace nav2_ec_dwvp_controller
{
namespace raw_cost_heuristics
{

inline bool isCostRegulating(const unsigned char cost)
{
  return cost != nav2_costmap_2d::FREE_SPACE &&
         cost != nav2_costmap_2d::NO_INFORMATION;
}

inline double normalizedRawCost(const unsigned char cost)
{
  if (!isCostRegulating(cost)) {
    return 0.0;
  }

  const auto clamped_cost = std::min(
    cost, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  return static_cast<double>(clamped_cost) /
         static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

inline double rawCostConstraint(
  const double raw_linear_vel,
  const unsigned char cost,
  const double min_linear_velocity)
{
  if (!isCostRegulating(cost) || raw_linear_vel == 0.0) {
    return raw_linear_vel;
  }

  const double raw_speed = std::abs(raw_linear_vel);
  const double min_speed = std::min(raw_speed, std::max(0.0, min_linear_velocity));
  const double cost_scale = normalizedRawCost(cost);
  const double scaled_speed = raw_speed - (raw_speed - min_speed) * cost_scale;
  return raw_linear_vel >= 0.0 ? scaled_speed : -scaled_speed;
}

inline geometry_msgs::msg::PoseStamped makeSamplePose(
  const std_msgs::msg::Header & header,
  const double x,
  const double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

inline std::vector<geometry_msgs::msg::PoseStamped> sampleReferencePath(
  const nav_msgs::msg::Path & path,
  const double lookahead_dist,
  const double sample_step)
{
  std::vector<geometry_msgs::msg::PoseStamped> samples;
  samples.push_back(makeSamplePose(path.header, 0.0, 0.0));

  if (lookahead_dist <= 0.0 || path.poses.empty()) {
    return samples;
  }

  const double step = sample_step > 1e-6 ? sample_step : lookahead_dist;
  double next_sample_dist = std::min(step, lookahead_dist);
  double traveled_dist = 0.0;
  auto previous_pose = samples.front();

  for (const auto & target_pose : path.poses) {
    const double dx = target_pose.pose.position.x - previous_pose.pose.position.x;
    const double dy = target_pose.pose.position.y - previous_pose.pose.position.y;
    const double segment_dist = std::hypot(dx, dy);

    if (segment_dist <= 1e-6) {
      previous_pose = target_pose;
      continue;
    }

    const double segment_end_dist = traveled_dist + segment_dist;
    const double max_dist_on_segment = std::min(lookahead_dist, segment_end_dist);

    while (next_sample_dist <= max_dist_on_segment + 1e-9) {
      const double ratio = (next_sample_dist - traveled_dist) / segment_dist;
      samples.push_back(makeSamplePose(
          path.header,
          previous_pose.pose.position.x + ratio * dx,
          previous_pose.pose.position.y + ratio * dy));
      next_sample_dist += step;
    }

    if (segment_end_dist >= lookahead_dist) {
      break;
    }

    traveled_dist = segment_end_dist;
    previous_pose = target_pose;
  }

  return samples;
}

}  // namespace raw_cost_heuristics
}  // namespace nav2_ec_dwvp_controller
