// Copyright (c) 2026 Fumiya Ohnishi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <array>
#include <fstream>
#include <string>
#include <tuple>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace nav2_dynamic_window_vector_pursuit_controller
{

class DynamicWindowVectorPursuitController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  DynamicWindowVectorPursuitController() = default;
  ~DynamicWindowVectorPursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

protected:
  struct DynamicWindowBounds3D
  {
    double min_vx;
    double max_vx;
    double min_vy;
    double max_vy;
    double min_w;
    double max_w;
  };

  double getLookAheadDistanceOmni(const geometry_msgs::msg::Twist & speed) const;
  double computeLookAheadYaw(const nav_msgs::msg::Path & transformed_plan, double lookahead_dist) const;
  double computeRemainingPathDistance(const nav_msgs::msg::Path & transformed_plan) const;
  bool shouldAccelerate(
    const nav_msgs::msg::Path & transformed_plan,
    const geometry_msgs::msg::Twist & current_velocity) const;

  geometry_msgs::msg::Twist computeDesiredVelocity(
    const geometry_msgs::msg::PoseStamped & lookahead_pose,
    const double lookahead_yaw) const;

  DynamicWindowBounds3D computeDynamicWindow(const geometry_msgs::msg::Twist & current_velocity) const;
  geometry_msgs::msg::Twist clipVelocityToBounds(
    const geometry_msgs::msg::Twist & velocity,
    const DynamicWindowBounds3D & bounds) const;

  std::tuple<bool, double, double> computeRayBoxIntersectionAlphaRange(
    const std::array<double, 3> & ray_direction,
    const std::array<double, 3> & box_min,
    const std::array<double, 3> & box_max) const;

  double computeClosestAlphaToBoxFromRay(
    const std::array<double, 3> & ray_direction,
    const std::array<double, 3> & box_min,
    const std::array<double, 3> & box_max,
    bool prefer_large_alpha) const;

  geometry_msgs::msg::Twist computeOptimalVelocityWithinDynamicWindow(
    const geometry_msgs::msg::Twist & desired_velocity,
    const DynamicWindowBounds3D & bounds,
    bool prefer_large_alpha) const;

  bool evaluateVelocityConstraints(
    const geometry_msgs::msg::Twist & next_cmd_vel,
    const geometry_msgs::msg::Twist & current_cmd_vel) const;

  void openNewCsvLogFile(const rclcpp::Time & stamp);
  void closeCsvLogFile();
  void writeCsvLogLine(
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::PoseStamped & map_pose,
    bool map_pose_valid,
    const geometry_msgs::msg::Twist & speed,
    const geometry_msgs::msg::Twist & feedback_speed,
    const geometry_msgs::msg::Twist & cmd_velocity,
    const DynamicWindowBounds3D & dynamic_window,
    double curvature,
    double regulated_speed,
    double lookahead_dist,
    double lookahead_yaw,
    bool is_accel,
    bool constraints_violation);

  double max_vel_x_ {0.6};
  double min_vel_x_ {-0.6};
  double max_vel_y_ {0.6};
  double min_vel_y_ {-0.6};
  double max_vel_theta_ {1.0};
  double min_vel_theta_ {-1.0};
  double max_accel_x_ {1.0};
  double max_accel_y_ {1.0};
  double max_accel_theta_ {2.0};
  double orientation_time_weight_ {1.0};
  double min_orientation_time_ {0.2};
  std::string velocity_feedback_ {"OPEN_LOOP"};
  bool use_dynamic_window_vector_pursuit_ {true};
  bool use_collision_detection_for_omni_ {false};
  bool enable_csv_logging_ {false};
  std::string csv_log_directory_ {"/tmp"};
  std::string csv_filename_prefix_ {"dwvp_log"};
  std::string csv_pose_frame_ {"map"};
  bool csv_header_written_ {false};
  std::ofstream csv_stream_;
  geometry_msgs::msg::Twist last_command_velocity_;
};

}  // namespace nav2_dynamic_window_vector_pursuit_controller
