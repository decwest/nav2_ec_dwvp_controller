// Copyright (c) 2026 Fumiya Ohnishi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <array>
#include <chrono>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace nav2_ec_dwvp_controller
{

class ECDWVPController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  ECDWVPController() = default;
  ~ECDWVPController() override = default;

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

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void reset() override;
  bool cancel() override;

protected:
  struct ReferenceProjection
  {
    // Projection and inherited unit tangent in the robot frame; error is left-positive.
    double x {0.0};
    double y {0.0};
    double tangent_x {1.0};
    double tangent_y {0.0};
    double lateral_error {0.0};
    size_t segment {0};
    double fraction {0.0};
  };

  struct ResponseDiagnostics
  {
    std::string rule {"inactive"};
    double lateral_error {std::numeric_limits<double>::quiet_NaN()};
    double chord_length {std::numeric_limits<double>::quiet_NaN()};
    double response_distance {std::numeric_limits<double>::quiet_NaN()};
  };

  struct PreviewTarget
  {
    geometry_msgs::msg::PoseStamped pose;
    double yaw;
    bool goal_region;
    bool nearest_target;
  };

  struct DynamicWindowBounds3D
  {
    double min_vx;
    double max_vx;
    double min_vy;
    double max_vy;
    double min_w;
    double max_w;
  };

  struct ControlStep
  {
    geometry_msgs::msg::Twist command;
    DynamicWindowBounds3D window;
    DynamicWindowBounds3D regulated_window;
    PreviewTarget preview;
    ResponseDiagnostics response;
    double lookahead;
    double regulated_speed;
    double curvature;
    bool terminal;
    bool at_goal;
  };

  struct PredictionContext
  {
    nav_msgs::msg::Path plan;
    nav_msgs::msg::Path reference;
    geometry_msgs::msg::PoseStamped start;
    double yaw_tolerance;
    double radius;
    double horizon;
  };

  struct PredictionResult
  {
    nav_msgs::msg::Path path;
    // Control-boundary poses and commands, also used by the independent rollout tests.
    std::vector<geometry_msgs::msg::PoseStamped> control_poses;
    std::vector<geometry_msgs::msg::Twist> commands;
    geometry_msgs::msg::Twist first_command;
    double first_scale {1.0};
    double distance {0.0};
    double event_distance {std::numeric_limits<double>::infinity()};
    size_t steps {0};
    std::string event {"clear"};
  };

  ControlStep computeControlStep(
    const nav_msgs::msg::Path & local_plan,
    const geometry_msgs::msg::PoseStamped & local_goal,
    const ReferenceProjection & reference, const geometry_msgs::msg::Twist & feedback,
    double pose_cost, double yaw_tolerance,
    const geometry_msgs::msg::PoseStamped & robot_pose);
  PredictionContext makePredictionContext(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & feedback,
    const geometry_msgs::msg::Twist & measured, double yaw_tolerance);
  nav_msgs::msg::Path predictionLocalPlan(
    const nav_msgs::msg::Path & plan, const geometry_msgs::msg::PoseStamped & pose,
    size_t & begin) const;
  PredictionResult predictTracking(
    const PredictionContext & context, const ControlStep & first,
    double stop_budget = std::numeric_limits<double>::infinity());
  geometry_msgs::msg::Twist limitPredictedCollision(
    const PredictionContext & context, const ControlStep & first);
  double stoppingTime(const geometry_msgs::msg::Twist & command) const;
  static double sweptSpeed(const geometry_msgs::msg::Twist & command, double radius);
  double stoppingDistance(const geometry_msgs::msg::Twist & command, double radius) const;
  double brakingScale(
    const geometry_msgs::msg::Twist & command, double radius, double available) const;
  double predictionHorizon(
    const geometry_msgs::msg::Twist & feedback,
    const geometry_msgs::msg::Twist & measured) const;
  std::string predictionPoseStatus(const geometry_msgs::msg::PoseStamped & pose);
  void publishPredictionDiagnostics(
    const PredictionContext & context, const ControlStep & first,
    const PredictionResult & nominal, const PredictionResult & checked,
    const std::string & status, double elapsed_ms);

  double getLookAheadDistanceOmni(const geometry_msgs::msg::Twist & speed) const;
  PreviewTarget computePreviewTarget(const nav_msgs::msg::Path & plan, double distance);
  double computeLookAheadYaw(
    const nav_msgs::msg::Path & transformed_plan,
    double lookahead_dist) const;
  geometry_msgs::msg::Twist computeDesiredVelocity(
    const geometry_msgs::msg::PoseStamped & lookahead_pose,
    const double lookahead_yaw,
    const ReferenceProjection * reference = nullptr,
    bool goal_region = false, bool nearest_target = false,
    ResponseDiagnostics * diagnostics = nullptr) const;
  static ReferenceProjection projectReference(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose);

  DynamicWindowBounds3D computeDynamicWindow(const geometry_msgs::msg::Twist & current_velocity)
  const;
  double maximumTranslationalSpeed() const;
  double computeRegulatedSpeed(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::Twist & speed, double pose_cost,
    const geometry_msgs::msg::PoseStamped & robot_pose);
  // The path is relative to robot_pose, including virtual poses during prediction.
  double applyRawCostConstraints(
    const nav_msgs::msg::Path & local_plan,
    const geometry_msgs::msg::PoseStamped & robot_pose, double input_speed);
  DynamicWindowBounds3D applyRegulationToDynamicWindow(
    const DynamicWindowBounds3D & physical_window, double regulated_speed) const;
  geometry_msgs::msg::Twist computeGoalVelocity(
    double yaw_error, double yaw_tolerance, const DynamicWindowBounds3D & window) const;
  static std::vector<geometry_msgs::msg::Pose2D> projectMotion(
    const geometry_msgs::msg::Pose2D & start, const geometry_msgs::msg::Twist & command,
    double horizon, double carrot_distance, double resolution, double footprint_radius);
  bool isCollisionImminentOmni(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & command, double carrot_distance);
  void publishCollisionSnapshot(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & command);

  geometry_msgs::msg::Twist computeScaledVelocity(
    const geometry_msgs::msg::Twist & desired, double regulated_speed,
    const DynamicWindowBounds3D & dynamic_window) const;

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
    bool constraints_violation,
    const ResponseDiagnostics & response);

  std::mutex controller_mutex_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr omni_arc_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr nominal_arc_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    prediction_diagnostics_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::Costmap>::SharedPtr collision_costmap_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    collision_command_pub_;

  double max_vel_x_ {0.6};
  double min_vel_x_ {-0.6};
  double max_vel_y_ {0.6};
  double min_vel_y_ {-0.6};
  double max_vel_theta_ {1.0};
  double min_vel_theta_ {-1.0};
  double max_accel_x_ {1.0};
  double max_accel_y_ {1.0};
  double max_accel_theta_ {2.0};
  double vp_translation_speed_ {0.0};
  double lateral_time_constant_ {0.0};
  bool use_raw_cost_regulated_linear_velocity_scaling_ {false};
  double raw_cost_lookahead_dist_ {0.0};
  double raw_cost_sample_step_ {0.05};
  double raw_cost_min_linear_velocity_ {0.1};
  double orientation_time_weight_ {1.0};
  double min_orientation_time_ {0.2};
  std::string velocity_feedback_ {"OPEN_LOOP"};
  bool use_dynamic_window_vector_pursuit_ {true};
  bool use_uniform_velocity_scaling_ {false};
  bool use_collision_detection_for_omni_ {true};
  std::string collision_prediction_mode_ {"constant_velocity"};
  double collision_prediction_time_buffer_ {2.0};
  double collision_stop_margin_ {0.3};
  int collision_prediction_max_steps_ {300};
  bool enable_csv_logging_ {false};
  std::string csv_log_directory_ {"/tmp"};
  std::string csv_filename_prefix_ {"dwvp_log"};
  std::string csv_pose_frame_ {"map"};
  bool csv_header_written_ {false};
  std::ofstream csv_stream_;
  geometry_msgs::msg::Twist last_command_velocity_;
  bool last_command_valid_ {false};
  std::chrono::steady_clock::time_point last_command_time_;
  // RPP prunes its own plan to the nearest vertex. Projection needs the full polyline.
  nav_msgs::msg::Path reference_plan_;
};

}  // namespace nav2_ec_dwvp_controller
