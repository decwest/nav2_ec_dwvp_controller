// Copyright (c) 2026 Fumiya Ohnishi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "nav2_ec_dwvp_controller/ec_dwvp_controller.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "angles/angles.h"
#if __has_include("nav2_core/controller_exceptions.hpp")
#include "nav2_core/controller_exceptions.hpp"
#elif __has_include("nav2_core/exceptions.hpp")
#include "nav2_core/exceptions.hpp"
#else
#error "Neither nav2_core/controller_exceptions.hpp nor nav2_core/exceptions.hpp was found."
#endif
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

namespace rpp = nav2_regulated_pure_pursuit_controller;

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_ec_dwvp_controller
{
namespace
{
double wrapOrientationError(double angle)
{
  const double wrapped = angles::normalize_angle(angle);
  // Use [-pi, pi) for both lookahead and final-goal errors.
  return wrapped == M_PI ? -M_PI : wrapped;
}
}  // namespace

void ECDWVPController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  rpp::RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error(
            "Failed to lock parent node in ECDWVPController::configure");
  }

  const std::string default_csv_log_directory = "/tmp";

  declare_parameter_if_not_declared(
    node.get(), name + ".vp_translation_speed", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".vp_translation_speed", vp_translation_speed_);
  declare_parameter_if_not_declared(
    node.get(), name + ".lateral_time_constant", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".lateral_time_constant", lateral_time_constant_);

  declare_parameter_if_not_declared(
    node.get(), name + ".max_vel_x",
    rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node.get(), name + ".min_vel_x",
    rclcpp::ParameterValue(-0.6));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_vel_y",
    rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node.get(), name + ".min_vel_y",
    rclcpp::ParameterValue(-0.6));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_vel_theta",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".min_vel_theta",
    rclcpp::ParameterValue(-1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_accel_x",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_accel_y",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".max_accel_theta",
    rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".orientation_time_weight",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".min_orientation_time",
    rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node.get(), name + ".velocity_feedback",
    rclcpp::ParameterValue("OPEN_LOOP"));
  declare_parameter_if_not_declared(
    node.get(), name + ".use_dynamic_window_vector_pursuit",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node.get(), name + ".use_uniform_velocity_scaling", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node.get(), name + ".use_collision_detection_for_omni",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node.get(), name + ".enable_csv_logging",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node.get(), name + ".csv_log_directory",
    rclcpp::ParameterValue(default_csv_log_directory));
  declare_parameter_if_not_declared(
    node.get(), name + ".csv_filename_prefix",
    rclcpp::ParameterValue(std::string("dwvp_log")));
  declare_parameter_if_not_declared(
    node.get(), name + ".csv_pose_frame",
    rclcpp::ParameterValue(std::string("map")));

  node->get_parameter(name + ".max_vel_x", max_vel_x_);
  node->get_parameter(name + ".min_vel_x", min_vel_x_);
  node->get_parameter(name + ".max_vel_y", max_vel_y_);
  node->get_parameter(name + ".min_vel_y", min_vel_y_);
  node->get_parameter(name + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(name + ".min_vel_theta", min_vel_theta_);
  node->get_parameter(name + ".max_accel_x", max_accel_x_);
  node->get_parameter(name + ".max_accel_y", max_accel_y_);
  node->get_parameter(name + ".max_accel_theta", max_accel_theta_);
  node->get_parameter(name + ".orientation_time_weight", orientation_time_weight_);
  node->get_parameter(name + ".min_orientation_time", min_orientation_time_);
  node->get_parameter(name + ".velocity_feedback", velocity_feedback_);
  node->get_parameter(
    name + ".use_dynamic_window_vector_pursuit",
    use_dynamic_window_vector_pursuit_);
  node->get_parameter(name + ".use_uniform_velocity_scaling", use_uniform_velocity_scaling_);
  if (use_uniform_velocity_scaling_ && use_dynamic_window_vector_pursuit_) {
    throw std::invalid_argument("Uniform scaling requires use_dynamic_window_vector_pursuit=false");
  }
  node->get_parameter(
    name + ".use_collision_detection_for_omni",
    use_collision_detection_for_omni_);
  node->get_parameter(name + ".enable_csv_logging", enable_csv_logging_);
  node->get_parameter(name + ".csv_log_directory", csv_log_directory_);
  node->get_parameter(name + ".csv_filename_prefix", csv_filename_prefix_);
  node->get_parameter(name + ".csv_pose_frame", csv_pose_frame_);

  const auto valid_axis = [](double lo, double hi, double accel) {
      return std::isfinite(lo) && std::isfinite(hi) && lo <= 0.0 && hi >= 0.0 &&
             lo <= hi && std::isfinite(accel) && accel > 0.0;
    };
  if (!valid_axis(min_vel_x_, max_vel_x_, max_accel_x_) ||
    !valid_axis(min_vel_y_, max_vel_y_, max_accel_y_) ||
    !valid_axis(min_vel_theta_, max_vel_theta_, max_accel_theta_) ||
    !std::isfinite(vp_translation_speed_) ||
    !std::isfinite(lateral_time_constant_) ||
    !std::isfinite(orientation_time_weight_) || orientation_time_weight_ <= 0.0 ||
    !std::isfinite(min_orientation_time_) || min_orientation_time_ <= 0.0 ||
    !std::isfinite(control_duration_) || control_duration_ <= 0.0 ||
    !std::isfinite(max_allowed_time_to_collision_up_to_carrot_) ||
    max_allowed_time_to_collision_up_to_carrot_ < 0.0 ||
    !std::isfinite(desired_linear_vel_) || desired_linear_vel_ < 0.0 ||
    !std::isfinite(approach_velocity_scaling_dist_) || approach_velocity_scaling_dist_ < 0.0)
  {
    throw std::invalid_argument("Invalid DWVP limits, control period, or orientation time");
  }
  std::transform(
    velocity_feedback_.begin(), velocity_feedback_.end(), velocity_feedback_.begin(),
    [](const unsigned char c) {return static_cast<char>(std::toupper(c));});
  if (velocity_feedback_ != "OPEN_LOOP" && velocity_feedback_ != "CLOSED_LOOP") {
    throw std::invalid_argument("velocity_feedback must be OPEN_LOOP or CLOSED_LOOP");
  }
  if (use_regulated_linear_velocity_scaling_) {
    RCLCPP_WARN(
      logger_,
      "DWVP supports cost and approach regulation; curvature regulation is unused.");
  }
  last_command_valid_ = false;

  if (enable_csv_logging_) {
    if (csv_log_directory_.empty()) {
      csv_log_directory_ = default_csv_log_directory;
    }
    if (csv_filename_prefix_.empty()) {
      csv_filename_prefix_ = "dwvp_log";
    }
    std::error_code ec;
    std::filesystem::create_directories(csv_log_directory_, ec);
    if (ec) {
      RCLCPP_WARN(
        logger_,
        "Failed to create csv_log_directory '%s' (%s). Falling back to /tmp.",
        csv_log_directory_.c_str(), ec.message().c_str());
      csv_log_directory_ = "/tmp";
      std::filesystem::create_directories(csv_log_directory_, ec);
    }
  }

}

void ECDWVPController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    throw std::invalid_argument("Speed limit must be finite and nonnegative");
  }
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  rpp::RegulatedPurePursuitController::setSpeedLimit(speed_limit, percentage);
}

void ECDWVPController::activate()
{
  last_command_valid_ = false;
  rpp::RegulatedPurePursuitController::activate();
}

void ECDWVPController::deactivate()
{
  last_command_valid_ = false;
  rpp::RegulatedPurePursuitController::deactivate();
  closeCsvLogFile();
}

void ECDWVPController::cleanup()
{
  last_command_valid_ = false;
  reference_plan_ = nav_msgs::msg::Path();
  rpp::RegulatedPurePursuitController::cleanup();
  closeCsvLogFile();
}

void ECDWVPController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  rpp::RegulatedPurePursuitController::setPlan(path);
  if (lateral_time_constant_ > 0.0) {
    reference_plan_ = path;
  }
  // New plans may follow a stopped/cancelled goal. Seed from measured speed once.
  last_command_valid_ = false;
  if (!enable_csv_logging_) {
    return;
  }
  openNewCsvLogFile(rclcpp::Clock(RCL_SYSTEM_TIME).now());
}

void ECDWVPController::openNewCsvLogFile(const rclcpp::Time & stamp)
{
  closeCsvLogFile();

  const auto sec = static_cast<std::time_t>(stamp.seconds());
  const auto nsec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
  std::tm tm_snapshot {};
  if (const std::tm * tm_ptr = std::localtime(&sec)) {
    tm_snapshot = *tm_ptr;
  }

  std::ostringstream filename;
  filename << csv_filename_prefix_ << "_"
           << std::put_time(&tm_snapshot, "%Y%m%d_%H%M%S")
           << "_" << std::setw(9) << std::setfill('0') << nsec
           << ".csv";
  const std::filesystem::path path = std::filesystem::path(csv_log_directory_) / filename.str();
  csv_stream_.open(path.string(), std::ios::out | std::ios::trunc);
  if (!csv_stream_.is_open()) {
    RCLCPP_WARN(logger_, "Failed to open DWVP csv log file: %s", path.string().c_str());
    return;
  }

  csv_header_written_ = false;
  RCLCPP_INFO(logger_, "DWVP csv logging started: %s", path.string().c_str());
}

void ECDWVPController::closeCsvLogFile()
{
  if (!csv_stream_.is_open()) {
    csv_header_written_ = false;
    return;
  }

  csv_stream_.flush();
  csv_stream_.close();
  csv_header_written_ = false;
}

void ECDWVPController::writeCsvLogLine(
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
  const ResponseDiagnostics & response)
{
  if (!enable_csv_logging_) {
    return;
  }
  if (!csv_stream_.is_open()) {
    openNewCsvLogFile(stamp);
  }
  if (!csv_stream_.is_open()) {
    return;
  }

  if (!csv_header_written_) {
    csv_stream_
      << "sec,nsec,x,y,yaw,map_x,map_y,map_yaw,map_pose_valid,"
      << "v_real,w_real,v_now,w_now,v_cmd,w_cmd,v_nav,w_nav,"
      << "vx_real,vy_real,vx_now,vy_now,vx_cmd,vy_cmd,vx_nav,vy_nav,"
      << "velocity_violation,curvature,"
      << "dw_vx_max,dw_vx_min,dw_vy_max,dw_vy_min,dw_w_max,dw_w_min,"
      << "v_reg,lookahead_dist,lookahead_yaw,"
      << "response_rule,lateral_error,chord_length,response_distance\n";
    csv_header_written_ = true;
  }

  const auto sec = static_cast<int32_t>(stamp.seconds());
  const auto nsec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
  const auto compute_nav_axis =
    [&](double next_vel, double current_vel, double min_vel, double max_vel,
      double max_accel) -> double
    {
      const double lower = std::max(current_vel - max_accel * control_duration_, min_vel);
      const double upper = std::min(current_vel + max_accel * control_duration_, max_vel);
      return std::clamp(next_vel, lower, upper);
    };
  const double vx_nav = compute_nav_axis(
    cmd_velocity.linear.x, feedback_speed.linear.x, min_vel_x_, max_vel_x_, max_accel_x_);
  const double vy_nav = compute_nav_axis(
    cmd_velocity.linear.y, feedback_speed.linear.y, min_vel_y_, max_vel_y_, max_accel_y_);
  const double w_nav = compute_nav_axis(
    cmd_velocity.angular.z, feedback_speed.angular.z, min_vel_theta_, max_vel_theta_,
    max_accel_theta_);

  const double v_real = std::hypot(speed.linear.x, speed.linear.y);
  const double v_now = std::hypot(feedback_speed.linear.x, feedback_speed.linear.y);
  const double v_cmd = std::hypot(cmd_velocity.linear.x, cmd_velocity.linear.y);
  const double v_nav = std::hypot(vx_nav, vy_nav);
  const double map_x =
    map_pose_valid ? map_pose.pose.position.x : std::numeric_limits<double>::quiet_NaN();
  const double map_y =
    map_pose_valid ? map_pose.pose.position.y : std::numeric_limits<double>::quiet_NaN();
  const double map_yaw = map_pose_valid ? tf2::getYaw(map_pose.pose.orientation) :
    std::numeric_limits<double>::quiet_NaN();

  csv_stream_ << sec << ","
              << nsec << ","
              << std::fixed << std::setprecision(6)
              << pose.pose.position.x << ","
              << pose.pose.position.y << ","
              << tf2::getYaw(pose.pose.orientation) << ","
              << map_x << ","
              << map_y << ","
              << map_yaw << ","
              << (map_pose_valid ? 1 : 0) << ","
              << v_real << ","
              << speed.angular.z << ","
              << v_now << ","
              << feedback_speed.angular.z << ","
              << v_cmd << ","
              << cmd_velocity.angular.z << ","
              << v_nav << ","
              << w_nav << ","
              << speed.linear.x << ","
              << speed.linear.y << ","
              << feedback_speed.linear.x << ","
              << feedback_speed.linear.y << ","
              << cmd_velocity.linear.x << ","
              << cmd_velocity.linear.y << ","
              << vx_nav << ","
              << vy_nav << ","
              << (constraints_violation ? 1 : 0) << ","
              << curvature << ","
              << dynamic_window.max_vx << ","
              << dynamic_window.min_vx << ","
              << dynamic_window.max_vy << ","
              << dynamic_window.min_vy << ","
              << dynamic_window.max_w << ","
              << dynamic_window.min_w << ","
              << regulated_speed << ","
              << lookahead_dist << ","
              << lookahead_yaw << ","
              << response.rule << ","
              << response.lateral_error << ","
              << response.chord_length << ","
              << response.response_distance
              << "\n";
  csv_stream_.flush();
}

double ECDWVPController::getLookAheadDistanceOmni(
  const geometry_msgs::msg::Twist & speed) const
{
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = std::hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
    lookahead_dist = std::clamp(
      lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }
  return lookahead_dist;
}

ECDWVPController::PreviewTarget
ECDWVPController::computePreviewTarget(
  const nav_msgs::msg::Path & plan,
  double distance)
{
  const auto outer = std::find_if(
    plan.poses.begin(), plan.poses.end(),
    [&](const geometry_msgs::msg::PoseStamped & p) {
      return std::hypot(p.pose.position.x, p.pose.position.y) >= distance;
    });
  return {getLookAheadPoint(distance, plan), computeLookAheadYaw(plan, distance),
    outer == plan.poses.end(), outer == plan.poses.begin()};
}

double ECDWVPController::computeLookAheadYaw(
  const nav_msgs::msg::Path & transformed_plan,
  double lookahead_dist) const
{
  if (transformed_plan.poses.empty()) {
    return 0.0;
  }

  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(),
    transformed_plan.poses.end(),
    [&](const geometry_msgs::msg::PoseStamped & ps) {
      return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }

  const auto & q = goal_pose_it->pose.orientation;
  const double q_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (q_norm > 1e-6) {
    const double lookahead_yaw = tf2::getYaw(q);
    if (std::isfinite(lookahead_yaw)) {
      return lookahead_yaw;
    }
  }

  if (transformed_plan.poses.size() == 1u) {
    return 0.0;
  }

  if (goal_pose_it == transformed_plan.poses.begin()) {
    const auto next_it = std::next(goal_pose_it);
    return std::atan2(
      next_it->pose.position.y - goal_pose_it->pose.position.y,
      next_it->pose.position.x - goal_pose_it->pose.position.x);
  }

  const auto prev_it = std::prev(goal_pose_it);
  return std::atan2(
    goal_pose_it->pose.position.y - prev_it->pose.position.y,
    goal_pose_it->pose.position.x - prev_it->pose.position.x);
}

ECDWVPController::ReferenceProjection
ECDWVPController::projectReference(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & pose)
{
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Cannot project onto an empty reference plan");
  }
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;
  ReferenceProjection result;
  double rx = path.poses.front().pose.position.x;
  double ry = path.poses.front().pose.position.y;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double q = length2 > 1e-20 ?
      std::clamp(((px - a.x) * dx + (py - a.y) * dy) / length2, 0.0, 1.0) : 0.0;
    const double x = a.x + q * dx;
    const double y = a.y + q * dy;
    const double distance2 = (px - x) * (px - x) + (py - y) * (py - y);
    // Strict comparison retains the earliest segment, including duplicates.
    if (distance2 < best_distance) {
      best_distance = distance2;
      rx = x;
      ry = y;
      result.segment = i;
      result.fraction = q;
    }
  }
  double tx = 1.0;
  double ty = 0.0;
  size_t nearest_index_distance = std::numeric_limits<size_t>::max();
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const size_t index_distance = i > result.segment ? i - result.segment : result.segment - i;
    if (length > 1e-10 && index_distance < nearest_index_distance) {
      nearest_index_distance = index_distance;
      tx = (b.x - a.x) / length;
      ty = (b.y - a.y) / length;
    }
  }
  result.lateral_error = -ty * (px - rx) + tx * (py - ry);
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  result.x = c * (rx - px) + s * (ry - py);
  result.y = -s * (rx - px) + c * (ry - py);
  result.tangent_x = c * tx + s * ty;
  result.tangent_y = -s * tx + c * ty;
  return result;
}

geometry_msgs::msg::Twist ECDWVPController::computeDesiredVelocity(
  const geometry_msgs::msg::PoseStamped & lookahead_pose,
  const double lookahead_yaw,
  const ReferenceProjection * reference,
  bool goal_region, bool nearest_target,
  ResponseDiagnostics * diagnostics) const
{
  geometry_msgs::msg::Twist desired_velocity;
  ResponseDiagnostics response;

  const double dx = lookahead_pose.pose.position.x;
  const double dy = lookahead_pose.pose.position.y;
  const double distance = std::hypot(dx, dy);

  // Radius of the inscribed circle, including asymmetric reverse limits.
  const double max_translational_speed = vp_translation_speed_ > 0.0 ?
    vp_translation_speed_ : std::min({max_vel_x_, -min_vel_x_, max_vel_y_, -min_vel_y_});

  if (distance > 1e-6) {
    desired_velocity.linear.x = max_translational_speed * dx / distance;
    desired_velocity.linear.y = max_translational_speed * dy / distance;
  }

  // Keep the original arithmetic above literally for every disabled setting.
  if (lateral_time_constant_ > 0.0 && max_translational_speed > 0.0) {
    if (!reference) {
      throw std::invalid_argument("Lateral compensation requires a reference projection");
    }
    const double chord_x = dx - reference->x;
    const double chord_y = dy - reference->y;
    const double length = std::hypot(chord_x, chord_y);
    const double response_distance = lateral_time_constant_ * max_translational_speed;
    if (!std::isfinite(response_distance) || response_distance <= 0.0) {
      throw std::invalid_argument("Lateral response distance must be finite and positive");
    }
    response.lateral_error = reference->lateral_error;
    response.chord_length = length;
    response.response_distance = response_distance;
    double ux = dx;
    double uy = dy;
    if (goal_region && length < 1e-9) {
      response.rule = "goal";
    } else {
      const bool tangent = length < 1e-9 || (!goal_region && nearest_target);
      const double effective =
        goal_region ? std::min(response_distance, length) : response_distance;
      const double correction = reference->lateral_error / effective;
      ux = (tangent ? reference->tangent_x : chord_x / length) +
        correction * reference->tangent_y;
      uy = (tangent ? reference->tangent_y : chord_y / length) -
        correction * reference->tangent_x;
      response.rule = goal_region ? "goal" : tangent ? "tangent" : "chord";
    }
    if (!(goal_region && distance <= 1e-6)) {
      const double norm = std::hypot(ux, uy);
      desired_velocity.linear.x = norm <= 1e-12 ? 0.0 : max_translational_speed * (ux / norm);
      desired_velocity.linear.y = norm <= 1e-12 ? 0.0 : max_translational_speed * (uy / norm);
    }
  }
  if (diagnostics) {
    *diagnostics = response;
  }

  const double position_time = max_translational_speed > 1e-12 ?
    distance / max_translational_speed : 0.0;
  const double orientation_time = std::max(
    min_orientation_time_, orientation_time_weight_ * position_time);
  desired_velocity.angular.z = wrapOrientationError(lookahead_yaw) / orientation_time;

  return desired_velocity;
}

ECDWVPController::DynamicWindowBounds3D
ECDWVPController::computeDynamicWindow(
  const geometry_msgs::msg::Twist & current_velocity) const
{
  DynamicWindowBounds3D bounds;
  const double dt = control_duration_;

  bounds.min_vx = std::max(current_velocity.linear.x - max_accel_x_ * dt, min_vel_x_);
  bounds.max_vx = std::min(current_velocity.linear.x + max_accel_x_ * dt, max_vel_x_);
  bounds.min_vy = std::max(current_velocity.linear.y - max_accel_y_ * dt, min_vel_y_);
  bounds.max_vy = std::min(current_velocity.linear.y + max_accel_y_ * dt, max_vel_y_);
  bounds.min_w = std::max(current_velocity.angular.z - max_accel_theta_ * dt, min_vel_theta_);
  bounds.max_w = std::min(current_velocity.angular.z + max_accel_theta_ * dt, max_vel_theta_);

  if (bounds.min_vx > bounds.max_vx || bounds.min_vy > bounds.max_vy ||
    bounds.min_w > bounds.max_w)
  {
    throw nav2_core::PlannerException(
            "Velocity feedback has no acceleration-feasible velocity window");
  }

  return bounds;
}

double ECDWVPController::maximumTranslationalSpeed() const
{
  return std::hypot(
    std::max(std::abs(min_vel_x_), std::abs(max_vel_x_)),
    std::max(std::abs(min_vel_y_), std::abs(max_vel_y_)));
}

double ECDWVPController::computeRegulatedSpeed(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::Twist & speed,
  double pose_cost)
{
  const double nominal = std::min(maximumTranslationalSpeed(), desired_linear_vel_);
  if (!std::isfinite(nominal) || nominal < 0.0) {
    throw nav2_core::PlannerException("Invalid nominal speed");
  }
  double regulated = nominal;
  double sign = 1.0;
  // Zero curvature excludes the nonholonomic PP curvature heuristic.
  applyConstraints(0.0, speed, pose_cost, path, regulated, sign);
  return std::clamp(regulated, 0.0, nominal);
}

ECDWVPController::DynamicWindowBounds3D
ECDWVPController::applyRegulationToDynamicWindow(
  const DynamicWindowBounds3D & physical_window, double regulated_speed) const
{
  auto result = physical_window;
  const double maximum = maximumTranslationalSpeed();
  const double ratio = maximum > 1e-12 ? std::clamp(regulated_speed / maximum, 0.0, 1.0) : 0.0;
  const auto intersect = [ratio](double & lo, double & hi, double physical_lo, double physical_hi) {
      const double cap_lo = ratio * physical_lo;
      const double cap_hi = ratio * physical_hi;
      if (lo > cap_hi) {
        hi = lo;  // strongest reachable deceleration, never jump to the cap
      } else if (hi < cap_lo) {
        lo = hi;
      } else {
        lo = std::max(lo, cap_lo);
        hi = std::min(hi, cap_hi);
      }
    };
  intersect(result.min_vx, result.max_vx, min_vel_x_, max_vel_x_);
  intersect(result.min_vy, result.max_vy, min_vel_y_, max_vel_y_);
  return result;
}

geometry_msgs::msg::Twist ECDWVPController::computeGoalVelocity(
  double yaw_error, double yaw_tolerance, const DynamicWindowBounds3D & window) const
{
  geometry_msgs::msg::Twist target;
  yaw_error = wrapOrientationError(yaw_error);
  if (std::abs(yaw_error) > yaw_tolerance) {
    target.angular.z = std::copysign(
      std::min(
        std::abs(yaw_error) / min_orientation_time_,
        std::sqrt(2.0 * max_accel_theta_ * std::abs(yaw_error))), yaw_error);
  }
  return clipVelocityToBounds(target, window);
}

std::vector<geometry_msgs::msg::Pose2D> ECDWVPController::projectMotion(
  const geometry_msgs::msg::Pose2D & start, const geometry_msgs::msg::Twist & command,
  double horizon, double carrot_distance, double resolution, double footprint_radius)
{
  std::vector<geometry_msgs::msg::Pose2D> poses {start};
  const double speed = std::hypot(command.linear.x, command.linear.y);
  const double swept_speed = speed + footprint_radius * std::abs(command.angular.z);
  if (horizon <= 0.0 || swept_speed < 1e-12) {
    return poses;
  }
  if (speed > 1e-12 && carrot_distance > 0.0) {
    horizon = std::min(horizon, carrot_distance / speed);
  }
  const double step = std::max(1e-6, 0.5 * resolution / swept_speed);
  const size_t steps = static_cast<size_t>(std::ceil(horizon / step));
  for (size_t i = 1; i <= steps; ++i) {
    const double t = horizon * static_cast<double>(i) / static_cast<double>(steps);
    const double turn = command.angular.z * t;
    double dx = command.linear.x * t;
    double dy = command.linear.y * t;
    if (std::abs(command.angular.z) > 1e-9) {
      dx = (command.linear.x * std::sin(turn) + command.linear.y * (std::cos(turn) - 1.0)) /
        command.angular.z;
      dy = (command.linear.x * (1.0 - std::cos(turn)) + command.linear.y * std::sin(turn)) /
        command.angular.z;
    }
    geometry_msgs::msg::Pose2D p;
    p.x = start.x + std::cos(start.theta) * dx - std::sin(start.theta) * dy;
    p.y = start.y + std::sin(start.theta) * dx + std::cos(start.theta) * dy;
    p.theta = angles::normalize_angle(start.theta + turn);
    poses.push_back(p);
  }
  return poses;
}

bool ECDWVPController::isCollisionImminentOmni(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & command, double carrot_distance)
{
  geometry_msgs::msg::Pose2D start;
  start.x = pose.pose.position.x;
  start.y = pose.pose.position.y;
  start.theta = tf2::getYaw(pose.pose.orientation);
  nav_msgs::msg::Path projected_path;
  projected_path.header = pose.header;
  const auto samples = projectMotion(
    start, command, max_allowed_time_to_collision_up_to_carrot_, carrot_distance,
    costmap_->getResolution(), costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
  bool collision = false;
  for (const auto & p : samples) {
    geometry_msgs::msg::PoseStamped sample;
    sample.header = pose.header;
    sample.pose.position.x = p.x;
    sample.pose.position.y = p.y;
    sample.pose.orientation.z = std::sin(p.theta / 2.0);
    sample.pose.orientation.w = std::cos(p.theta / 2.0);
    projected_path.poses.push_back(sample);
    unsigned int mx, my;
    if (!costmap_->worldToMap(p.x, p.y, mx, my) || inCollision(p.x, p.y, p.theta)) {
      collision = true;
      break;
    }
  }
  carrot_arc_pub_->publish(projected_path);
  return collision;
}

geometry_msgs::msg::Twist ECDWVPController::computeScaledVelocity(
  const geometry_msgs::msg::Twist & desired, double regulated_speed,
  const DynamicWindowBounds3D & dynamic_window) const
{
  // Scale against velocity limits only, then enforce the same reachable window
  // as clipped VP (including retained braking endpoints after sudden caps).
  const auto velocity_box = applyRegulationToDynamicWindow(
    {min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, min_vel_theta_, max_vel_theta_},
    regulated_speed);
  const std::array<double, 3> values {desired.linear.x, desired.linear.y, desired.angular.z};
  const std::array<double, 3> lower {velocity_box.min_vx, velocity_box.min_vy, velocity_box.min_w};
  const std::array<double, 3> upper {velocity_box.max_vx, velocity_box.max_vy, velocity_box.max_w};
  double scale = 1.0;
  for (size_t i = 0; i < values.size(); ++i) {
    if (values[i] > 0.0) {
      scale = std::min(scale, upper[i] / values[i]);
    } else if (values[i] < 0.0) {
      scale = std::min(scale, lower[i] / values[i]);
    }
  }
  geometry_msgs::msg::Twist target;
  target.linear.x = scale * desired.linear.x;
  target.linear.y = scale * desired.linear.y;
  target.angular.z = scale * desired.angular.z;
  return clipVelocityToBounds(target, dynamic_window);
}

geometry_msgs::msg::Twist ECDWVPController::clipVelocityToBounds(
  const geometry_msgs::msg::Twist & velocity,
  const DynamicWindowBounds3D & bounds) const
{
  geometry_msgs::msg::Twist clipped_velocity;
  clipped_velocity.linear.x = std::clamp(velocity.linear.x, bounds.min_vx, bounds.max_vx);
  clipped_velocity.linear.y = std::clamp(velocity.linear.y, bounds.min_vy, bounds.max_vy);
  clipped_velocity.angular.z = std::clamp(velocity.angular.z, bounds.min_w, bounds.max_w);
  return clipped_velocity;
}

std::tuple<bool, double, double> ECDWVPController::computeRayBoxIntersectionAlphaRange(
  const std::array<double, 3> & ray_direction,
  const std::array<double, 3> & box_min,
  const std::array<double, 3> & box_max) const
{
  constexpr double eps = 1e-12;
  double alpha_low = 0.0;
  double alpha_high = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < ray_direction.size(); ++i) {
    const double direction = ray_direction[i];
    const double lower = box_min[i];
    const double upper = box_max[i];

    if (std::abs(direction) < eps) {
      if (lower <= 0.0 && 0.0 <= upper) {
        continue;
      }
      return std::make_tuple(false, 0.0, 0.0);
    }

    const double alpha_1 = lower / direction;
    const double alpha_2 = upper / direction;
    const double axis_low = std::min(alpha_1, alpha_2);
    const double axis_high = std::max(alpha_1, alpha_2);

    alpha_low = std::max(alpha_low, axis_low);
    alpha_high = std::min(alpha_high, axis_high);

    if (alpha_low > alpha_high) {
      return std::make_tuple(false, 0.0, 0.0);
    }
  }

  if (alpha_high < 0.0) {
    return std::make_tuple(false, 0.0, 0.0);
  }

  alpha_low = std::max(alpha_low, 0.0);
  if (alpha_low <= alpha_high) {
    return std::make_tuple(true, alpha_low, alpha_high);
  }

  return std::make_tuple(false, 0.0, 0.0);
}

double ECDWVPController::computeClosestAlphaToBoxFromRay(
  const std::array<double, 3> & ray_direction,
  const std::array<double, 3> & box_min,
  const std::array<double, 3> & box_max,
  bool prefer_large_alpha) const
{
  constexpr double eps = 1e-12;
  constexpr double tol = 1e-10;

  double best_alpha = 0.0;
  double best_dist = std::numeric_limits<double>::infinity();

  auto evaluate_alpha = [&](double alpha) {
      const std::array<double, 3> point {
        alpha * ray_direction[0],
        alpha * ray_direction[1],
        alpha * ray_direction[2]
      };

      const std::array<double, 3> nearest {
        std::clamp(point[0], box_min[0], box_max[0]),
        std::clamp(point[1], box_min[1], box_max[1]),
        std::clamp(point[2], box_min[2], box_max[2])
      };

      const double dist =
        (point[0] - nearest[0]) * (point[0] - nearest[0]) +
        (point[1] - nearest[1]) * (point[1] - nearest[1]) +
        (point[2] - nearest[2]) * (point[2] - nearest[2]);

      if (dist + tol < best_dist) {
        best_dist = dist;
        best_alpha = alpha;
        return;
      }

      if (std::abs(dist - best_dist) <= tol) {
        if (prefer_large_alpha && alpha > best_alpha) {
          best_alpha = alpha;
        }
        if (!prefer_large_alpha && alpha < best_alpha) {
          best_alpha = alpha;
        }
      }
    };

  std::vector<double> breakpoints {0.0};
  breakpoints.reserve(1 + ray_direction.size() * 2);

  for (size_t i = 0; i < ray_direction.size(); ++i) {
    if (std::abs(ray_direction[i]) < eps) {
      continue;
    }
    breakpoints.push_back(box_min[i] / ray_direction[i]);
    breakpoints.push_back(box_max[i] / ray_direction[i]);
  }

  breakpoints.erase(
    std::remove_if(
      breakpoints.begin(),
      breakpoints.end(),
      [](double alpha) {return !std::isfinite(alpha) || alpha < 0.0;}),
    breakpoints.end());

  std::sort(breakpoints.begin(), breakpoints.end());
  breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()), breakpoints.end());

  if (breakpoints.empty()) {
    return 0.0;
  }

  for (const double alpha : breakpoints) {
    evaluate_alpha(alpha);
  }

  auto calc_interval_quadratic_coeff = [&](
    double left,
    double right,
    bool has_right) -> std::pair<double, double>
    {
      const double probe = has_right ? 0.5 * (left + right) : left + 1.0;

      double qa = 0.0;
      double qb = 0.0;
      for (size_t i = 0; i < ray_direction.size(); ++i) {
        const double projection = probe * ray_direction[i];
        if (projection < box_min[i] - eps) {
          qa += ray_direction[i] * ray_direction[i];
          qb += -2.0 * box_min[i] * ray_direction[i];
        } else if (projection > box_max[i] + eps) {
          qa += ray_direction[i] * ray_direction[i];
          qb += -2.0 * box_max[i] * ray_direction[i];
        }
      }
      return std::make_pair(qa, qb);
    };

  for (size_t i = 0; i + 1 < breakpoints.size(); ++i) {
    const double left = breakpoints[i];
    const double right = breakpoints[i + 1];
    const auto [qa, qb] = calc_interval_quadratic_coeff(left, right, true);
    if (qa <= eps) {
      continue;
    }
    const double alpha_star = -qb / (2.0 * qa);
    if (left <= alpha_star && alpha_star <= right) {
      evaluate_alpha(alpha_star);
    }
  }

  const double last = breakpoints.back();
  const auto [qa_last, qb_last] = calc_interval_quadratic_coeff(last, 0.0, false);
  if (qa_last > eps) {
    const double alpha_star = -qb_last / (2.0 * qa_last);
    if (alpha_star >= last) {
      evaluate_alpha(alpha_star);
    }
  }

  return best_alpha;
}

geometry_msgs::msg::Twist ECDWVPController::computeOptimalVelocityWithinDynamicWindow(
  const geometry_msgs::msg::Twist & desired_velocity,
  const DynamicWindowBounds3D & bounds,
  bool prefer_large_alpha) const
{
  const std::array<double, 3> ray_direction {
    desired_velocity.linear.x,
    desired_velocity.linear.y,
    desired_velocity.angular.z
  };
  const std::array<double, 3> box_min {bounds.min_vx, bounds.min_vy, bounds.min_w};
  const std::array<double, 3> box_max {bounds.max_vx, bounds.max_vy, bounds.max_w};

  const double direction_norm = std::sqrt(
    ray_direction[0] * ray_direction[0] +
    ray_direction[1] * ray_direction[1] +
    ray_direction[2] * ray_direction[2]);

  if (direction_norm <= 1e-12) {
    return clipVelocityToBounds(geometry_msgs::msg::Twist(), bounds);
  }

  bool intersects = false;
  double alpha_min = 0.0;
  double alpha_max = 0.0;
  std::tie(intersects, alpha_min, alpha_max) =
    computeRayBoxIntersectionAlphaRange(ray_direction, box_min, box_max);

  const double alpha =
    intersects ? (prefer_large_alpha ? alpha_max : alpha_min) :
    computeClosestAlphaToBoxFromRay(ray_direction, box_min, box_max, prefer_large_alpha);

  geometry_msgs::msg::Twist optimal_velocity;
  optimal_velocity.linear.x = alpha * ray_direction[0];
  optimal_velocity.linear.y = alpha * ray_direction[1];
  optimal_velocity.angular.z = alpha * ray_direction[2];

  return clipVelocityToBounds(optimal_velocity, bounds);
}

bool ECDWVPController::evaluateVelocityConstraints(
  const geometry_msgs::msg::Twist & next_cmd_vel,
  const geometry_msgs::msg::Twist & current_cmd_vel) const
{
  constexpr double eps = 1e-8;
  const double dt = control_duration_;

  auto violates_axis_constraints = [&](double next_vel, double current_vel,
      double min_vel, double max_vel, double max_accel) -> bool
    {
      const double lower = std::max(current_vel - max_accel * dt, min_vel);
      const double upper = std::min(current_vel + max_accel * dt, max_vel);
      return next_vel < lower - eps || next_vel > upper + eps;
    };

  const bool x_violation = violates_axis_constraints(
    next_cmd_vel.linear.x, current_cmd_vel.linear.x,
    min_vel_x_, max_vel_x_, max_accel_x_);

  const bool y_violation = violates_axis_constraints(
    next_cmd_vel.linear.y, current_cmd_vel.linear.y,
    min_vel_y_, max_vel_y_, max_accel_y_);

  const bool w_violation = violates_axis_constraints(
    next_cmd_vel.angular.z, current_cmd_vel.angular.z,
    min_vel_theta_, max_vel_theta_, max_accel_theta_);

  return x_violation || y_violation || w_violation;
}

geometry_msgs::msg::TwistStamped ECDWVPController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  const geometry_msgs::msg::Twist feedback_speed =
    (velocity_feedback_ == "CLOSED_LOOP" || !last_command_valid_) ? speed : last_command_velocity_;
  // A failed command may cause Nav2 to stop; never reuse the pre-failure command.
  last_command_valid_ = false;
  if (!std::isfinite(feedback_speed.linear.x) || !std::isfinite(feedback_speed.linear.y) ||
    !std::isfinite(feedback_speed.angular.z))
  {
    throw nav2_core::PlannerException("Non-finite velocity feedback");
  }


  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  geometry_msgs::msg::Pose pose_tolerance;
  pose_tolerance.orientation.w = 1.0;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker || !goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker tolerances.");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  ReferenceProjection reference;
  if (lateral_time_constant_ > 0.0) {
    geometry_msgs::msg::PoseStamped reference_pose;
    if (!transformPose(reference_plan_.header.frame_id, pose, reference_pose)) {
      throw nav2_core::PlannerException("Cannot transform pose into the reference plan frame");
    }
    // Preserve the incoming segment and all earlier ties across repeated RPP pruning.
    reference = projectReference(reference_plan_, reference_pose);
  }
  auto transformed_plan = transformGlobalPlan(pose);
  geometry_msgs::msg::PoseStamped final_goal;
  auto global_goal = global_plan_.poses.back();
  global_goal.header.frame_id = global_plan_.header.frame_id;
  global_goal.header.stamp = pose.header.stamp;
  if (!transformPose(costmap_ros_->getBaseFrameID(), global_goal, final_goal)) {
    throw nav2_core::PlannerException("Cannot transform final goal");
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException(
            "ECDWVPController: transformed plan is empty.");
  }


  const double lookahead_dist = getLookAheadDistanceOmni(feedback_speed);
  const auto preview = computePreviewTarget(transformed_plan, lookahead_dist);
  const auto & carrot_pose = preview.pose;
  carrot_pub_->publish(createCarrotMsg(carrot_pose));
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);
  const double curvature = (carrot_dist2 > 1e-9) ?
    (2.0 * carrot_pose.pose.position.y / carrot_dist2) : 0.0;

  const double lookahead_yaw = preview.yaw;
  bool goal_region = false;
  bool nearest_target = false;
  if (lateral_time_constant_ > 0.0) {
    const auto & end = transformed_plan.poses.back().pose.position;
    goal_region = preview.goal_region &&
      std::hypot(end.x - final_goal.pose.position.x, end.y - final_goal.pose.position.y) < 1e-9;
    nearest_target = preview.nearest_target;
  }
  ResponseDiagnostics response;
  const geometry_msgs::msg::Twist desired_velocity = computeDesiredVelocity(
    carrot_pose, lookahead_yaw, &reference, goal_region, nearest_target, &response);
  const double regulated_speed = computeRegulatedSpeed(
    transformed_plan, feedback_speed, costAtPose(pose.pose.position.x, pose.pose.position.y));
  const DynamicWindowBounds3D dynamic_window = computeDynamicWindow(feedback_speed);

  const auto regulated_window = applyRegulationToDynamicWindow(dynamic_window, regulated_speed);
  const double final_distance = std::hypot(
    final_goal.pose.position.x, final_goal.pose.position.y);
  const double final_yaw = tf2::getYaw(final_goal.pose.orientation);
  const double yaw_tolerance =
    goal_checker ? std::abs(tf2::getYaw(pose_tolerance.orientation)) : 0.0;
  geometry_msgs::msg::Twist cmd_twist;
  if (final_distance <= goal_dist_tol_ || maximumTranslationalSpeed() < 1e-12) {
    cmd_twist = computeGoalVelocity(final_yaw, yaw_tolerance, regulated_window);
    response = ResponseDiagnostics();
  } else if (use_dynamic_window_vector_pursuit_) {
    cmd_twist = computeOptimalVelocityWithinDynamicWindow(
      desired_velocity, regulated_window, true);
  } else if (use_uniform_velocity_scaling_) {
    cmd_twist = computeScaledVelocity(desired_velocity, regulated_speed, regulated_window);
  } else {
    // Explicit clipped VP ablation: the same reference and feasible limits, without ray search.
    cmd_twist = clipVelocityToBounds(desired_velocity, regulated_window);
  }

  if (use_collision_detection_ && use_collision_detection_for_omni_) {
    const double carrot_dist = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    if (isCollisionImminentOmni(pose, cmd_twist, carrot_dist)) {
      last_command_valid_ = false;
      throw nav2_core::PlannerException("ECDWVPController detected collision ahead");
    }
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist = cmd_twist;

  const bool constraints_violation =
    evaluateVelocityConstraints(cmd_vel.twist, feedback_speed);
  rclcpp::Time log_stamp(pose.header.stamp);
  if (log_stamp.nanoseconds() == 0) {
    log_stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  }
  geometry_msgs::msg::PoseStamped pose_in_map;
  const bool map_pose_valid = enable_csv_logging_ && !csv_pose_frame_.empty() &&
    nav2_util::transformPoseInTargetFrame(
    pose, pose_in_map, *tf_, csv_pose_frame_, costmap_ros_->getTransformTolerance());
  writeCsvLogLine(
    log_stamp,
    pose,
    pose_in_map,
    map_pose_valid,
    speed,
    feedback_speed,
    cmd_vel.twist,
    dynamic_window,
    curvature,
    regulated_speed,
    lookahead_dist,
    lookahead_yaw,
    constraints_violation,
    response);

  last_command_velocity_ = cmd_vel.twist;
  last_command_valid_ = true;
  return cmd_vel;
}

}  // namespace nav2_ec_dwvp_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_ec_dwvp_controller::ECDWVPController,
  nav2_core::Controller)
