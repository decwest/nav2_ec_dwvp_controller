// Copyright (c) 2026 Fumiya Ohnishi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "nav2_dynamic_window_vector_pursuit_controller/dynamic_window_vector_pursuit_controller.hpp"

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
#include "ament_index_cpp/get_package_share_directory.hpp"
#if __has_include("nav2_core/controller_exceptions.hpp")
#include "nav2_core/controller_exceptions.hpp"
#elif __has_include("nav2_core/exceptions.hpp")
#include "nav2_core/exceptions.hpp"
#else
#error "Neither nav2_core/controller_exceptions.hpp nor nav2_core/exceptions.hpp was found."
#endif
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/utils.hpp"

namespace dwvp = nav2_dynamic_window_vector_pursuit_controller;
namespace rpp = nav2_regulated_pure_pursuit_controller;

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_dynamic_window_vector_pursuit_controller
{
namespace
{
std::string getDefaultCsvLogDirectory(const rclcpp::Logger & logger)
{
  try {
    const auto pkg_share =
      ament_index_cpp::get_package_share_directory("nav2_dynamic_window_vector_pursuit_controller");
    const auto log_dir = std::filesystem::path(pkg_share) / "data";
    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    if (ec) {
      RCLCPP_WARN(
        logger,
        "Failed to create default csv directory '%s' (%s). Falling back to /tmp.",
        log_dir.string().c_str(), ec.message().c_str());
      return "/tmp";
    }
    return log_dir.string();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger,
      "Failed to resolve package share for default csv directory (%s). Falling back to /tmp.",
      e.what());
    return "/tmp";
  }
}
}  // namespace

void DynamicWindowVectorPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  rpp::RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error(
            "Failed to lock parent node in DynamicWindowVectorPursuitController::configure");
  }

  const std::string default_csv_log_directory = getDefaultCsvLogDirectory(logger_);

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
    node.get(), name + ".use_collision_detection_for_omni",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node.get(), name + ".enable_csv_logging",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node.get(), name + ".csv_log_directory",
    rclcpp::ParameterValue(default_csv_log_directory));
  declare_parameter_if_not_declared(
    node.get(), name + ".csv_filename_prefix",
    rclcpp::ParameterValue(std::string("dwvp_log")));

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
  node->get_parameter(name + ".use_dynamic_window_vector_pursuit", use_dynamic_window_vector_pursuit_);
  node->get_parameter(name + ".use_collision_detection_for_omni", use_collision_detection_for_omni_);
  node->get_parameter(name + ".enable_csv_logging", enable_csv_logging_);
  node->get_parameter(name + ".csv_log_directory", csv_log_directory_);
  node->get_parameter(name + ".csv_filename_prefix", csv_filename_prefix_);

  max_accel_x_ = std::max(1e-6, std::abs(max_accel_x_));
  max_accel_y_ = std::max(1e-6, std::abs(max_accel_y_));
  max_accel_theta_ = std::max(1e-6, std::abs(max_accel_theta_));
  min_orientation_time_ = std::max(1e-3, min_orientation_time_);
  orientation_time_weight_ = std::max(1e-3, orientation_time_weight_);

  std::transform(
    velocity_feedback_.begin(), velocity_feedback_.end(), velocity_feedback_.begin(),
    [](const unsigned char c) {return static_cast<char>(std::toupper(c));});

  if (use_collision_detection_ && !use_collision_detection_for_omni_) {
    RCLCPP_WARN(
      logger_,
      "Collision detection is enabled in RPP params, but DWVP omni collision checking is "
      "disabled by default. Set %s.use_collision_detection_for_omni:=true to enable approximate "
      "checking based on translational speed magnitude.",
      name.c_str());
  }

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

  constraints_violation_flag_pub_ =
    node->create_publisher<std_msgs::msg::Bool>("constraints_violation_flag", 1);
}

void DynamicWindowVectorPursuitController::activate()
{
  rpp::RegulatedPurePursuitController::activate();
  if (constraints_violation_flag_pub_) {
    constraints_violation_flag_pub_->on_activate();
  }
}

void DynamicWindowVectorPursuitController::deactivate()
{
  rpp::RegulatedPurePursuitController::deactivate();
  closeCsvLogFile();
  if (constraints_violation_flag_pub_) {
    constraints_violation_flag_pub_->on_deactivate();
  }
}

void DynamicWindowVectorPursuitController::cleanup()
{
  rpp::RegulatedPurePursuitController::cleanup();
  closeCsvLogFile();
  constraints_violation_flag_pub_.reset();
}

void DynamicWindowVectorPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  rpp::RegulatedPurePursuitController::setPlan(path);
  if (!enable_csv_logging_) {
    return;
  }
  openNewCsvLogFile(rclcpp::Clock(RCL_SYSTEM_TIME).now());
}

void DynamicWindowVectorPursuitController::openNewCsvLogFile(const rclcpp::Time & stamp)
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

void DynamicWindowVectorPursuitController::closeCsvLogFile()
{
  if (!csv_stream_.is_open()) {
    csv_header_written_ = false;
    return;
  }

  csv_stream_.flush();
  csv_stream_.close();
  csv_header_written_ = false;
}

void DynamicWindowVectorPursuitController::writeCsvLogLine(
  const rclcpp::Time & stamp,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  const geometry_msgs::msg::Twist & feedback_speed,
  const geometry_msgs::msg::Twist & cmd_velocity,
  const DynamicWindowBounds3D & dynamic_window,
  double curvature,
  double regulated_speed,
  double lookahead_dist,
  double lookahead_yaw,
  bool is_accel,
  bool constraints_violation)
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
      << "sec,nsec,x,y,yaw,"
      << "v_real,w_real,v_now,w_now,v_cmd,w_cmd,v_nav,w_nav,"
      << "vx_real,vy_real,vx_now,vy_now,vx_cmd,vy_cmd,vx_nav,vy_nav,"
      << "velocity_violation,curvature,"
      << "dw_vx_max,dw_vx_min,dw_vy_max,dw_vy_min,dw_w_max,dw_w_min,"
      << "v_reg,lookahead_dist,lookahead_yaw,is_accel\n";
    csv_header_written_ = true;
  }

  const auto sec = static_cast<int32_t>(stamp.seconds());
  const auto nsec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
  const auto compute_nav_axis = [&](double next_vel, double current_vel, double min_vel, double max_vel,
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
    cmd_velocity.angular.z, feedback_speed.angular.z, min_vel_theta_, max_vel_theta_, max_accel_theta_);

  const double v_real = std::hypot(speed.linear.x, speed.linear.y);
  const double v_now = std::hypot(feedback_speed.linear.x, feedback_speed.linear.y);
  const double v_cmd = std::hypot(cmd_velocity.linear.x, cmd_velocity.linear.y);
  const double v_nav = std::hypot(vx_nav, vy_nav);

  csv_stream_ << sec << ","
              << nsec << ","
              << std::fixed << std::setprecision(6)
              << pose.pose.position.x << ","
              << pose.pose.position.y << ","
              << tf2::getYaw(pose.pose.orientation) << ","
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
              << (is_accel ? 1 : 0)
              << "\n";
  csv_stream_.flush();
}

double DynamicWindowVectorPursuitController::getLookAheadDistanceOmni(
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

double DynamicWindowVectorPursuitController::computeLookAheadYaw(
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

double DynamicWindowVectorPursuitController::computeRemainingPathDistance(
  const nav_msgs::msg::Path & transformed_plan) const
{
  if (transformed_plan.poses.empty()) {
    return 0.0;
  }

  double distance = std::hypot(
    transformed_plan.poses.front().pose.position.x,
    transformed_plan.poses.front().pose.position.y);

  for (size_t i = 1; i < transformed_plan.poses.size(); ++i) {
    const auto & p0 = transformed_plan.poses[i - 1].pose.position;
    const auto & p1 = transformed_plan.poses[i].pose.position;
    distance += std::hypot(p1.x - p0.x, p1.y - p0.y);
  }

  return distance;
}

bool DynamicWindowVectorPursuitController::shouldAccelerate(
  const nav_msgs::msg::Path & transformed_plan,
  const geometry_msgs::msg::Twist & current_velocity) const
{
  const double goal_distance = computeRemainingPathDistance(transformed_plan);
  const double translational_speed = std::hypot(current_velocity.linear.x, current_velocity.linear.y);
  const double translational_accel_limit = std::min(max_accel_x_, max_accel_y_);

  if (translational_accel_limit <= 1e-6) {
    return false;
  }

  const double decel_distance =
    (translational_speed * translational_speed) / (2.0 * translational_accel_limit);

  return goal_distance > decel_distance;
}

geometry_msgs::msg::Twist DynamicWindowVectorPursuitController::computeDesiredVelocity(
  const geometry_msgs::msg::PoseStamped & lookahead_pose,
  const double lookahead_yaw) const
{
  geometry_msgs::msg::Twist desired_velocity;

  const double dx = lookahead_pose.pose.position.x;
  const double dy = lookahead_pose.pose.position.y;
  const double distance = std::hypot(dx, dy);

  const double vx_limit = std::max(std::abs(max_vel_x_), std::abs(min_vel_x_));
  const double vy_limit = std::max(std::abs(max_vel_y_), std::abs(min_vel_y_));
  const double max_translational_speed = std::hypot(vx_limit, vy_limit);

  if (distance > 1e-6 && max_translational_speed > 1e-6) {
    desired_velocity.linear.x = max_translational_speed * dx / distance;
    desired_velocity.linear.y = max_translational_speed * dy / distance;
  }

  if (max_translational_speed > 1e-6) {
    const double time_to_goal_position = distance / max_translational_speed;
    const double time_to_goal_orientation =
      std::max(min_orientation_time_, orientation_time_weight_ * time_to_goal_position);
    const double heading_error = angles::normalize_angle(lookahead_yaw);
    desired_velocity.angular.z = heading_error / time_to_goal_orientation;
  }

  return desired_velocity;
}

DynamicWindowVectorPursuitController::DynamicWindowBounds3D
DynamicWindowVectorPursuitController::computeDynamicWindow(
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

  if (bounds.min_vx > bounds.max_vx) {
    const double clipped = std::clamp(current_velocity.linear.x, min_vel_x_, max_vel_x_);
    bounds.min_vx = clipped;
    bounds.max_vx = clipped;
  }

  if (bounds.min_vy > bounds.max_vy) {
    const double clipped = std::clamp(current_velocity.linear.y, min_vel_y_, max_vel_y_);
    bounds.min_vy = clipped;
    bounds.max_vy = clipped;
  }

  if (bounds.min_w > bounds.max_w) {
    const double clipped = std::clamp(current_velocity.angular.z, min_vel_theta_, max_vel_theta_);
    bounds.min_w = clipped;
    bounds.max_w = clipped;
  }

  return bounds;
}

geometry_msgs::msg::Twist DynamicWindowVectorPursuitController::clipVelocityToBounds(
  const geometry_msgs::msg::Twist & velocity,
  const DynamicWindowBounds3D & bounds) const
{
  geometry_msgs::msg::Twist clipped_velocity;
  clipped_velocity.linear.x = std::clamp(velocity.linear.x, bounds.min_vx, bounds.max_vx);
  clipped_velocity.linear.y = std::clamp(velocity.linear.y, bounds.min_vy, bounds.max_vy);
  clipped_velocity.angular.z = std::clamp(velocity.angular.z, bounds.min_w, bounds.max_w);
  return clipped_velocity;
}

std::tuple<bool, double, double> DynamicWindowVectorPursuitController::computeRayBoxIntersectionAlphaRange(
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

double DynamicWindowVectorPursuitController::computeClosestAlphaToBoxFromRay(
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

geometry_msgs::msg::Twist DynamicWindowVectorPursuitController::computeOptimalVelocityWithinDynamicWindow(
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
    intersects ? (prefer_large_alpha ? alpha_max : alpha_min)
    : computeClosestAlphaToBoxFromRay(ray_direction, box_min, box_max, prefer_large_alpha);

  geometry_msgs::msg::Twist optimal_velocity;
  optimal_velocity.linear.x = alpha * ray_direction[0];
  optimal_velocity.linear.y = alpha * ray_direction[1];
  optimal_velocity.angular.z = alpha * ray_direction[2];

  return clipVelocityToBounds(optimal_velocity, bounds);
}

bool DynamicWindowVectorPursuitController::evaluateVelocityConstraints(
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

geometry_msgs::msg::TwistStamped DynamicWindowVectorPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker || !goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker tolerances.");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  auto transformed_plan = transformGlobalPlan(pose);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException(
            "DynamicWindowVectorPursuitController: transformed plan is empty.");
  }

  const geometry_msgs::msg::Twist feedback_speed =
    (velocity_feedback_ == "CLOSED_LOOP") ? speed : last_command_velocity_;

  const double lookahead_dist = getLookAheadDistanceOmni(feedback_speed);
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);
  const double curvature = (carrot_dist2 > 1e-9) ?
    (2.0 * carrot_pose.pose.position.y / carrot_dist2) : 0.0;

  const double lookahead_yaw = computeLookAheadYaw(transformed_plan, lookahead_dist);
  const geometry_msgs::msg::Twist desired_velocity = computeDesiredVelocity(carrot_pose, lookahead_yaw);
  const double regulated_speed = std::hypot(desired_velocity.linear.x, desired_velocity.linear.y);
  const DynamicWindowBounds3D dynamic_window = computeDynamicWindow(feedback_speed);
  const bool is_accel = shouldAccelerate(transformed_plan, feedback_speed);

  const geometry_msgs::msg::Twist cmd_twist =
    use_dynamic_window_vector_pursuit_ ?
    computeOptimalVelocityWithinDynamicWindow(desired_velocity, dynamic_window, is_accel) :
    clipVelocityToBounds(desired_velocity, dynamic_window);

  if (use_collision_detection_ && use_collision_detection_for_omni_) {
    const double translational_speed = std::hypot(cmd_twist.linear.x, cmd_twist.linear.y);
    const double carrot_dist = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    if (isCollisionImminent(
        pose, translational_speed, cmd_twist.angular.z, carrot_dist))
    {
      throw nav2_core::PlannerException(
              "DynamicWindowVectorPursuitController detected collision ahead.");
    }
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist = cmd_twist;

  std_msgs::msg::Bool constraints_violation_flag_msg;
  constraints_violation_flag_msg.data =
    evaluateVelocityConstraints(cmd_vel.twist, feedback_speed);
  rclcpp::Time log_stamp(pose.header.stamp);
  if (log_stamp.nanoseconds() == 0) {
    log_stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  }
  writeCsvLogLine(
    log_stamp,
    pose,
    speed,
    feedback_speed,
    cmd_vel.twist,
    dynamic_window,
    curvature,
    regulated_speed,
    lookahead_dist,
    lookahead_yaw,
    is_accel,
    constraints_violation_flag_msg.data);
  if (constraints_violation_flag_pub_) {
    constraints_violation_flag_pub_->publish(constraints_violation_flag_msg);
  }

  last_command_velocity_ = cmd_vel.twist;
  return cmd_vel;
}

}  // namespace nav2_dynamic_window_vector_pursuit_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_dynamic_window_vector_pursuit_controller::DynamicWindowVectorPursuitController,
  nav2_core::Controller)
