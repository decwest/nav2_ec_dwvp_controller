// Copyright (c) 2026 Fumiya Ohnishi
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_ec_dwvp_controller/ec_dwvp_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "angles/angles.h"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_ec_dwvp_controller
{
namespace
{
geometry_msgs::msg::PoseStamped localPose(
  const geometry_msgs::msg::PoseStamped & target,
  const geometry_msgs::msg::PoseStamped & robot)
{
  auto result = target;
  const double yaw = tf2::getYaw(robot.pose.orientation);
  const double dx = target.pose.position.x - robot.pose.position.x;
  const double dy = target.pose.position.y - robot.pose.position.y;
  result.pose.position.x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  result.pose.position.y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  result.pose.position.z = 0.0;
  // Preserve invalid/absent orientation so computeLookAheadYaw keeps its fallback.
  const auto & q = target.pose.orientation;
  if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-6) {
    const double angle = angles::normalize_angle(tf2::getYaw(q) - yaw);
    result.pose.orientation = geometry_msgs::msg::Quaternion();
    result.pose.orientation.z = std::sin(angle / 2.0);
    result.pose.orientation.w = std::cos(angle / 2.0);
  }
  return result;
}
}  // namespace

double ECDWVPController::stoppingTime(const geometry_msgs::msg::Twist & command) const
{
  return std::max({std::abs(command.linear.x) / max_accel_x_,
             std::abs(command.linear.y) / max_accel_y_,
             std::abs(command.angular.z) / max_accel_theta_});
}

double ECDWVPController::sweptSpeed(const geometry_msgs::msg::Twist & command, double radius)
{
  return std::hypot(command.linear.x, command.linear.y) + radius * std::abs(command.angular.z);
}

double ECDWVPController::stoppingDistance(
  const geometry_msgs::msg::Twist & command, double radius) const
{
  // Common-ratio braking obeys every body-axis limit. One full control period
  // covers command holding and the discretization of the subsequent deceleration.
  return sweptSpeed(command, radius) * (control_duration_ + 0.5 * stoppingTime(command));
}

double ECDWVPController::brakingScale(
  const geometry_msgs::msg::Twist & command, double radius, double available) const
{
  if (stoppingDistance(command, radius) <= available) {
    return 1.0;
  }
  if (available <= 0.0) {
    return 0.0;
  }
  const double speed = sweptSpeed(command, radius);
  const double time = stoppingTime(command);
  // Stable positive root of speed * (dt * scale + time * scale^2 / 2) = available.
  const double d = available / speed;
  return std::clamp(2.0 * d / (control_duration_ +
           std::sqrt(control_duration_ * control_duration_ + 2.0 * time * d)), 0.0, 1.0);
}

double ECDWVPController::predictionHorizon(
  const geometry_msgs::msg::Twist & feedback, const geometry_msgs::msg::Twist & measured) const
{
  for (const auto * velocity : {&feedback, &measured}) {
    if (!std::isfinite(velocity->linear.x) || !std::isfinite(velocity->linear.y) ||
      !std::isfinite(velocity->angular.z))
    {
      throw nav2_core::NoValidControl("Non-finite prediction velocity");
    }
  }
  const double maximum_stop_time = std::max({
      std::max(-min_vel_x_, max_vel_x_) / max_accel_x_,
      std::max(-min_vel_y_, max_vel_y_) / max_accel_y_,
      std::max(-min_vel_theta_, max_vel_theta_) / max_accel_theta_,
      stoppingTime(feedback), stoppingTime(measured)});
  return std::max(control_duration_, maximum_stop_time + collision_prediction_time_buffer_);
}

ECDWVPController::PredictionContext ECDWVPController::makePredictionContext(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & feedback,
  const geometry_msgs::msg::Twist & measured, double yaw_tolerance)
{
  PredictionContext context;
  context.start = pose;
  context.yaw_tolerance = yaw_tolerance;
  context.radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  context.horizon = predictionHorizon(feedback, measured);
  const auto transform_plan = [&](const nav_msgs::msg::Path & source) {
      auto result = source;
      result.header = pose.header;
      if (source.header.frame_id == pose.header.frame_id) {
        for (auto & point : result.poses) {
          point.header = pose.header;
        }
        return result;
      }
      try {
        const auto transform = tf_->lookupTransform(
          pose.header.frame_id, source.header.frame_id, rclcpp::Time(pose.header.stamp),
          rclcpp::Duration::from_seconds(params_->transform_tolerance));
        for (size_t i = 0; i < source.poses.size(); ++i) {
          tf2::doTransform(source.poses[i], result.poses[i], transform);
          result.poses[i].header = pose.header;
        }
      } catch (const tf2::TransformException & error) {
        throw nav2_core::ControllerTFError(error.what());
      }
      return result;
    };
  context.plan = transform_plan(path_handler_->getPlan());
  if (lateral_time_constant_ > 0.0) {
    context.reference = transform_plan(reference_plan_);
  }
  return context;
}

nav_msgs::msg::Path ECDWVPController::predictionLocalPlan(
  const nav_msgs::msg::Path & plan, const geometry_msgs::msg::PoseStamped & pose,
  size_t & begin) const
{
  if (begin >= plan.poses.size()) {
    throw nav2_core::InvalidPath("Empty prediction path");
  }
  // Mirror Jazzy RPP's bounded nearest-vertex search and retain two terminal
  // points. This cursor belongs only to this rollout, never to path_handler_.
  const auto first = plan.poses.begin() + begin;
  const auto upper = nav2_util::geometry_utils::first_after_integrated_distance(
    first, plan.poses.end(), params_->max_robot_pose_search_dist);
  auto closest = nav2_util::geometry_utils::min_by(
    first, upper, [&pose](const auto & point) {
      return nav2_util::geometry_utils::euclidean_distance(point, pose);
    });
  if (std::distance(first, upper) > 1 && closest == std::prev(upper)) {
    --closest;
  }
  begin = static_cast<size_t>(std::distance(plan.poses.begin(), closest));
  const double extent = 0.5 * std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  nav_msgs::msg::Path result;
  result.header = pose.header;
  result.header.frame_id = costmap_ros_->getBaseFrameID();
  for (auto it = closest; it != plan.poses.end(); ++it) {
    if (nav2_util::geometry_utils::euclidean_distance(*it, pose) > extent) {
      break;
    }
    auto point = localPose(*it, pose);
    point.header = result.header;
    result.poses.push_back(point);
  }
  if (result.poses.empty()) {
    throw nav2_core::InvalidPath("Prediction path outside local extent");
  }
  return result;
}

std::string ECDWVPController::predictionPoseStatus(const geometry_msgs::msg::PoseStamped & pose)
{
  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  unsigned int mx, my;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    throw nav2_core::NoValidControl("Non-finite predicted pose");
  }
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return "map_boundary";
  }
  // A footprint partially outside the snapshot is not verified free space.
  for (const auto & vertex : costmap_ros_->getRobotFootprint()) {
    if (!costmap_->worldToMap(
        x + std::cos(yaw) * vertex.x - std::sin(yaw) * vertex.y,
        y + std::sin(yaw) * vertex.x + std::cos(yaw) * vertex.y, mx, my))
    {
      return "map_boundary";
    }
  }
  return collision_checker_->inCollision(x, y, yaw) ? "collision" : "clear";
}

ECDWVPController::PredictionResult ECDWVPController::predictTracking(
  const PredictionContext & context, const ControlStep & first, double stop_budget)
{
  PredictionResult result;
  result.path.header = context.start.header;
  auto pose = context.start;
  result.path.poses.push_back(pose);
  result.event = predictionPoseStatus(pose);
  if (result.event != "clear") {
    result.event_distance = 0.0;
    return result;
  }
  if (!std::isfinite(context.horizon) || context.horizon <= 0.0 ||
    context.horizon / control_duration_ > collision_prediction_max_steps_ + 1e-9)
  {
    result.event = "step_limit";
    return result;
  }
  size_t begin = 0;
  double elapsed = 0.0;
  auto step = first;
  while (elapsed + 1e-10 < context.horizon) {
    auto command = step.command;
    if (!std::isfinite(command.linear.x) || !std::isfinite(command.linear.y) ||
      !std::isfinite(command.angular.z))
    {
      throw nav2_core::NoValidControl("Non-finite predicted command");
    }
    const double available = std::max(0.0, stop_budget - result.distance);
    // Do not creep back and forth within the first-contact refinement accuracy.
    // This only rounds toward an earlier stop and still respects the reachable window.
    const double contact_tolerance = costmap_->getResolution() / 512.0;
    const double scale = std::isfinite(stop_budget) && available <= contact_tolerance ?
      0.0 : brakingScale(command, context.radius, available);
    if (scale < 1.0) {
      command.linear.x *= scale;
      command.linear.y *= scale;
      command.angular.z *= scale;
      command = clipVelocityToBounds(command, step.regulated_window);
    }
    if (result.steps == 0) {
      result.first_command = command;
      result.first_scale = scale;
    }
    // Clipping can prevent a requested slowdown or change the velocity direction.
    // Do not claim that the scaled command is reachable AND stopping-safe without checking it.
    if (std::isfinite(stop_budget) &&
      stoppingDistance(command, context.radius) > available + 1e-8)
    {
      result.event = "insufficient_braking_distance";
      return result;
    }
    result.control_poses.push_back(pose);
    result.commands.push_back(command);
    ++result.steps;
    const double speed = sweptSpeed(command, context.radius);
    if (speed < 1e-9 && (step.at_goal ||
      (std::isfinite(stop_budget) && available <= contact_tolerance)))
    {
      return result;
    }
    const double dt = std::min(control_duration_, context.horizon - elapsed);
    geometry_msgs::msg::Pose2D start;
    start.x = pose.pose.position.x;
    start.y = pose.pose.position.y;
    start.theta = tf2::getYaw(pose.pose.orientation);
    const auto samples = projectMotion(
      start, command, dt, 0.0, costmap_->getResolution(), context.radius);
    const double increment = samples.size() > 1 ? speed * dt / (samples.size() - 1) : 0.0;
    for (size_t i = 1; i < samples.size(); ++i) {
      pose.pose.position.x = samples[i].x;
      pose.pose.position.y = samples[i].y;
      pose.pose.orientation = geometry_msgs::msg::Quaternion();
      pose.pose.orientation.z = std::sin(samples[i].theta / 2.0);
      pose.pose.orientation.w = std::cos(samples[i].theta / 2.0);
      result.path.poses.push_back(pose);
      result.event = predictionPoseStatus(pose);
      if (result.event != "clear") {
        // Refine only the first contact interval. Using a whole preceding sample
        // makes the stopping target jump on successive cycles and can demand
        // more deceleration than the dynamic window permits.
        const double interval = dt / (samples.size() - 1);
        double lower = interval * (i - 1);
        double upper = interval * i;
        for (int refinement = 0; refinement < 8; ++refinement) {
          const double middle = 0.5 * (lower + upper);
          const auto p = projectMotion(
            start, command, middle, 0.0, costmap_->getResolution(), context.radius).back();
          auto probe = pose;
          probe.pose.position.x = p.x;
          probe.pose.position.y = p.y;
          probe.pose.orientation.z = std::sin(p.theta / 2.0);
          probe.pose.orientation.w = std::cos(p.theta / 2.0);
          if (predictionPoseStatus(probe) == "clear") {lower = middle;} else {upper = middle;}
        }
        result.event_distance = result.distance + speed * (lower - interval * (i - 1));
        return result;
      }
      result.distance += increment;
    }
    elapsed += dt;
    if (elapsed + 1e-10 >= context.horizon) {
      break;
    }
    const auto local_plan = predictionLocalPlan(context.plan, pose, begin);
    const auto goal = localPose(context.plan.poses.back(), pose);
    ReferenceProjection reference;
    if (lateral_time_constant_ > 0.0) {
      reference = projectReference(context.reference, pose);
    }
    step = computeControlStep(
      local_plan, goal, reference, command,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y),
      context.yaw_tolerance);
  }
  return result;
}

geometry_msgs::msg::Twist ECDWVPController::limitPredictedCollision(
  const PredictionContext & context, const ControlStep & first)
{
  const auto start = std::chrono::steady_clock::now();
  PredictionResult nominal;
  PredictionResult checked;
  std::string status;
  try {
    nominal = predictTracking(context, first);
    checked = nominal;
    status = nominal.event;
    if (nominal.event == "collision" || nominal.event == "map_boundary") {
      checked = predictTracking(
        context, first, std::max(0.0, nominal.event_distance - collision_stop_margin_));
      status = checked.event == "clear" ?
        (checked.first_scale < 1.0 ? "limited" : "obstacle_ahead") : checked.event;
    }
  } catch (const nav2_core::ControllerException &) {
    status = "prediction_failed";
    checked.event = status;
  }
  const double elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  nominal_arc_pub_->publish(nominal.path);
  omni_arc_pub_->publish(checked.path);
  publishPredictionDiagnostics(context, first, nominal, checked, status, elapsed);
  if (checked.event != "clear" || checked.first_scale < 1.0) {
    publishCollisionSnapshot(context.start, first.command);
  }
  if (checked.event != "clear") {
    throw nav2_core::NoValidControl("EC-DWVP collision prediction: " + status);
  }
  return checked.first_command;
}

void ECDWVPController::publishPredictionDiagnostics(
  const PredictionContext & context, const ControlStep & first,
  const PredictionResult & nominal, const PredictionResult & checked,
  const std::string & status, double elapsed_ms)
{
  if (prediction_diagnostics_pub_->get_subscription_count() == 0) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header = context.start.header;
  diagnostic_msgs::msg::DiagnosticStatus entry;
  entry.name = plugin_name_ + "/collision_prediction";
  entry.hardware_id = "ec_dwvp";
  entry.message = status;
  entry.level = checked.event != "clear" ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    (checked.first_scale < 1.0 ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK);
  const auto add = [&entry](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      entry.values.push_back(pair);
    };
  add("mode", collision_prediction_mode_);
  add("nominal_event", nominal.event);
  add("collision_distance", std::to_string(nominal.event_distance));
  add("braking_distance", std::to_string(stoppingDistance(first.command, context.radius)));
  add("stop_margin", std::to_string(collision_stop_margin_));
  add("speed_scale", std::to_string(checked.first_scale));
  add("selected_braking_distance",
    std::to_string(stoppingDistance(checked.first_command, context.radius)));
  add("prediction_time", std::to_string(context.horizon));
  add("prediction_steps", std::to_string(nominal.steps));
  const bool verified = nominal.event == "collision" || nominal.event == "map_boundary";
  add("verification_steps", std::to_string(verified ? checked.steps : 0u));
  add("compute_time_ms", std::to_string(elapsed_ms));
  message.status.push_back(entry);
  prediction_diagnostics_pub_->publish(message);
}

}  // namespace nav2_ec_dwvp_controller
