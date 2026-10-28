// Copyright (c) 2026 Fumiya Ohnishi
// Licensed under the Apache License, Version 2.0.
#pragma once

#include <memory>
#include <cmath>
#include <string>

#include "nav2_ec_dwvp_controller/ec_dwvp_controller.hpp"
#include "tf2/utils.hpp"

class PredictionController : public nav2_ec_dwvp_controller::ECDWVPController
{
public:
  using ECDWVPController::ControlStep;
  using ECDWVPController::PredictionContext;
  using ECDWVPController::PredictionResult;
  using ECDWVPController::brakingScale;
  using ECDWVPController::evaluateVelocityConstraints;
  using ECDWVPController::makePredictionContext;
  using ECDWVPController::limitPredictedCollision;
  using ECDWVPController::predictTracking;
  using ECDWVPController::predictionHorizon;
  using ECDWVPController::projectMotion;
  using ECDWVPController::stoppingDistance;
  using ECDWVPController::stoppingTime;
  using ECDWVPController::sweptSpeed;
  using ECDWVPController::isCollisionImminentOmni;

  void enable(bool value) {params_->use_collision_detection = value;}
  const std::string & mode() const {return collision_prediction_mode_;}
  void maxSteps(int value) {collision_prediction_max_steps_ = value;}
  void goalTolerance(double value) {goal_dist_tol_ = value;}
  void compensation(double value) {lateral_time_constant_ = value;}
  void costRegulation(bool value) {params_->use_cost_regulated_linear_velocity_scaling = value;}
  void approachRegulation(double value) {params_->approach_velocity_scaling_dist = value;}
  size_t pathSize() {return path_handler_->getPlan().poses.size();}
  bool commandValid() const {return last_command_valid_;}
  auto command() const {return last_command_velocity_;}
  void seed(const geometry_msgs::msg::Twist & value)
  {
    last_command_velocity_ = value;
    last_command_valid_ = true;
    last_command_time_ = std::chrono::steady_clock::now();
  }
  ControlStep firstStep(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & feedback,
    double yaw_tolerance = 0.0)
  {
    ReferenceProjection reference;
    if (lateral_time_constant_ > 0.0) {
      geometry_msgs::msg::PoseStamped reference_pose;
      if (!path_handler_->transformPose(reference_plan_.header.frame_id, pose, reference_pose)) {
        throw std::runtime_error("reference transform failed");
      }
      reference = projectReference(reference_plan_, reference_pose);
    }
    const auto local = path_handler_->transformGlobalPlan(pose,
      params_->max_robot_pose_search_dist);
    auto target = path_handler_->getPlan().poses.back();
    target.header = path_handler_->getPlan().header;
    target.header.stamp = pose.header.stamp;
    geometry_msgs::msg::PoseStamped goal;
    if (!path_handler_->transformPose(costmap_ros_->getBaseFrameID(), target, goal)) {
      throw std::runtime_error("goal transform failed");
    }
    return computeControlStep(
      local, goal, reference, feedback,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), yaw_tolerance);
  }
};

inline geometry_msgs::msg::Twist velocity(double x, double y, double w)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = x;
  result.linear.y = y;
  result.angular.z = w;
  return result;
}

inline geometry_msgs::msg::PoseStamped worldPose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "odom";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.z = std::sin(yaw / 2.0);
  result.pose.orientation.w = std::cos(yaw / 2.0);
  return result;
}

struct PredictionHarness
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::shared_ptr<tf2_ros::Buffer> tf;
  PredictionController controller;
  geometry_msgs::msg::PoseStamped pose = worldPose(0, 0, 0);

  explicit PredictionHarness(double stop_margin = 0.3)
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("prediction_test");
    node->declare_parameter("controller_frequency", 30.0);
    node->declare_parameter("FollowPath.collision_prediction_mode", "closed_loop");
    node->declare_parameter("FollowPath.collision_stop_margin", stop_margin);
    node->declare_parameter("FollowPath.use_collision_detection", false);
    node->declare_parameter("FollowPath.use_regulated_linear_velocity_scaling", false);
    node->declare_parameter("FollowPath.use_cost_regulated_linear_velocity_scaling", false);
    node->declare_parameter("FollowPath.approach_velocity_scaling_dist", 0.6);
    node->declare_parameter("FollowPath.min_approach_linear_velocity", 0.05);
    node->declare_parameter("FollowPath.regulated_linear_scaling_min_speed", 0.05);
    node->declare_parameter("FollowPath.cost_scaling_dist", 0.6);
    node->declare_parameter("FollowPath.cost_scaling_gain", 1.0);
    node->declare_parameter("FollowPath.inflation_cost_scaling_factor", 3.0);
    node->declare_parameter("FollowPath.max_robot_pose_search_dist", 2.5);
    node->declare_parameter("FollowPath.desired_linear_vel", 0.5);
    node->declare_parameter("FollowPath.vp_translation_speed", 0.5);
    node->declare_parameter("FollowPath.lateral_time_constant", 0.75);
    node->declare_parameter("FollowPath.lookahead_time", 2.0);
    node->declare_parameter("FollowPath.min_lookahead_dist", 0.5);
    node->declare_parameter("FollowPath.max_lookahead_dist", 1.0);
    node->declare_parameter("FollowPath.use_velocity_scaled_lookahead_dist", true);
    for (const auto & axis : {"x", "y", "theta"}) {
      node->declare_parameter(std::string("FollowPath.max_vel_") + axis, 0.5);
      node->declare_parameter(std::string("FollowPath.min_vel_") + axis, -0.5);
      node->declare_parameter(std::string("FollowPath.max_accel_") + axis, 0.5);
    }
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("prediction_map");
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    map->set_parameter(rclcpp::Parameter("global_frame", "odom"));
    // Below we supply the already padded Castella footprint.
    map->set_parameter(rclcpp::Parameter("footprint_padding", 0.0));
    map->on_configure(rclcpp_lifecycle::State());
    map->getCostmap()->resizeMap(200, 200, 0.05, -5.0, -5.0);
    clear();
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = footprint[1].x = 0.42;
    footprint[2].x = footprint[3].x = -0.48;
    footprint[0].y = footprint[3].y = 0.31;
    footprint[1].y = footprint[2].y = -0.31;
    map->setRobotFootprint(footprint);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);
    updateTransform();
    controller.configure(node, "FollowPath", tf, map);
    controller.activate();
  }
  ~PredictionHarness()
  {
    controller.deactivate();
    controller.cleanup();
    map->on_cleanup(rclcpp_lifecycle::State());
  }
  void clear()
  {
    auto * grid = map->getCostmap();
    std::fill(grid->getCharMap(), grid->getCharMap() + grid->getSizeInCellsX() *
      grid->getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
  }
  void updateTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.rotation = pose.pose.orientation;
    tf->setTransform(transform, "prediction_test", true);
  }
  void obstacle(double x, double y)
  {
    unsigned int mx, my;
    if (!map->getCostmap()->worldToMap(x, y, mx, my)) {
      throw std::runtime_error("obstacle outside map");
    }
    map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  void advance(const geometry_msgs::msg::Twist & command)
  {
    const auto yaw = tf2::getYaw(pose.pose.orientation);
    geometry_msgs::msg::Pose2D start;
    start.x = pose.pose.position.x; start.y = pose.pose.position.y; start.theta = yaw;
    const auto end = controller.projectMotion(start, command, 1.0 / 30, 0, 0.05, 0.6).back();
    pose = worldPose(end.x, end.y, end.theta);
    updateTransform();
  }
  nav_msgs::msg::Path straight(double x = 4.0, double y = 0.0, double yaw = 0.0)
  {
    nav_msgs::msg::Path result;
    result.header.frame_id = "odom";
    for (int i = 0; i <= 80; ++i) {
      result.poses.push_back(worldPose(x * i / 80, y * i / 80, yaw));
    }
    return result;
  }
};
