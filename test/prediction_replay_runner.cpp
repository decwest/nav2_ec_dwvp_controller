// Copyright (c) 2026 Fumiya Ohnishi
// Licensed under the Apache License, Version 2.0.
// Offline snapshot comparison. No subscriptions, actions or actuator publishers.

#include <iomanip>
#include <iostream>

#include "prediction_harness.hpp"
#include "rcutils/logging.h"

class ReplayGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(
    geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist &) override
  {
    pose.position.x = pose.position.y = 0.25;
    pose.orientation = worldPose(0, 0, 0.25).pose.orientation;
    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_ERROR);
  // Optional positional argument: stop margin in metres (default: 0.3).
  const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  double stop_margin = 0.3;
  try {
    if (arguments.size() > 2) {throw std::invalid_argument("too many arguments");}
    if (arguments.size() == 2) {
      size_t consumed = 0;
      stop_margin = std::stod(arguments[1], &consumed);
      if (consumed != arguments[1].size() || !std::isfinite(stop_margin) || stop_margin < 0) {
        throw std::invalid_argument("invalid stop margin");
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "Usage: prediction_replay_runner [nonnegative_stop_margin_m]: " <<
      error.what() << '\n';
    rclcpp::shutdown();
    return 2;
  }
  {
    PredictionHarness h(stop_margin);
    ReplayGoalChecker goal_checker;
    auto & c = h.controller;
    c.enable(true);
    c.costRegulation(true);
    std::string id;
    unsigned int width, height;
    double resolution, ox, oy, x, y, yaw, vx, vy, w, mx, my, mw;
    size_t count;
    while (std::cin >> id >> width >> height >> resolution >> ox >> oy >>
      x >> y >> yaw >> vx >> vy >> w >> mx >> my >> mw >> count)
    {
      auto * grid = h.map->getCostmap();
      grid->resizeMap(width, height, resolution, ox, oy);
      for (size_t i = 0; i < static_cast<size_t>(width) * height; ++i) {
        int cost;
        if (!(std::cin >> cost)) {return 2;}
        grid->getCharMap()[i] = static_cast<unsigned char>(cost);
      }
      nav_msgs::msg::Path path;
      path.header.frame_id = "odom";
      for (size_t i = 0; i < count; ++i) {
        double px, py, heading;
        if (!(std::cin >> px >> py >> heading)) {return 2;}
        path.poses.push_back(worldPose(px, py, heading));
      }
      h.pose = worldPose(x, y, yaw);
      h.updateTransform();
      const auto feedback = velocity(vx, vy, w);
      const auto measured = velocity(mx, my, mw);
      c.reset();
      c.setPlan(path);
      c.seed(feedback);
      bool legacy = false;
      std::string status = "clear", event = "unavailable", rejection = "none";
      double distance = std::numeric_limits<double>::infinity(), scale = 1.0;
      geometry_msgs::msg::Twist output;
      size_t steps = 0;
      double elapsed_ms = 0.0;
      try {
        const auto first = c.firstStep(h.pose, feedback, 0.25);
        legacy = c.isCollisionImminentOmni(h.pose, first.command,
          std::hypot(first.preview.pose.pose.position.x, first.preview.pose.pose.position.y));
        const auto context = c.makePredictionContext(h.pose, feedback, measured, 0.25);
        const auto nominal = c.predictTracking(context, first);
        event = nominal.event;
        distance = nominal.event_distance;
        steps = nominal.steps;
        if (event == "collision" || event == "map_boundary") {
          const auto limited = c.predictTracking(context, first,
            std::max(0.0, distance - stop_margin));
          scale = limited.first_scale;
        }
        c.seed(feedback);
        const auto start = std::chrono::steady_clock::now();
        try {
          output = c.computeVelocityCommands(h.pose, measured, &goal_checker).twist;
          if (scale < 1.0) {status = "limited";}
        } catch (const nav2_core::ControllerException & error) {
          status = "rejected";
          rejection = error.what();
          std::replace(rejection.begin(), rejection.end(), ' ', '_');
        }
        elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      } catch (const nav2_core::ControllerException & error) {
        status = "input_invalid";
        rejection = error.what();
        std::replace(rejection.begin(), rejection.end(), ' ', '_');
      }
      std::cout << std::setprecision(12) << id << ' ' << legacy << ' ' << status << ' ' <<
        event << ' ' << distance << ' ' << scale << ' ' << steps << ' ' << elapsed_ms << ' ' <<
        output.linear.x << ' ' << output.linear.y << ' ' << output.angular.z << ' ' <<
        rejection << '\n';
    }
  }
  rclcpp::shutdown();
  return 0;
}
