// Copyright (c) 2026 Fumiya Ohnishi
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_ec_dwvp_controller/raw_cost_heuristics.hpp"
#include "prediction_harness.hpp"

namespace heuristics = nav2_ec_dwvp_controller::raw_cost_heuristics;

nav_msgs::msg::Path localPath(std::initializer_list<std::pair<double, double>> points)
{
  nav_msgs::msg::Path result;
  result.header.frame_id = "base_link";
  for (const auto & point : points) {
    result.poses.push_back(heuristics::makeSamplePose(result.header, point.first, point.second));
  }
  return result;
}

TEST(RawCostHeuristics, CostValuesAndReverseSign)
{
  const std::vector<std::pair<unsigned char, double>> cases {
    {0, 0.8}, {80, 0.5786561264822134}, {160, 0.3573122529644269},
    {253, 0.1}, {254, 0.1}, {255, 0.8}};
  for (const auto & item : cases) {
    SCOPED_TRACE(static_cast<int>(item.first));
    EXPECT_NEAR(heuristics::rawCostConstraint(0.8, item.first, 0.1), item.second, 1e-12);
    EXPECT_NEAR(heuristics::rawCostConstraint(-0.8, item.first, 0.1), -item.second, 1e-12);
    EXPECT_DOUBLE_EQ(heuristics::rawCostConstraint(0.0, item.first, 0.1), 0.0);
    EXPECT_DOUBLE_EQ(heuristics::rawCostConstraint(0.08, item.first, 0.1), 0.08);
  }
}

TEST(RawCostHeuristics, ZeroLookaheadAndEmptyOrShortPaths)
{
  const auto path = localPath({{0.1, 0.0}});
  for (const auto & samples : {
    heuristics::sampleReferencePath(path, 0.0, 0.05),
    heuristics::sampleReferencePath(localPath({}), 1.0, 0.05),
    heuristics::sampleReferencePath(path, 1.0, 0.25)})
  {
    ASSERT_EQ(samples.size(), 1u);
    EXPECT_DOUBLE_EQ(samples.front().pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(samples.front().pose.position.y, 0.0);
    EXPECT_EQ(samples.front().header.frame_id, "base_link");
  }
}

TEST(RawCostHeuristics, SparsePolylineInterpolationAndDuplicatePoints)
{
  const auto path = localPath({{0, 0}, {0, 0}, {0.4, 0}, {0.4, 0}, {0.4, 0.4}});
  const auto samples = heuristics::sampleReferencePath(path, 0.65, 0.25);
  ASSERT_EQ(samples.size(), 3u);
  EXPECT_NEAR(samples[1].pose.position.x, 0.25, 1e-12);
  EXPECT_NEAR(samples[1].pose.position.y, 0.0, 1e-12);
  EXPECT_NEAR(samples[2].pose.position.x, 0.4, 1e-12);
  EXPECT_NEAR(samples[2].pose.position.y, 0.1, 1e-12);
  // Match RPP: a non-aligned horizon or final path vertex is not an extra sample.
  const auto short_samples = heuristics::sampleReferencePath(path, 2.0, 0.25);
  ASSERT_EQ(short_samples.size(), 4u);
  EXPECT_NEAR(short_samples.back().pose.position.y, 0.35, 1e-12);
}

TEST(RawCostHeuristics, RobotToPathConnectorLateralAndReverseSampling)
{
  for (const auto & direction : {std::make_pair(0.0, 1.0), std::make_pair(-1.0, 0.0),
      std::make_pair(0.6, 0.8)})
  {
    const auto path = localPath({direction});
    const auto samples = heuristics::sampleReferencePath(path, 0.5, 0.25);
    ASSERT_EQ(samples.size(), 3u);
    for (size_t i = 0; i < samples.size(); ++i) {
      EXPECT_NEAR(samples[i].pose.position.x, 0.25 * i * direction.first, 1e-12);
      EXPECT_NEAR(samples[i].pose.position.y, 0.25 * i * direction.second, 1e-12);
    }
    const auto coarse = heuristics::sampleReferencePath(path, 0.5, 1.0);
    ASSERT_EQ(coarse.size(), 2u);
    EXPECT_NEAR(coarse.back().pose.position.x, 0.5 * direction.first, 1e-12);
  }
}

class RawCostTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {int argc = 0; rclcpp::init(argc, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static void fill(PredictionHarness & h, unsigned char cost)
  {
    auto * grid = h.map->getCostmap();
    std::fill(grid->getCharMap(), grid->getCharMap() +
      grid->getSizeInCellsX() * grid->getSizeInCellsY(), cost);
  }

  static void cell(PredictionHarness & h, double x, double y, unsigned char cost)
  {
    unsigned int mx, my;
    ASSERT_TRUE(h.map->getCostmap()->worldToMap(x, y, mx, my));
    h.map->getCostmap()->setCost(mx, my, cost);
  }
};

TEST_F(RawCostTest, DefaultsAndNestedParametersAreReadAtConfigure)
{
  PredictionHarness h;
  EXPECT_EQ(h.controller.rawCostSettings(), std::make_tuple(false, 0.0, 0.05, 0.1));
  const std::string name = "FollowPath.primary_controller";
  h.node->declare_parameter(name + ".use_raw_cost_regulated_linear_velocity_scaling", true);
  h.node->declare_parameter(name + ".raw_cost_lookahead_dist", 0.6);
  h.node->declare_parameter(name + ".raw_cost_sample_step", 0.125);
  h.node->declare_parameter(name + ".raw_cost_min_linear_velocity", 0.02);
  PredictionController nested;
  nested.configure(h.node, name, h.tf, h.map);
  EXPECT_EQ(nested.rawCostSettings(), std::make_tuple(true, 0.6, 0.125, 0.02));
  nested.cleanup();
  EXPECT_EQ(h.controller.rawCostSettings(), std::make_tuple(false, 0.0, 0.05, 0.1));
}

TEST_F(RawCostTest, InvalidRawCostParametersAreRejected)
{
  PredictionHarness h;
  for (const auto & name : {"raw_cost_lookahead_dist", "raw_cost_sample_step",
      "raw_cost_min_linear_velocity"})
  {
    for (double value : {-0.1, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN(), 0.0})
    {
      SCOPED_TRACE(name);
      SCOPED_TRACE(value);
      auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("invalid_raw_cost");
      node->declare_parameter(std::string("FollowPath.") + name, value);
      PredictionController other;
      if (value == 0.0 && std::string(name) != "raw_cost_sample_step") {
        EXPECT_NO_THROW(other.configure(node, "FollowPath", h.tf, h.map));
      } else {
        EXPECT_THROW(other.configure(node, "FollowPath", h.tf, h.map), std::invalid_argument);
      }
      other.cleanup();
    }
  }
}

TEST_F(RawCostTest, CombinedMapCostsRegulateSpeedWithoutInflationScaling)
{
  PredictionHarness h;
  auto & c = h.controller;
  c.rawCostRegulation(true);
  c.approachRegulation(0.0);
  const auto local = localPath({{0, 0}, {2, 0}});
  for (bool inflation : {false, true}) {
    c.costRegulation(inflation);
    for (unsigned char cost : {0, 80, 160, 253, 254, 255}) {
      SCOPED_TRACE(static_cast<int>(cost));
      fill(h, cost);
      const double expected = cost == 0 || cost == 255 ? 0.5 :
        0.5 - 0.4 * std::min(static_cast<int>(cost), 253) / 253.0;
      EXPECT_NEAR(c.computeRegulatedSpeed(local, velocity(0, 0, 0), 253, h.pose), expected, 1e-12);
      EXPECT_EQ(c.costRegulationEnabled(), inflation);
    }
  }
}

TEST_F(RawCostTest, DisabledModeRetainsInheritedRegulation)
{
  PredictionHarness h;
  auto & c = h.controller;
  c.costRegulation(true);
  const auto local = localPath({{0, 0}, {2, 0}});
  const auto feedback = velocity(0, 0, 0);
  const double before = c.computeRegulatedSpeed(local, feedback, 80, h.pose);
  fill(h, 253);
  EXPECT_DOUBLE_EQ(c.computeRegulatedSpeed(local, feedback, 80, h.pose), before);
  c.rawCostRegulation(true);
  EXPECT_NEAR(c.computeRegulatedSpeed(local, feedback, 80, h.pose), 0.1, 1e-12);
  EXPECT_TRUE(c.costRegulationEnabled());
  c.rawCostRegulation(false);
  EXPECT_DOUBLE_EQ(c.computeRegulatedSpeed(local, feedback, 80, h.pose), before);
}

TEST_F(RawCostTest, ApproachPrecedesRawCostAndSpeedLimitsStillApply)
{
  PredictionHarness h;
  auto & c = h.controller;
  c.rawCostRegulation(true);
  fill(h, 80);
  const auto feedback = velocity(0, 0, 0);
  EXPECT_NEAR(c.computeRegulatedSpeed(localPath({{0, 0}, {0.3, 0}}), feedback, 80, h.pose),
    0.25 - 0.15 * 80.0 / 253.0, 1e-12);
  c.approachRegulation(0.0);
  const auto local = localPath({{0, 0}, {2, 0}});
  c.setSpeedLimit(0.2, false);
  EXPECT_NEAR(c.computeRegulatedSpeed(local, feedback, 80, h.pose),
    0.2 - 0.1 * 80.0 / 253.0, 1e-12);
  c.setSpeedLimit(50, true);
  EXPECT_NEAR(c.computeRegulatedSpeed(local, feedback, 80, h.pose),
    0.25 - 0.15 * 80.0 / 253.0, 1e-12);
  c.setSpeedLimit(0.08, false);
  fill(h, 253);
  EXPECT_DOUBLE_EQ(c.computeRegulatedSpeed(local, feedback, 253, h.pose), 0.08);
  c.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_NEAR(c.computeRegulatedSpeed(local, feedback, 253, h.pose), 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(c.applyRawCostConstraints(local, h.pose, 0.0), 0.0);
}

TEST_F(RawCostTest, SamplesUseExplicitRobotPoseAndIgnoreUnknownAndDistantCosts)
{
  PredictionHarness h;
  auto & c = h.controller;
  c.rawCostRegulation(true, 0.75, 0.25);
  const auto robot = worldPose(1.025, -0.975, M_PI_2);
  const auto local = localPath({{0, 0}, {2, 0}});
  cell(h, 1.025, -0.975, 80);
  cell(h, 1.025, -0.725, 255);
  cell(h, 1.025, -0.475, 160);
  cell(h, 1.025, 0.025, 253);  // Beyond the raw cost horizon.
  // The live base_link remains at (0, 0, 0); it must not locate these samples.
  EXPECT_NEAR(c.applyRawCostConstraints(local, robot, 0.5), 0.5 - 0.4 * 160.0 / 253.0, 1e-12);
  c.rawCostRegulation(true, 0.0);
  EXPECT_NEAR(c.applyRawCostConstraints(local, robot, 0.5), 0.5 - 0.4 * 80.0 / 253.0, 1e-12);
  c.rawCostRegulation(true, 0.75, 0.25);
  cell(h, 4.825, 0.025, 160);
  EXPECT_NEAR(c.applyRawCostConstraints(local, worldPose(4.825, 0.025, 0), 0.5),
    0.5 - 0.4 * 160.0 / 253.0, 1e-12);
  EXPECT_DOUBLE_EQ(c.applyRawCostConstraints(local, worldPose(10, 10, 0), 0.5), 0.5);
}

TEST_F(RawCostTest, RobotPoseTransformsIntoCostmapFrameAndMissingTransformSkipsRegulation)
{
  PredictionHarness h;
  h.controller.rawCostRegulation(true, 0.5, 0.25);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "reference";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 1.0;
  transform.transform.rotation = worldPose(0, 0, M_PI_2).pose.orientation;
  ASSERT_TRUE(h.tf->setTransform(transform, "raw_cost_test", true));
  auto robot = worldPose(0.225, 0.125, 0);
  robot.header.frame_id = "reference";
  cell(h, 0.875, 1.475, 160);
  const auto local = localPath({{0, 0}, {2, 0}});
  EXPECT_NEAR(h.controller.applyRawCostConstraints(local, robot, 0.5),
    0.5 - 0.4 * 160.0 / 253.0, 1e-12);
  robot.header.frame_id = "missing_frame";
  EXPECT_DOUBLE_EQ(h.controller.applyRawCostConstraints(local, robot, 0.5), 0.5);
}

TEST_F(RawCostTest, ForwardLateralAndReverseCommandsSlowAndResumeWithinAccelerationLimits)
{
  for (const auto & direction : {std::make_pair(4.0, 0.0), std::make_pair(0.0, 4.0),
      std::make_pair(-4.0, 0.0)})
  {
    SCOPED_TRACE(::testing::PrintToString(direction));
    PredictionHarness h;
    auto & c = h.controller;
    c.rawCostRegulation(true);
    c.approachRegulation(0.0);
    c.setPlan(h.straight(direction.first, direction.second));
    auto feedback = velocity(0, 0, 0);
    std::vector<double> settled_speeds;
    for (unsigned char cost : {0, 80, 160, 253, 0}) {
      fill(h, cost);
      for (int i = 0; i < 40; ++i) {
        const auto command = c.computeVelocityCommands(h.pose, feedback, nullptr).twist;
        EXPECT_FALSE(c.evaluateVelocityConstraints(command, feedback));
        EXPECT_TRUE(std::isfinite(command.angular.z));
        EXPECT_NEAR(command.angular.z, 0.0, 1e-9);
        EXPECT_NEAR(command.linear.x * direction.second - command.linear.y * direction.first,
          0.0, 1e-9);
        EXPECT_GE(command.linear.x * direction.first + command.linear.y * direction.second, 0.0);
        h.advance(command);
        feedback = command;
      }
      const double speed = std::hypot(feedback.linear.x, feedback.linear.y);
      settled_speeds.push_back(speed);
      // The enclosing translation cap contracts both axis limits: an axis-aligned
      // command has magnitude v_reg / sqrt(2), not raw_cost_min_linear_velocity.
      EXPECT_NEAR(speed, (0.5 - 0.4 * cost / 253.0) / std::sqrt(2.0), 1e-8);
    }
    EXPECT_GT(settled_speeds[0], settled_speeds[1]);
    EXPECT_GT(settled_speeds[1], settled_speeds[2]);
    EXPECT_GT(settled_speeds[2], settled_speeds[3]);
    EXPECT_NEAR(settled_speeds[4], settled_speeds[0], 1e-9);
  }
}

TEST_F(RawCostTest, TerminalRotationAndCollisionCheckingRemainActive)
{
  PredictionHarness h;
  auto & c = h.controller;
  c.rawCostRegulation(true);
  c.goalTolerance(0.1);
  c.setPlan(h.straight(0.0, 0.0, 0.7));
  fill(h, 253);
  const auto command = c.computeVelocityCommands(h.pose, velocity(0, 0, 0), nullptr).twist;
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
  EXPECT_GT(command.angular.z, 0.0);
  h.clear();
  c.reset();
  c.enable(true);
  c.setPlan(h.straight());
  h.obstacle(0.42, 0.31);  // The polygon collision checker samples the footprint boundary.
  EXPECT_THROW(c.computeVelocityCommands(h.pose, velocity(0, 0, 0), nullptr),
    nav2_core::NoValidControl);
}

TEST_F(RawCostTest, PredictionMatchesIndependentControlAcrossSpatialCostBands)
{
  for (double lookahead : {0.0, 0.4}) {
    SCOPED_TRACE(lookahead);
    PredictionHarness h;
    auto & c = h.controller;
    c.rawCostRegulation(true, lookahead);
    c.costRegulation(true);
    h.pose = worldPose(-1.013, 0.517, 0.7);
    h.updateTransform();
    const auto start = h.pose;
    const double cosine = std::cos(0.7);
    const double sine = std::sin(0.7);
    auto path = h.straight();
    for (auto & p : path.poses) {
      const double x = p.pose.position.x;
      const double y = 0.15 * std::sin(2.0 * x);
      p = worldPose(start.pose.position.x + cosine * x - sine * y,
        start.pose.position.y + sine * x + cosine * y, 0.7 + 0.2 * std::sin(x));
    }
    auto * grid = h.map->getCostmap();
    for (unsigned int mx = 0; mx < grid->getSizeInCellsX(); ++mx) {
      for (unsigned int my = 0; my < grid->getSizeInCellsY(); ++my) {
        double x, y;
        grid->mapToWorld(mx, my, x, y);
        const double along = cosine * (x - start.pose.position.x) +
          sine * (y - start.pose.position.y);
        grid->setCost(mx, my, along >= 0.35 && along < 0.55 ? 160 :
          along >= 0.2 && along < 0.35 ? 80 : 0);
      }
    }
    c.setPlan(path);
    auto feedback = velocity(0.3, 0.02, 0.0);
    c.seed(feedback);
    const auto first = c.firstStep(h.pose, feedback);
    const auto context = c.makePredictionContext(h.pose, feedback, feedback, 0.0);
    const auto path_size = c.pathSize();
    const auto settings = c.rawCostSettings();
    const auto result = c.predictTracking(context, first);
    ASSERT_EQ(result.event, "clear");
    ASSERT_EQ(result.steps, 90u);
    EXPECT_EQ(c.pathSize(), path_size);
    EXPECT_EQ(c.rawCostSettings(), settings);
    EXPECT_TRUE(c.costRegulationEnabled());
    EXPECT_TRUE(c.commandValid());
    EXPECT_EQ(c.command(), feedback);
    double minimum_speed = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < result.commands.size(); ++i) {
      SCOPED_TRACE(i);
      EXPECT_NEAR(result.control_poses[i].pose.position.x, h.pose.pose.position.x, 1e-9);
      EXPECT_NEAR(result.control_poses[i].pose.position.y, h.pose.pose.position.y, 1e-9);
      const auto actual = c.computeVelocityCommands(h.pose, feedback, nullptr).twist;
      EXPECT_NEAR(actual.linear.x, result.commands[i].linear.x, 1e-9);
      EXPECT_NEAR(actual.linear.y, result.commands[i].linear.y, 1e-9);
      EXPECT_NEAR(actual.angular.z, result.commands[i].angular.z, 1e-9);
      EXPECT_FALSE(c.evaluateVelocityConstraints(actual, feedback));
      minimum_speed = std::min(minimum_speed, std::hypot(actual.linear.x, actual.linear.y));
      feedback = actual;
      h.advance(actual);
    }
    EXPECT_LT(minimum_speed, 0.25);
  }
}
