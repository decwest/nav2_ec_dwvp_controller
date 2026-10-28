// Copyright (c) 2026 Fumiya Ohnishi
// Licensed under the Apache License, Version 2.0.

#include "gtest/gtest.h"
#include "prediction_harness.hpp"

#include <thread>

class PredictionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {int argc = 0; rclcpp::init(argc, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(PredictionTest, HorizonAndStoppingEnvelope)
{
  PredictionHarness h;
  auto & c = h.controller;
  EXPECT_DOUBLE_EQ(c.predictionHorizon(velocity(0, 0, 0), velocity(0, 0, 0)), 3.0);
  EXPECT_DOUBLE_EQ(c.predictionHorizon(velocity(0, 0, 0), velocity(0, 0, 0.75)), 3.5);
  EXPECT_NEAR(c.stoppingDistance(velocity(0.5, 0, 0), 0.6), 0.25 + 0.5 / 30, 1e-12);
  for (const auto & u : {velocity(0.5, 0, 0), velocity(-0.2, 0.3, -0.5), velocity(0, 0, 0.5)}) {
    EXPECT_DOUBLE_EQ(c.brakingScale(u, 0.6, 10.0), 1.0);
    const double scale = c.brakingScale(u, 0.6, 0.03);
    const auto limited = velocity(u.linear.x * scale, u.linear.y * scale, u.angular.z * scale);
    EXPECT_NEAR(c.stoppingDistance(limited, 0.6), 0.03, 1e-12);
  }
}

TEST_F(PredictionTest, RolloutMatchesIndependentControllerAndDoesNotChangeState)
{
  PredictionHarness h;
  auto path = h.straight();
  for (auto & p : path.poses) {
    const double x = p.pose.position.x;
    p.pose.position.y = 0.4 * std::sin(x);
    p.pose.orientation = worldPose(0, 0, 0.3 * std::cos(x)).pose.orientation;
  }
  auto & c = h.controller;
  c.setPlan(path);
  auto feedback = velocity(0.15, 0.02, 0.0);
  c.seed(feedback);
  const auto first = c.firstStep(h.pose, feedback);
  const auto context = c.makePredictionContext(h.pose, feedback, feedback, 0.0);
  const auto size = c.pathSize();
  const auto result = c.predictTracking(context, first);
  ASSERT_EQ(result.event, "clear");
  ASSERT_EQ(result.steps, 90u);
  EXPECT_EQ(c.pathSize(), size);
  EXPECT_TRUE(c.commandValid());
  EXPECT_DOUBLE_EQ(c.command().linear.x, feedback.linear.x);
  for (size_t i = 0; i < result.commands.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_NEAR(result.control_poses[i].pose.position.x, h.pose.pose.position.x, 1e-9);
    EXPECT_NEAR(result.control_poses[i].pose.position.y, h.pose.pose.position.y, 1e-9);
    const auto actual = c.computeVelocityCommands(h.pose, feedback, nullptr).twist;
    EXPECT_NEAR(actual.linear.x, result.commands[i].linear.x, 1e-9);
    EXPECT_NEAR(actual.linear.y, result.commands[i].linear.y, 1e-9);
    EXPECT_NEAR(actual.angular.z, result.commands[i].angular.z, 1e-9);
    EXPECT_FALSE(c.evaluateVelocityConstraints(actual, feedback));
    feedback = actual;
    h.advance(actual);
  }
}

class PredictionMarginTest : public PredictionTest, public ::testing::WithParamInterface<double> {};

INSTANTIATE_TEST_SUITE_P(StopMargins, PredictionMarginTest, ::testing::Values(0.2, 0.3));

TEST_P(PredictionMarginTest, DistantCollisionKeepsFirstCommandThenStopsSmoothlyAndResumes)
{
  const double margin = GetParam();
  PredictionHarness h(margin);
  auto & c = h.controller;
  c.setPlan(h.straight());
  // A wall, to avoid relying on perimeter sampling of a single interior cell.
  for (double y = -1; y <= 1; y += 0.025) {
    h.obstacle(1.4, y);
  }
  auto feedback = velocity(0.3, 0, 0);
  const auto first = c.firstStep(h.pose, feedback);
  auto context = c.makePredictionContext(h.pose, feedback, feedback, 0.0);
  const auto nominal = c.predictTracking(context, first);
  ASSERT_EQ(nominal.event, "collision");
  EXPECT_FALSE(c.isCollisionImminentOmni(h.pose, first.command,
      std::hypot(first.preview.pose.pose.position.x, first.preview.pose.pose.position.y)));
  ASSERT_GT(nominal.event_distance, margin + c.stoppingDistance(first.command, context.radius));
  const auto selected = c.limitPredictedCollision(context, first);
  EXPECT_DOUBLE_EQ(selected.linear.x, first.command.linear.x);
  c.enable(true);
  c.seed(feedback);
  double previous = feedback.linear.x;
  int reductions = 0;
  for (int i = 0; i < 240; ++i) {
    SCOPED_TRACE(::testing::Message() << "cycle=" << i << " x=" <<
      h.pose.pose.position.x << " v=" << feedback.linear.x);
    ASSERT_NO_THROW(feedback = c.computeVelocityCommands(h.pose, feedback, nullptr).twist);
    EXPECT_LE(std::abs(feedback.linear.x - previous), 0.5 / 30 + 1e-9);
    if (reductions > 0) {
      EXPECT_LE(feedback.linear.x, previous + 1e-4);  // No stop-boundary chatter.
    }
    if (feedback.linear.x < previous - 1e-7) {++reductions;}
    previous = feedback.linear.x;
    h.advance(feedback);
  }
  EXPECT_GT(reductions, 3);
  EXPECT_LT(std::abs(feedback.linear.x), 1e-6);
  EXPECT_LE(h.pose.pose.position.x + 0.42, 1.4 - margin + 0.05);
  h.clear();
  const auto resumed = c.computeVelocityCommands(h.pose, feedback, nullptr).twist;
  EXPECT_GT(resumed.linear.x, 0.0);
  EXPECT_LE(resumed.linear.x, feedback.linear.x + 0.5 / 30 + 1e-9);
}

TEST_F(PredictionTest, LateralReverseAndRotationDetectCollisions)
{
  PredictionHarness h;
  auto & c = h.controller;
  for (const auto & u : {velocity(0, 0.3, 0), velocity(-0.3, 0, 0), velocity(0, 0, 0.3)}) {
    h.clear();
    c.reset();
    c.setPlan(h.straight(u.linear.x * 10, u.linear.y * 10, u.angular.z * 5));
    if (u.linear.y > 0) {h.obstacle(0.0, 0.8);} else if (u.linear.x < 0) {
      h.obstacle(-0.9, 0.0);
    } else {h.obstacle(-0.25, -0.5);}
    const auto first = c.firstStep(h.pose, u);
    const auto result = c.predictTracking(c.makePredictionContext(h.pose, u, u, 0.0), first);
    EXPECT_EQ(result.event, "collision");
    EXPECT_GT(result.event_distance, 0.0);
  }
}

TEST_F(PredictionTest, ImminentCollisionAndStepLimitFailClosed)
{
  PredictionHarness h;
  auto & c = h.controller;
  c.setPlan(h.straight());
  const auto speed = velocity(0.35, 0, 0);
  const auto first = c.firstStep(h.pose, speed);
  const auto context = c.makePredictionContext(h.pose, speed, speed, 0.0);
  h.obstacle(0.5, 0.0);
  EXPECT_THROW(c.limitPredictedCollision(context, first), nav2_core::NoValidControl);
  h.clear();
  c.maxSteps(89);
  EXPECT_EQ(c.predictTracking(context, first).event, "step_limit");
  EXPECT_THROW(c.limitPredictedCollision(context, first), nav2_core::NoValidControl);
}

TEST_F(PredictionTest, MapBoundaryIsAStoppingBoundaryNotImmediateRejection)
{
  PredictionHarness h;
  auto & c = h.controller;
  h.map->getCostmap()->resizeMap(80, 80, 0.05, -2.7, -2.0);
  h.clear();
  c.setPlan(h.straight());
  const auto u = velocity(0.3, 0, 0);
  const auto first = c.firstStep(h.pose, u);
  const auto context = c.makePredictionContext(h.pose, u, u, 0.0);
  EXPECT_EQ(c.predictTracking(context, first).event, "map_boundary");
  EXPECT_NO_THROW(c.limitPredictedCollision(context, first));
}

TEST_F(PredictionTest, GoalApproachRotationAndDuplicatePointsRemainFinite)
{
  PredictionHarness h;
  auto & c = h.controller;
  auto plan = h.straight(0.35, 0, 1.0);
  plan.poses.insert(plan.poses.begin() + 2, plan.poses[1]);
  c.setPlan(plan);
  const auto u = velocity(0.1, 0, 0);
  const auto first = c.firstStep(h.pose, u);
  const auto result = c.predictTracking(c.makePredictionContext(h.pose, u, u, 0.0), first);
  ASSERT_EQ(result.event, "clear");
  ASSERT_FALSE(result.commands.empty());
  EXPECT_LT(std::abs(result.commands.back().linear.x), 1e-6);
  EXPECT_TRUE(std::isfinite(result.commands.back().angular.z));
  EXPECT_GT(tf2::getYaw(result.path.poses.back().pose.orientation), 0.5);
}

TEST_F(PredictionTest, CornerUsesFutureCommandsRatherThanHoldingFirstCommand)
{
  PredictionHarness h;
  auto & c = h.controller;
  auto plan = h.straight(0.25);
  for (int i = 1; i <= 60; ++i) {
    plan.poses.push_back(worldPose(0.25, i * 0.05, M_PI_2));
  }
  c.setPlan(plan);
  const auto speed = velocity(0.3, 0, 0);
  const auto first = c.firstStep(h.pose, speed);
  const auto context = c.makePredictionContext(h.pose, speed, speed, 0.0);
  bool found = false;
  // Fixed wall-cell lattice beside this corner: find a cell in the constant
  // command's swept footprint which the feedback trajectory does not enter.
  for (double x = 0.5; x < 0.9 && !found; x += 0.05) {
    for (double y = -0.3; y < 0.3 && !found; y += 0.05) {
      h.clear(); h.obstacle(x, y);
      if (c.isCollisionImminentOmni(h.pose, first.command, 1.0) &&
        c.predictTracking(context, first).event == "clear")
      {
        found = true;
        EXPECT_NO_THROW(c.limitPredictedCollision(context, first));
      }
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(PredictionTest, RotationReversalAccumulatesTravelAndGovernedTrajectoryIsChecked)
{
  PredictionHarness h;
  auto & c = h.controller;
  // Clockwise demand with initially counterclockwise motion. Net yaw is smaller
  // than the amount actually swept, so using final-minus-initial yaw is unsafe.
  c.setPlan(h.straight(0, 0, -0.8));
  const auto speed = velocity(0, 0, 0.3);
  const auto first = c.firstStep(h.pose, speed);
  const auto context = c.makePredictionContext(h.pose, speed, speed, 0.0);
  const auto nominal = c.predictTracking(context, first);
  ASSERT_EQ(nominal.event, "clear");
  EXPECT_GT(nominal.distance,
    context.radius * std::abs(tf2::getYaw(nominal.path.poses.back().pose.orientation)) + 0.02);
  const auto stopped = c.predictTracking(context, first, 0.08);
  ASSERT_EQ(stopped.event, "clear");
  EXPECT_LE(stopped.distance, 0.08 + 1e-8);
  // A new obstacle on the verification trajectory must not be accepted merely
  // because a stopping-distance scalar was satisfied.
  const auto & p = stopped.path.poses.back().pose;
  const double yaw = tf2::getYaw(p.orientation);
  h.obstacle(p.position.x + 0.42 * std::cos(yaw) - 0.31 * std::sin(yaw),
    p.position.y + 0.42 * std::sin(yaw) + 0.31 * std::cos(yaw));
  EXPECT_NE(c.predictTracking(context, first, 0.08).event, "clear");
}

TEST_F(PredictionTest, MissingTransformAndInvalidPredictionSettingsAreRejected)
{
  PredictionHarness h;
  auto plan = h.straight();
  h.controller.setPlan(plan);
  auto invalid = h.pose;
  invalid.header.frame_id = "missing_frame";
  EXPECT_THROW(h.controller.makePredictionContext(invalid, velocity(0, 0, 0),
      velocity(0, 0, 0), 0.0), nav2_core::ControllerTFError);
  EXPECT_THROW(h.controller.predictionHorizon(velocity(NAN, 0, 0), velocity(0, 0, 0)),
    nav2_core::NoValidControl);
  for (const auto & value : {std::string("typo"), std::string("closed_loop")}) {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("invalid_prediction_config");
    node->declare_parameter("FollowPath.collision_prediction_mode", value);
    if (value == "closed_loop") {
      node->declare_parameter("FollowPath.collision_stop_margin", -0.1);
    }
    PredictionController other;
    EXPECT_THROW(other.configure(node, "FollowPath", h.tf, h.map), std::invalid_argument);
  }
}

TEST_F(PredictionTest, DiagnosticsAndSnapshotDescribeLimitedCandidate)
{
  PredictionHarness h;
  h.pose = worldPose(0.635, 0, 0);
  h.updateTransform();
  auto & c = h.controller;
  c.setPlan(h.straight());
  for (double y = -1; y <= 1; y += 0.025) {
    h.obstacle(1.4, y);
  }
  const auto speed = velocity(0.2, 0, 0);
  const auto first = c.firstStep(h.pose, speed);
  const auto context = c.makePredictionContext(h.pose, speed, speed, 0.0);
  auto listener = std::make_shared<rclcpp::Node>("prediction_diagnostics_test");
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic;
  nav2_msgs::msg::Costmap::SharedPtr snapshot;
  nav_msgs::msg::Path::SharedPtr nominal;
  auto diag_sub = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "FollowPath/collision_prediction", 10,
    [&diagnostic](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {diagnostic = msg;});
  auto map_sub = listener->create_subscription<nav2_msgs::msg::Costmap>(
    "FollowPath/collision_costmap", 10,
    [&snapshot](nav2_msgs::msg::Costmap::SharedPtr msg) {snapshot = msg;});
  auto path_sub = listener->create_subscription<nav_msgs::msg::Path>(
    "FollowPath/nominal_collision_arc", 10,
    [&nominal](nav_msgs::msg::Path::SharedPtr msg) {nominal = msg;});
  geometry_msgs::msg::Twist selected;
  for (int i = 0; i < 200 && (!diagnostic || !snapshot || !nominal); ++i) {
    selected = c.limitPredictedCollision(context, first);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(diagnostic);
  ASSERT_TRUE(snapshot);
  ASSERT_TRUE(nominal);
  ASSERT_EQ(diagnostic->status.size(), 1u);
  EXPECT_EQ(diagnostic->status[0].message, "limited");
  EXPECT_LT(selected.linear.x, first.command.linear.x);
  EXPECT_FALSE(c.evaluateVelocityConstraints(selected, speed));
  EXPECT_EQ(diagnostic->header.stamp, nominal->header.stamp);
  EXPECT_EQ(diagnostic->header.stamp, snapshot->header.stamp);
  EXPECT_EQ(snapshot->data.size(), 40000u);
}

TEST_F(PredictionTest, NestedShimNamespaceUsesPredictionOverrides)
{
  PredictionHarness h;
  const std::string name = "FollowPath.primary_controller";
  h.node->declare_parameter(name + ".collision_prediction_mode", "closed_loop");
  h.node->declare_parameter(name + ".collision_prediction_time_buffer", 0.7);
  PredictionController nested;
  nested.configure(h.node, name, h.tf, h.map);
  EXPECT_EQ(nested.mode(), "closed_loop");
  EXPECT_NEAR(nested.predictionHorizon(velocity(0, 0, 0), velocity(0, 0, 0)), 1.3, 1e-12);
  nested.cleanup();
}
