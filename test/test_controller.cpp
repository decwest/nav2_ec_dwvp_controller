// Copyright (c) 2026 Fumiya Ohnishi
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_omnidirectional_dwvp_controller/omnidirectional_dwvp_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pluginlib/class_loader.hpp"

using nav2_omnidirectional_dwvp_controller::OmnidirectionalDWVPController;

class TestController : public OmnidirectionalDWVPController
{
public:
  using OmnidirectionalDWVPController::DynamicWindowBounds3D;
  using OmnidirectionalDWVPController::applyRegulationToDynamicWindow;
  using OmnidirectionalDWVPController::clipVelocityToBounds;
  using OmnidirectionalDWVPController::computeDesiredVelocity;
  using OmnidirectionalDWVPController::computeDynamicWindow;
  using OmnidirectionalDWVPController::computeGoalVelocity;
  using OmnidirectionalDWVPController::computeOptimalVelocityWithinDynamicWindow;
  using OmnidirectionalDWVPController::computeRegulatedSpeed;
  using OmnidirectionalDWVPController::evaluateVelocityConstraints;
  using OmnidirectionalDWVPController::isCollisionImminentOmni;
  using OmnidirectionalDWVPController::maximumTranslationalSpeed;
  using OmnidirectionalDWVPController::projectMotion;

  TestController() {control_duration_ = 0.05;}
  void cost(bool enable) {use_cost_regulated_linear_velocity_scaling_ = enable;}
  void approach(double distance) {approach_velocity_scaling_dist_ = distance;}
  void nominal(double value) {desired_linear_vel_ = base_desired_linear_vel_ = value;}
  void clipped(bool value) {use_dynamic_window_vector_pursuit_ = !value;}
  void feedback(const std::string & value) {velocity_feedback_ = value;}
  bool commandValid() const {return last_command_valid_;}
  void seed(const geometry_msgs::msg::Twist & value)
  {
    last_command_velocity_ = value;
    last_command_valid_ = true;
  }
};

geometry_msgs::msg::Twist twist(double x, double y, double w)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = x;
  result.linear.y = y;
  result.angular.z = w;
  return result;
}

nav_msgs::msg::Path path(double length, double yaw = 0.0)
{
  nav_msgs::msg::Path result;
  result.header.frame_id = "base_link";
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header = result.header;
    p.pose.position.x = length * i / 20.0;
    p.pose.orientation.z = std::sin(yaw / 2.0);
    p.pose.orientation.w = std::cos(yaw / 2.0);
    result.poses.push_back(p);
  }
  return result;
}

class ControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dwvp_test");
    node->declare_parameter("controller_frequency", 20.0);
    node->declare_parameter("FollowPath.use_regulated_linear_velocity_scaling", false);
    node->declare_parameter("FollowPath.use_cost_regulated_linear_velocity_scaling", false);
    node->declare_parameter("FollowPath.approach_velocity_scaling_dist", 0.0);
    node->declare_parameter("FollowPath.regulated_linear_scaling_min_speed", 0.05);
    node->declare_parameter("FollowPath.use_collision_detection", false);
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("dwvp_test_map");
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    map->set_parameter(rclcpp::Parameter("global_frame", "base_link"));
    map->on_configure(rclcpp_lifecycle::State());
    map->getCostmap()->resizeMap(200, 200, 0.05, -5.0, -5.0);
    map->getCostmap()->resetMap(0, 0, 200, 200);
    // Explicit free cells; resetMap may use NO_INFORMATION.
    for (unsigned int x = 0; x < 200; ++x) {
      for (unsigned int y = 0; y < 200; ++y) {
        map->getCostmap()->setCost(x, y, nav2_costmap_2d::FREE_SPACE);
      }
    }
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    ctrl.configure(node, "FollowPath", tf, map);
    ctrl.activate();
    pose.header.frame_id = "base_link";
    pose.pose.orientation.w = 1.0;
  }
  void TearDown() override
  {
    ctrl.deactivate();
    ctrl.cleanup();
    map->on_cleanup(rclcpp_lifecycle::State());
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::shared_ptr<tf2_ros::Buffer> tf;
  TestController ctrl;
  geometry_msgs::msg::PoseStamped pose;
};

TEST(Solver, IntersectionAndZeroComponents)
{
  TestController c;
  TestController::DynamicWindowBounds3D b {-0.1, 0.2, -0.1, 0.1, -0.3, 0.3};
  auto u = c.computeOptimalVelocityWithinDynamicWindow(twist(1, 0, 1), b, true);
  EXPECT_DOUBLE_EQ(u.linear.x, 0.2);
  EXPECT_DOUBLE_EQ(u.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(u.angular.z, 0.2);
  u = c.computeOptimalVelocityWithinDynamicWindow(twist(1, 0, 1), b, false);
  EXPECT_DOUBLE_EQ(u.linear.x, 0.0);
  u = c.computeOptimalVelocityWithinDynamicWindow(twist(-1, -1, 0), b, true);
  EXPECT_DOUBLE_EQ(u.linear.x, -0.1);
  EXPECT_DOUBLE_EQ(u.linear.y, -0.1);
}

TEST(Solver, NonintersectionMatchesIndependentConvexOracle)
{
  TestController c;
  std::mt19937 rng(7301);
  std::uniform_real_distribution<double> sample(-1.0, 1.0);
  for (int trial = 0; trial < 1000; ++trial) {
    const auto current = twist(0.6 * sample(rng), 0.6 * sample(rng), sample(rng));
    const auto ray = twist(sample(rng), sample(rng), sample(rng));
    const auto b = c.computeDynamicWindow(current);
    const auto output = c.computeOptimalVelocityWithinDynamicWindow(ray, b, true);
    EXPECT_FALSE(c.evaluateVelocityConstraints(output, current));
    const std::array<double, 3> d {ray.linear.x, ray.linear.y, ray.angular.z};
    const std::array<double, 3> lo {b.min_vx, b.min_vy, b.min_w};
    const std::array<double, 3> hi {b.max_vx, b.max_vy, b.max_w};
    const auto objective = [&](double alpha) {
        double result = 0.0;
        for (size_t i = 0; i < 3; ++i) {
          const double delta = alpha * d[i] - std::clamp(alpha * d[i], lo[i], hi[i]);
          result += delta * delta;
        }
        return result;
      };
    double lower = 0.0;
    double upper = 1.0;
    for (size_t i = 0; i < 3; ++i) {
      if (std::abs(d[i]) > 1e-12) {
        upper = std::max(upper, std::max(std::abs(lo[i]), std::abs(hi[i])) / std::abs(d[i]) + 1.0);
      }
    }
    for (int i = 0; i < 150; ++i) {
      const double a = lower + (upper - lower) / 3.0;
      const double b0 = upper - (upper - lower) / 3.0;
      if (objective(a) < objective(b0)) {upper = b0;} else {lower = a;}
    }
    const double norm2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    const double alpha = std::max(
      0.0,
      (output.linear.x * d[0] + output.linear.y * d[1] + output.angular.z * d[2]) / norm2);
    const double residual = std::pow(output.linear.x - alpha * d[0], 2) +
      std::pow(output.linear.y - alpha * d[1], 2) + std::pow(output.angular.z - alpha * d[2], 2);
    EXPECT_NEAR(residual, objective((lower + upper) / 2.0), 1e-8);
    const auto scaled = c.computeOptimalVelocityWithinDynamicWindow(
      twist(0.37 * d[0], 0.37 * d[1], 0.37 * d[2]), b, true);
    EXPECT_NEAR(output.linear.x, scaled.linear.x, 1e-9);
    EXPECT_NEAR(output.linear.y, scaled.linear.y, 1e-9);
    EXPECT_NEAR(output.angular.z, scaled.angular.z, 1e-9);
  }
}

TEST(Solver, RegulatedWindowReachabilityAndSuddenCaps)
{
  TestController c;
  const auto current = twist(0.5, -0.4, 0.7);
  const auto b = c.computeDynamicWindow(current);
  const auto r = c.applyRegulationToDynamicWindow(b, 0.0);
  EXPECT_DOUBLE_EQ(r.min_vx, b.min_vx);
  EXPECT_DOUBLE_EQ(r.max_vx, b.min_vx);
  EXPECT_DOUBLE_EQ(r.min_vy, b.max_vy);
  EXPECT_DOUBLE_EQ(r.max_vy, b.max_vy);
  EXPECT_DOUBLE_EQ(r.min_w, b.min_w);
  EXPECT_DOUBLE_EQ(r.max_w, b.max_w);
  EXPECT_FALSE(
    c.evaluateVelocityConstraints(
      c.computeOptimalVelocityWithinDynamicWindow(twist(1, 1, -1), r, true), current));
  const auto identity = c.applyRegulationToDynamicWindow(b, c.maximumTranslationalSpeed());
  EXPECT_DOUBLE_EQ(identity.min_vx, b.min_vx);
  EXPECT_DOUBLE_EQ(identity.max_vy, b.max_vy);
  TestController::DynamicWindowBounds3D full {-0.6, 0.6, -0.6, 0.6, -1, 1};
  const auto capped = c.applyRegulationToDynamicWindow(full, 0.2);
  const auto u = c.computeOptimalVelocityWithinDynamicWindow(twist(1, 1, 1), capped, true);
  EXPECT_NEAR(std::hypot(u.linear.x, u.linear.y), 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(u.angular.z, u.linear.x);
}

TEST(Solver, ZeroRayAndFiniteGoalRotation)
{
  TestController c;
  const auto b = c.computeDynamicWindow(twist(0.2, -0.2, 0.0));
  const auto u = c.computeOptimalVelocityWithinDynamicWindow(twist(0, 0, 0), b, true);
  EXPECT_DOUBLE_EQ(u.linear.x, b.min_vx);
  EXPECT_DOUBLE_EQ(u.linear.y, b.max_vy);
  geometry_msgs::msg::PoseStamped zero;
  const auto desired = c.computeDesiredVelocity(zero, 1.0);
  EXPECT_DOUBLE_EQ(desired.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(desired.angular.z, 5.0);
  const auto turn = c.computeGoalVelocity(1.0, 0.1, b);
  EXPECT_GT(turn.angular.z, 0.0);
  EXPECT_FALSE(c.evaluateVelocityConstraints(turn, twist(0.2, -0.2, 0.0)));
  EXPECT_THROW(c.computeDynamicWindow(twist(2, 0, 0)), std::exception);
}

TEST(Solver, AntipodalDesiredYawUsesNegativePi)
{
  TestController c;
  geometry_msgs::msg::PoseStamped carrot;
  for (const double yaw : {M_PI, -M_PI}) {
    const auto desired = c.computeDesiredVelocity(carrot, yaw);
    EXPECT_DOUBLE_EQ(desired.angular.z, -M_PI / 0.2);
    EXPECT_DOUBLE_EQ(desired.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(desired.linear.y, 0.0);
  }
  carrot.pose.position.x = 0.3;
  const auto positive = c.computeDesiredVelocity(carrot, M_PI);
  const auto negative = c.computeDesiredVelocity(carrot, -M_PI);
  EXPECT_DOUBLE_EQ(positive.angular.z, negative.angular.z);
  EXPECT_LT(positive.angular.z, 0.0);
  EXPECT_GT(positive.linear.x, 0.0);
  // The convention changes the exact tie, not either neighboring direction.
  EXPECT_GT(c.computeDesiredVelocity(carrot, M_PI - 1e-8).angular.z, 0.0);
  EXPECT_LT(c.computeDesiredVelocity(carrot, -M_PI + 1e-8).angular.z, 0.0);
}

TEST(Solver, HolonomicProjection)
{
  geometry_msgs::msg::Pose2D start;
  const auto lateral = TestController::projectMotion(start, twist(0, 0.5, 0), 1, 2, 0.05, 0.2);
  EXPECT_NEAR(lateral.back().x, 0.0, 1e-12);
  EXPECT_NEAR(lateral.back().y, 0.5, 1e-12);
  const auto reverse = TestController::projectMotion(start, twist(-0.5, 0, 0), 1, 2, 0.05, 0.2);
  EXPECT_NEAR(reverse.back().x, -0.5, 1e-12);
  const auto circular = TestController::projectMotion(start, twist(1, 1, M_PI_2), 1, 3, 0.05, 0.2);
  EXPECT_NEAR(circular.back().x, 0.0, 1e-12);
  EXPECT_NEAR(circular.back().y, 2.0 / M_PI_2, 1e-12);
  const auto rotation = TestController::projectMotion(start, twist(0, 0, 1), 1, 0, 0.05, 0.3);
  EXPECT_NEAR(rotation.back().theta, 1.0, 1e-12);
  EXPECT_GT(rotation.size(), 2u);
  EXPECT_EQ(TestController::projectMotion(start, twist(0, 0, 0), 1, 0, 0.05, 0.2).size(), 1u);
}

TEST_F(ControllerTest, CostApproachAndInheritedSpeedLimits)
{
  ctrl.nominal(0.6);
  EXPECT_DOUBLE_EQ(ctrl.computeRegulatedSpeed(path(2), twist(0, 0, 0), 0), 0.6);
  ctrl.cost(true);
  const double radius = map->getLayeredCostmap()->getInscribedRadius();
  const double cost = 252.0 * std::exp(-3.0 * (0.4 - radius));
  const double regulated = ctrl.computeRegulatedSpeed(path(2), twist(0, 0, 0), cost);
  EXPECT_NEAR(regulated, 0.4, 1e-8);  // 0.6 nominal * 0.4 m / 0.6 m
  ctrl.approach(0.6);
  EXPECT_NEAR(ctrl.computeRegulatedSpeed(path(0.3), twist(0, 0, 0), cost), 0.2, 1e-8);
  ctrl.cost(false);
  ctrl.approach(0.0);
  ctrl.setSpeedLimit(0.2, false);
  EXPECT_DOUBLE_EQ(ctrl.computeRegulatedSpeed(path(2), twist(0, 0, 0), 0), 0.2);
  ctrl.setSpeedLimit(50, true);
  EXPECT_DOUBLE_EQ(ctrl.computeRegulatedSpeed(path(2), twist(0, 0, 0), 0), 0.3);
  ctrl.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_DOUBLE_EQ(ctrl.computeRegulatedSpeed(path(2), twist(0, 0, 0), 0), 0.6);
  EXPECT_THROW(ctrl.setSpeedLimit(-1, false), std::invalid_argument);
}

TEST_F(ControllerTest, NewPlanReseedsMeasuredFeedback)
{
  ctrl.seed(twist(0.5, 0.5, 0.5));
  ctrl.setPlan(path(2));
  EXPECT_FALSE(ctrl.commandValid());
  const auto u = ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
  EXPECT_TRUE(ctrl.commandValid());
  EXPECT_FALSE(ctrl.evaluateVelocityConstraints(u, twist(0, 0, 0)));
  ctrl.setPlan(path(2));
  ctrl.clipped(true);
  const auto clipped = ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
  EXPECT_FALSE(ctrl.evaluateVelocityConstraints(clipped, twist(0, 0, 0)));
  ctrl.deactivate();
  EXPECT_FALSE(ctrl.commandValid());
  ctrl.activate();
  EXPECT_FALSE(ctrl.commandValid());
}

TEST_F(ControllerTest, FailedCommandInvalidatesFeedback)
{
  ctrl.setPlan(path(2));
  ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr);
  EXPECT_TRUE(ctrl.commandValid());
  auto invalid_pose = pose;
  invalid_pose.header.frame_id = "unavailable_frame";
  EXPECT_THROW(ctrl.computeVelocityCommands(invalid_pose, twist(0, 0, 0), nullptr), std::exception);
  EXPECT_FALSE(ctrl.commandValid());
  ctrl.feedback("CLOSED_LOOP");
  const auto current = twist(0.3, 0.0, 0.0);
  const auto u = ctrl.computeVelocityCommands(pose, current, nullptr).twist;
  EXPECT_FALSE(ctrl.evaluateVelocityConstraints(u, current));
}

TEST_F(ControllerTest, GoalOnlyRotationCommandsAndFiniteOutput)
{
  ctrl.setPlan(path(0, 1.0));
  const auto u = ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
  EXPECT_GT(u.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(u.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(u.linear.y, 0.0);
  EXPECT_TRUE(std::isfinite(u.angular.z));
}

TEST_F(ControllerTest, AntipodalFinalGoalUsesNegativeTurn)
{
  for (const double yaw : {M_PI, -M_PI}) {
    ctrl.setPlan(path(0, yaw));
    const auto u = ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
    EXPECT_LT(u.angular.z, 0.0);
    EXPECT_DOUBLE_EQ(u.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(u.linear.y, 0.0);
    EXPECT_FALSE(ctrl.evaluateVelocityConstraints(u, twist(0, 0, 0)));
  }
}

TEST_F(ControllerTest, LateralAndReverseFootprintCollision)
{
  unsigned int mx, my;
  ASSERT_TRUE(map->getCostmap()->worldToMap(0.0, 0.4, mx, my));
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(ctrl.isCollisionImminentOmni(pose, twist(0, 0.5, 0), 1.0));
  EXPECT_FALSE(ctrl.isCollisionImminentOmni(pose, twist(0.5, 0, 0), 1.0));
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
  ASSERT_TRUE(map->getCostmap()->worldToMap(-0.4, 0.0, mx, my));
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(ctrl.isCollisionImminentOmni(pose, twist(-0.5, 0, 0), 1.0));
}

TEST_F(ControllerTest, RotatingFootprintCollision)
{
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = 0.4; footprint[0].y = 0.1;
  footprint[1].x = -0.4; footprint[1].y = 0.1;
  footprint[2].x = -0.4; footprint[2].y = -0.1;
  footprint[3].x = 0.4; footprint[3].y = -0.1;
  map->setRobotFootprint(footprint);
  unsigned int mx, my;
  ASSERT_TRUE(map->getCostmap()->worldToMap(0.0, 0.35, mx, my));
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(ctrl.isCollisionImminentOmni(pose, twist(0, 0, 0), 0));
  EXPECT_TRUE(ctrl.isCollisionImminentOmni(pose, twist(0, 0, M_PI_2), 0));
}

TEST_F(ControllerTest, PluginExportLoads)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  EXPECT_NE(
    loader.createSharedInstance(
      "nav2_omnidirectional_dwvp_controller::OmnidirectionalDWVPController"), nullptr);
}
