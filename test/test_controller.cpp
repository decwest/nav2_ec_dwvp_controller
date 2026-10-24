// Copyright (c) 2026 Fumiya Ohnishi
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "angles/angles.h"
#include "nav2_ec_dwvp_controller/ec_dwvp_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pluginlib/class_loader.hpp"

using nav2_ec_dwvp_controller::ECDWVPController;

class TestController : public ECDWVPController
{
public:
  using ECDWVPController::DynamicWindowBounds3D;
  using ECDWVPController::ResponseDiagnostics;
  using ECDWVPController::applyRegulationToDynamicWindow;
  using ECDWVPController::clipVelocityToBounds;
  using ECDWVPController::computeDesiredVelocity;
  using ECDWVPController::computeScaledVelocity;
  using ECDWVPController::computeDynamicWindow;
  using ECDWVPController::computeGoalVelocity;
  using ECDWVPController::computeOptimalVelocityWithinDynamicWindow;
  using ECDWVPController::computeRegulatedSpeed;
  using ECDWVPController::evaluateVelocityConstraints;
  using ECDWVPController::isCollisionImminentOmni;
  using ECDWVPController::maximumTranslationalSpeed;
  using ECDWVPController::projectMotion;
  using ECDWVPController::projectReference;
  using ECDWVPController::computePreviewTarget;

  TestController()
  {
    control_duration_ = 0.05;
    // Solver-only tests do not configure a ROS lifecycle node.
    params_ = &test_parameters_;
  }
  nav2_regulated_pure_pursuit_controller::Parameters test_parameters_ {};
  void cost(bool enable) {params_->use_cost_regulated_linear_velocity_scaling = enable;}
  void approach(double distance) {params_->approach_velocity_scaling_dist = distance;}
  void nominal(double value)
  {
    params_->desired_linear_vel = params_->base_desired_linear_vel = value;
  }
  void translation(double value) {vp_translation_speed_ = value;}
  void lateralTime(double value) {lateral_time_constant_ = value;}
  double lateralTime() const {return lateral_time_constant_;}
  void previewDistance(double value)
  {
    params_->lookahead_dist = value;
    params_->use_velocity_scaled_lookahead_dist = false;
  }
  auto retainedReference(const geometry_msgs::msg::PoseStamped & pose) const
  {
    return projectReference(reference_plan_, pose);
  }
  size_t prunedSize() const {return path_handler_->getPlan().poses.size();}
  void limits(double xmin, double xmax, double ymin, double ymax)
  {
    min_vel_x_ = xmin; max_vel_x_ = xmax; min_vel_y_ = ymin; max_vel_y_ = ymax;
  }
  void shortGoal() {goal_dist_tol_ = 0.01; max_accel_x_ = max_accel_y_ = 0.22;}
  void clipped(bool value) {use_dynamic_window_vector_pursuit_ = !value;}
  void scaled(bool value) {use_uniform_velocity_scaling_ = value;}
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
  // Jazzy RPP reconstructs distance with the inscribed-obstacle cost (253).
  const double cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE *
    std::exp(-3.0 * (0.4 - radius));
  const double regulated = ctrl.computeRegulatedSpeed(path(2), twist(0, 0, 0), cost);
  // Jazzy uses log(253.0f), so its inverse cost conversion has float precision.
  EXPECT_NEAR(regulated, 0.4, 1e-7);  // 0.6 nominal * 0.4 m / 0.6 m
  ctrl.approach(0.6);
  EXPECT_NEAR(ctrl.computeRegulatedSpeed(path(0.3), twist(0, 0, 0), cost), 0.2, 1e-7);
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
      "nav2_ec_dwvp_controller::ECDWVPController"), nullptr);
}

TEST(Solver, DesiredTranslationSpeedAndOrientationTime)
{
  TestController c;
  geometry_msgs::msg::PoseStamped target;
  target.pose.position.x = 0.3;
  target.pose.position.y = 0.4;
  c.limits(-0.4, 0.6, -0.3, 0.5);
  for (double setting : {0.0, -1.0, 0.2}) {
    c.translation(setting);
    const double speed = setting > 0.0 ? setting : 0.3;
    const auto u = c.computeDesiredVelocity(target, 0.5);
    EXPECT_NEAR(std::hypot(u.linear.x, u.linear.y), speed, 1e-12);
    EXPECT_NEAR(u.angular.z, speed, 1e-12);
  }
}

TEST(Solver, TranslationMagnitudePreservesRayExceptAtOrientationTimeFloor)
{
  TestController c;
  geometry_msgs::msg::PoseStamped target;
  target.pose.position.x = 0.5;
  const auto b = c.computeDynamicWindow(twist(0.2, 0, 0.1));
  c.translation(0.2);
  const auto a = c.computeDesiredVelocity(target, 0.25);
  c.translation(0.6);
  const auto d = c.computeDesiredVelocity(target, 0.25);
  const auto ua = c.computeOptimalVelocityWithinDynamicWindow(a, b, true);
  const auto ud = c.computeOptimalVelocityWithinDynamicWindow(d, b, true);
  EXPECT_NEAR(ua.linear.x, ud.linear.x, 1e-12);
  EXPECT_NEAR(ua.angular.z, ud.angular.z, 1e-12);
  EXPECT_GT(ua.linear.x / a.linear.x, 1.0);  // alpha is not capped at one
  target.pose.position.x = 0.01;
  c.translation(0.2);
  const auto near_a = c.computeDesiredVelocity(target, 0.25);
  c.translation(0.6);
  const auto near_d = c.computeDesiredVelocity(target, 0.25);
  EXPECT_DOUBLE_EQ(near_a.angular.z, near_d.angular.z);
  EXPECT_NE(near_a.linear.x, near_d.linear.x);  // floor changes the ray direction
}

TEST_F(ControllerTest, FastestPointInsideStoppingDistance)
{
  ctrl.limits(-0.22, 0.22, -0.22, 0.22);
  ctrl.shortGoal();
  ctrl.setPlan(path(0.05));
  const auto current = twist(0.22, 0.0, 0.0);
  const auto u = ctrl.computeVelocityCommands(pose, current, nullptr).twist;
  EXPECT_NEAR(u.linear.x, 0.22, 1e-12);
  EXPECT_FALSE(ctrl.evaluateVelocityConstraints(u, current));
  ctrl.approach(0.6);
  ctrl.setPlan(path(0.05));
  const auto regulated = ctrl.computeVelocityCommands(pose, current, nullptr).twist;
  EXPECT_LT(regulated.linear.x, u.linear.x);
  EXPECT_FALSE(ctrl.evaluateVelocityConstraints(regulated, current));
}

TEST(Solver, NonintersectionTiePrefersLargerAlpha)
{
  TestController c;
  TestController::DynamicWindowBounds3D box {0.1, 0.3, 0.2, 0.4, 0.0, 0.0};
  const auto u = c.computeOptimalVelocityWithinDynamicWindow(twist(1, 0, 0), box, true);
  EXPECT_NEAR(u.linear.x, 0.3, 1e-12);
  EXPECT_NEAR(u.linear.y, 0.2, 1e-12);
}

TEST(Solver, ScaledVpPreservesFeasibleDesiredAndUniformlyReducesAngularSaturation)
{
  TestController ctrl;
  const TestController::DynamicWindowBounds3D bounds {-0.6, 0.6, -0.6, 0.6, -1.0, 1.0};
  const auto desired = twist(0.2, -0.1, 0.3);
  const auto clip = ctrl.clipVelocityToBounds(desired, bounds);
  const auto scaled = ctrl.computeScaledVelocity(desired, ctrl.maximumTranslationalSpeed(), bounds);
  EXPECT_DOUBLE_EQ(clip.linear.x, scaled.linear.x);
  EXPECT_DOUBLE_EQ(clip.linear.y, scaled.linear.y);
  EXPECT_DOUBLE_EQ(clip.angular.z, scaled.angular.z);
  const auto limited = ctrl.computeScaledVelocity(
    twist(0.2, -0.1, 2.0), ctrl.maximumTranslationalSpeed(), bounds);
  EXPECT_DOUBLE_EQ(limited.linear.x, 0.1);
  EXPECT_DOUBLE_EQ(limited.linear.y, -0.05);
  EXPECT_DOUBLE_EQ(limited.angular.z, 1.0);
}

TEST(Solver, ScaledVpAlwaysUsesReachableWindowIncludingSuddenRegulation)
{
  TestController ctrl;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for (int i = 0; i < 1000; ++i) {
    const auto current = twist(0.6 * unit(rng), 0.6 * unit(rng), unit(rng));
    const double cap = 0.4 * (1.0 + unit(rng));
    const auto box = ctrl.applyRegulationToDynamicWindow(ctrl.computeDynamicWindow(current), cap);
    const auto command = ctrl.computeScaledVelocity(
      twist(unit(rng), unit(rng), 3.0 * unit(rng)), cap, box);
    EXPECT_FALSE(ctrl.evaluateVelocityConstraints(command, current));
    EXPECT_GE(command.linear.x, box.min_vx);
    EXPECT_LE(command.linear.x, box.max_vx);
    EXPECT_GE(command.linear.y, box.min_vy);
    EXPECT_LE(command.linear.y, box.max_vy);
    EXPECT_GE(command.angular.z, box.min_w);
    EXPECT_LE(command.angular.z, box.max_w);
  }
}

TEST_F(ControllerTest, ScaledVpSharesClippedTerminalHandling)
{
  ctrl.clipped(true);
  ctrl.setPlan(path(0.001, 0.9));
  const auto clip = ctrl.computeVelocityCommands(pose, twist(0.1, -0.1, 0.2), nullptr).twist;
  ctrl.scaled(true);
  ctrl.setPlan(path(0.001, 0.9));
  const auto scaled = ctrl.computeVelocityCommands(pose, twist(0.1, -0.1, 0.2), nullptr).twist;
  EXPECT_DOUBLE_EQ(clip.linear.x, scaled.linear.x);
  EXPECT_DOUBLE_EQ(clip.linear.y, scaled.linear.y);
  EXPECT_DOUBLE_EQ(clip.angular.z, scaled.angular.z);
}

TEST(Solver, NonpositiveLateralTimePreservesLegacyDemandBits)
{
  TestController ctrl;
  ctrl.limits(-0.4, 0.6, -0.3, 0.5);
  for (double tau : {0.0, -0.5, -10.0}) {
    ctrl.lateralTime(tau);
    for (double setting : {0.0, -1.0, 0.22}) {
      ctrl.translation(setting);
      const double speed = setting > 0.0 ? setting : 0.3;
      for (const auto & xy : std::vector<std::array<double, 2>>{
        {0.3, 0.4}, {-0.2, 0.07}, {0.0, 0.0}, {1e-6, 0.0}, {0.0, -2e-6}})
      {
        geometry_msgs::msg::PoseStamped target;
        target.pose.position.x = xy[0];
        target.pose.position.y = xy[1];
        for (double yaw : {-M_PI, 0.0, 0.73, M_PI}) {
          geometry_msgs::msg::Twist legacy;
          const double distance = std::hypot(xy[0], xy[1]);
          if (distance > 1e-6) {
            legacy.linear.x = speed * xy[0] / distance;
            legacy.linear.y = speed * xy[1] / distance;
          }
          double wrapped = angles::normalize_angle(yaw);
          if (wrapped == M_PI) {wrapped = -M_PI;}
          legacy.angular.z = wrapped / std::max(0.2, distance / speed);
          TestController::ResponseDiagnostics response;
          const auto actual = ctrl.computeDesiredVelocity(
            target, yaw, nullptr, true, true,
            &response);
          EXPECT_EQ(std::memcmp(&actual.linear.x, &legacy.linear.x, sizeof(double)), 0);
          EXPECT_EQ(std::memcmp(&actual.linear.y, &legacy.linear.y, sizeof(double)), 0);
          EXPECT_EQ(std::memcmp(&actual.angular.z, &legacy.angular.z, sizeof(double)), 0);
          EXPECT_EQ(response.rule, "inactive");
        }
      }
    }
  }
}

TEST(Solver, LateralTimeGivesStraightPathIdentityAndUnchangedAngularDemand)
{
  TestController ctrl;
  ctrl.limits(-0.4, 0.6, -0.3, 0.5);
  for (double speed_setting : {0.0, 0.22}) {
    ctrl.translation(speed_setting);
    const double speed = speed_setting > 0.0 ? speed_setting : 0.3;
    for (double error : {-0.5, -0.01, 0.01, 0.5}) {
      auto reference_path = path(2.0);
      for (auto & point : reference_path.poses) {
        point.pose.position.x -= 0.83;
        point.pose.position.y -= error;
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.orientation.w = 1.0;
      const auto reference = ctrl.projectReference(reference_path, pose);
      ctrl.previewDistance(0.11);
      auto suffix = reference_path;
      suffix.poses.erase(suffix.poses.begin(), suffix.poses.begin() + 8);
      const auto preview = ctrl.computePreviewTarget(suffix, 0.11);
      ctrl.lateralTime(0.0);
      const auto original = ctrl.computeDesiredVelocity(preview.pose, 0.7);
      for (double tau : {0.75, 1.5}) {
        ctrl.lateralTime(tau);
        TestController::ResponseDiagnostics response;
        const auto actual = ctrl.computeDesiredVelocity(
          preview.pose, 0.7, &reference, preview.goal_region, preview.nearest_target, &response);
        const double ratio = -error / (tau * speed);
        EXPECT_NEAR(actual.linear.x, speed / std::hypot(1.0, ratio), 1e-12);
        EXPECT_NEAR(actual.linear.y, speed * ratio / std::hypot(1.0, ratio), 1e-12);
        EXPECT_EQ(actual.angular.z, original.angular.z);
        EXPECT_NEAR(response.lateral_error, error, 1e-12);
        EXPECT_NEAR(response.response_distance, tau * speed, 1e-12);
        EXPECT_EQ(response.rule, std::abs(error) > 0.11 ? "tangent" : "chord");
      }
    }
  }
}

TEST(Solver, ProjectionKeepsEarliestSegmentsAndInheritsDuplicateTangents)
{
  auto reference_path = path(0.0);
  reference_path.poses.resize(4);
  reference_path.poses[2].pose.position.y = 1.0;
  reference_path.poses[3].pose.position.x = 1.0;
  reference_path.poses[3].pose.position.y = 1.0;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  auto reference = TestController::projectReference(reference_path, pose);
  EXPECT_EQ(reference.segment, 0u);  // Duplicate ties with the next segment.
  EXPECT_EQ(reference.fraction, 0.0);
  EXPECT_EQ(reference.tangent_x, 0.0);
  EXPECT_EQ(reference.tangent_y, 1.0);
  pose.pose.position.y = 0.4;
  reference = TestController::projectReference(reference_path, pose);
  EXPECT_EQ(reference.segment, 1u);
  EXPECT_NEAR(reference.fraction, 0.4, 1e-12);
  pose.pose.position.y = 1.0;
  reference = TestController::projectReference(reference_path, pose);
  EXPECT_EQ(reference.segment, 1u);  // Corner ties use the incoming tangent.
  EXPECT_EQ(reference.tangent_y, 1.0);
  pose.pose.orientation.z = std::sin(0.35);
  pose.pose.orientation.w = std::cos(0.35);
  reference = TestController::projectReference(path(0.0), pose);
  EXPECT_NEAR(reference.tangent_x, std::cos(0.7), 1e-12);
  EXPECT_NEAR(reference.tangent_y, -std::sin(0.7), 1e-12);
  EXPECT_NEAR(reference.lateral_error, 1.0, 1e-12);
  reference_path.poses.resize(1);
  EXPECT_NO_THROW(TestController::projectReference(reference_path, pose));
  reference_path.poses.clear();
  EXPECT_THROW(TestController::projectReference(reference_path, pose), std::exception);
}

TEST(Solver, GoalResponseClampsDistanceAndPreservesZeroDistanceThreshold)
{
  TestController ctrl;
  ctrl.translation(0.22);
  ctrl.lateralTime(1.5);
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  auto reference_path = path(0.08);
  for (auto & point : reference_path.poses) {
    point.pose.position.x -= 0.04;
    point.pose.position.y = -0.03;
  }
  auto reference = ctrl.projectReference(reference_path, pose);
  auto target = reference_path.poses.back();
  TestController::ResponseDiagnostics response;
  auto actual = ctrl.computeDesiredVelocity(target, 0.4, &reference, true, false, &response);
  EXPECT_EQ(response.rule, "goal");
  EXPECT_NEAR(response.chord_length, 0.04, 1e-12);
  EXPECT_NEAR(actual.linear.x, 0.22 * 0.04 / 0.05, 1e-12);
  EXPECT_NEAR(actual.linear.y, -0.22 * 0.03 / 0.05, 1e-12);
  // Projection at the end: vanishing chord must aim back at the goal.
  pose.pose.position.x = 0.1;
  reference = ctrl.projectReference(reference_path, pose);
  target.pose.position.x -= 0.1;
  actual = ctrl.computeDesiredVelocity(target, 0.4, &reference, true, false, &response);
  EXPECT_EQ(response.rule, "goal");
  EXPECT_NEAR(actual.linear.x, -0.22 * 0.06 / std::hypot(0.06, 0.03), 1e-12);
  EXPECT_NEAR(actual.linear.y, -0.22 * 0.03 / std::hypot(0.06, 0.03), 1e-12);
  target.pose.position.x = 0.0;
  target.pose.position.y = 1e-6;
  actual = ctrl.computeDesiredVelocity(target, 0.4, &reference, true, false, &response);
  EXPECT_EQ(actual.linear.x, 0.0);
  EXPECT_EQ(actual.linear.y, 0.0);
  EXPECT_NEAR(actual.angular.z, 2.0, 1e-12);
  // A zero inscribed demand cannot define a positive response distance.
  ctrl.translation(0.0);
  ctrl.limits(0.0, 0.0, -0.2, 0.2);
  actual = ctrl.computeDesiredVelocity(target, 0.4, &reference, false, false, &response);
  EXPECT_EQ(actual.linear.x, 0.0);
  EXPECT_EQ(actual.linear.y, 0.0);
  EXPECT_EQ(response.rule, "inactive");
}

TEST_F(ControllerTest, CompensationRetainsIncomingSegmentAcrossPruning)
{
  ctrl.lateralTime(0.75);
  ctrl.translation(0.22);
  ctrl.previewDistance(0.3);
  auto reference_path = path(2.0);
  reference_path.poses = {
    reference_path.poses.front(), reference_path.poses[10], reference_path.poses.back()};
  for (auto & point : reference_path.poses) {
    point.pose.position.x -= 0.8;
    point.pose.position.y = -0.1;
  }
  ctrl.setPlan(reference_path);
  for (int cycle = 0; cycle < 3; ++cycle) {
    const auto cmd = ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
    EXPECT_EQ(ctrl.prunedSize(), 2u);
    const auto reference = ctrl.retainedReference(pose);
    EXPECT_EQ(reference.segment, 0u);
    EXPECT_NEAR(reference.fraction, 0.8, 1e-12);
    EXPECT_NEAR(reference.x, 0.0, 1e-12);
    EXPECT_NEAR(reference.y, -0.1, 1e-12);
    EXPECT_LT(cmd.linear.y, 0.0);
  }
}

TEST_F(ControllerTest, DisabledCommandsAndCompensatedTerminalCommandsAreIdentical)
{
  for (int mode = 0; mode < 3; ++mode) {
    ctrl.clipped(mode != 0);
    ctrl.scaled(mode == 2);
    ctrl.lateralTime(0.0);
    ctrl.setPlan(path(2.0, 0.7));
    const auto baseline = ctrl.computeVelocityCommands(pose, twist(0.1, -0.1, 0.2), nullptr).twist;
    ctrl.lateralTime(-0.75);
    ctrl.setPlan(path(2.0, 0.7));
    const auto disabled = ctrl.computeVelocityCommands(pose, twist(0.1, -0.1, 0.2), nullptr).twist;
    EXPECT_EQ(baseline, disabled);
    ctrl.lateralTime(0.0);
    ctrl.setPlan(path(0.001, 0.9));
    const auto terminal = ctrl.computeVelocityCommands(pose, twist(0.1, -0.1, 0.2), nullptr).twist;
    ctrl.lateralTime(0.75);
    ctrl.setPlan(path(0.001, 0.9));
    const auto compensated =
      ctrl.computeVelocityCommands(pose, twist(0.1, -0.1, 0.2), nullptr).twist;
    EXPECT_EQ(terminal, compensated);
  }
}

TEST_F(ControllerTest, LateralTimeIsReadAtConfigureAndRejectsNonfiniteValues)
{
  EXPECT_EQ(ctrl.lateralTime(), 0.0);
  node->set_parameter(rclcpp::Parameter("FollowPath.lateral_time_constant", 0.75));
  TestController configured;
  configured.configure(node, "FollowPath", tf, map);
  EXPECT_EQ(configured.lateralTime(), 0.75);
  configured.cleanup();
  node->set_parameter(
    rclcpp::Parameter(
      "FollowPath.lateral_time_constant", std::numeric_limits<double>::infinity()));
  EXPECT_THROW(configured.configure(node, "FollowPath", tf, map), std::invalid_argument);
  configured.cleanup();
}

TEST_F(ControllerTest, CompensatedCommandUsesRobotPoseInReferenceFrame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "reference";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 0.8;
  transform.transform.translation.y = 0.1;
  transform.transform.rotation.z = std::sin(0.35);
  transform.transform.rotation.w = std::cos(0.35);
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  auto reference_path = path(2.0, 0.7);
  reference_path.header.frame_id = "reference";
  for (auto & point : reference_path.poses) {
    point.header = reference_path.header;
  }
  ctrl.lateralTime(0.75);
  ctrl.translation(0.22);
  ctrl.clipped(true);
  ctrl.previewDistance(0.3);
  ctrl.setPlan(reference_path);
  const double ratio = -0.1 / (0.75 * 0.22);
  const double vx = 0.22 / std::hypot(1.0, ratio);
  const double vy = vx * ratio;
  const auto expected = twist(
    std::cos(0.7) * vx + std::sin(0.7) * vy,
    -std::sin(0.7) * vx + std::cos(0.7) * vy, 0.0);
  const auto cmd = ctrl.computeVelocityCommands(pose, expected, nullptr).twist;
  EXPECT_NEAR(cmd.linear.x, expected.linear.x, 1e-12);
  EXPECT_NEAR(cmd.linear.y, expected.linear.y, 1e-12);
  EXPECT_NEAR(cmd.angular.z, expected.angular.z, 1e-12);
}

TEST_F(ControllerTest, ResetAndCancelReseedMeasuredFeedback)
{
  ctrl.setPlan(path(2));
  ctrl.seed(twist(0.5, 0.5, 0.5));
  ctrl.reset();
  EXPECT_FALSE(ctrl.commandValid());
  auto command = ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
  EXPECT_FALSE(ctrl.evaluateVelocityConstraints(command, twist(0, 0, 0)));
  ctrl.seed(twist(0.5, 0.5, 0.5));
  EXPECT_TRUE(ctrl.cancel());
  EXPECT_FALSE(ctrl.commandValid());
  command = ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
  EXPECT_FALSE(ctrl.evaluateVelocityConstraints(command, twist(0, 0, 0)));
  EXPECT_TRUE(ctrl.commandValid());
}

TEST_F(ControllerTest, LateralAndReversePathsPreserveRequestedHeading)
{
  auto lateral = path(2);
  for (auto & point : lateral.poses) {
    std::swap(point.pose.position.x, point.pose.position.y);
  }
  ctrl.setPlan(lateral);
  auto command = ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
  EXPECT_GT(command.linear.y, 0.0);
  EXPECT_NEAR(command.linear.x, 0.0, 1e-12);
  EXPECT_NEAR(command.angular.z, 0.0, 1e-12);
  ctrl.setPlan(path(-2));
  command = ctrl.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
  EXPECT_LT(command.linear.x, 0.0);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-12);
  EXPECT_NEAR(command.angular.z, 0.0, 1e-12);
}

TEST_F(ControllerTest, NestedPrimaryControllerReadsItsOwnParameters)
{
  const std::string name = "FollowPath.primary_controller";
  node->declare_parameter(name + ".lateral_time_constant", 0.75);
  node->declare_parameter(name + ".max_accel_x", 0.1);
  node->declare_parameter(name + ".use_collision_detection", false);
  node->declare_parameter(name + ".use_regulated_linear_velocity_scaling", false);
  TestController primary;
  primary.configure(node, name, tf, map);
  EXPECT_DOUBLE_EQ(primary.lateralTime(), 0.75);
  primary.activate();
  primary.setPlan(path(2));
  const auto command = primary.computeVelocityCommands(pose, twist(0, 0, 0), nullptr).twist;
  EXPECT_NEAR(command.linear.x, 0.1 / 20.0, 1e-12);
  primary.deactivate();
  primary.cleanup();
}
