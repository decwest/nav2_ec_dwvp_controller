// Copyright (c) 2026 Fumiya Ohnishi
// Licensed under the Apache License, Version 2.0.
// Fixtures copied from omnidirectional_dwvp commit b09ba8e3a2a5feced5ba317adf793b0e33e69591.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

#include "nav2_ec_dwvp_controller/ec_dwvp_controller.hpp"

class FixtureController
  : public nav2_ec_dwvp_controller::ECDWVPController
{
public:
  bool run(size_t count)
  {
    double px, py, yaw, lookahead, response_distance;
    if (!(std::cin >> px >> py >> yaw >> lookahead >> response_distance >>
      vp_translation_speed_ >> orientation_time_weight_ >> min_orientation_time_))
    {
      return false;
    }
    lateral_time_constant_ = response_distance > 0.0 ?
      response_distance / vp_translation_speed_ : 0.0;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = px;
    pose.pose.position.y = py;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    nav_msgs::msg::Path path, body_path;
    size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    for (size_t i = 0; i < count; ++i) {
      double x, y, heading;
      if (!(std::cin >> x >> y >> heading)) {return false;}
      geometry_msgs::msg::PoseStamped point;
      point.pose.position.x = x;
      point.pose.position.y = y;
      point.pose.orientation.z = std::sin(heading / 2.0);
      point.pose.orientation.w = std::cos(heading / 2.0);
      path.poses.push_back(point);
      const double dx = x - px;
      const double dy = y - py;
      const double distance2 = dx * dx + dy * dy;
      if (distance2 < best) {
        nearest = i;
        best = distance2;
      }
      point.pose.position.x = c * dx + s * dy;
      point.pose.position.y = -s * dx + c * dy;
      point.pose.orientation.z = std::sin((heading - yaw) / 2.0);
      point.pose.orientation.w = std::cos((heading - yaw) / 2.0);
      body_path.poses.push_back(point);
    }
    // Supply RPP's retained preview suffix, but project on the complete reference.
    body_path.poses.erase(body_path.poses.begin(), body_path.poses.begin() + nearest);
    const auto preview = computePreviewTarget(body_path, lookahead);
    const auto reference = projectReference(path, pose);
    ResponseDiagnostics response;
    const auto desired = computeDesiredVelocity(
      preview.pose, preview.yaw, &reference, preview.goal_region, preview.nearest_target,
      &response);
    std::cout << std::setprecision(17) << desired.linear.x << ' ' << desired.linear.y << ' ' <<
      desired.angular.z << ' ' << preview.pose.pose.position.x << ' ' <<
      preview.pose.pose.position.y << ' ' << preview.yaw << ' ' << response.rule << ' ' <<
      preview.goal_region << ' ' << nearest << '\n';
    return true;
  }
};

int main()
{
  FixtureController controller;
  size_t count;
  while (std::cin >> count) {
    if (count == 0 || !controller.run(count)) {return 1;}
  }
  return 0;
}
