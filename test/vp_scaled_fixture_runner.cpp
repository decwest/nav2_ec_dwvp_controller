// Copyright (c) 2026 Fumiya Ohnishi
// Licensed under the Apache License, Version 2.0.

#include <array>
#include <iomanip>
#include <iostream>

#include "nav2_ec_dwvp_controller/ec_dwvp_controller.hpp"

class FixtureController
  : public nav2_ec_dwvp_controller::ECDWVPController
{
public:
  void run(const std::array<double, 18> & p)
  {
    min_vel_x_ = p[3]; min_vel_y_ = p[4]; min_vel_theta_ = p[5];
    max_vel_x_ = p[6]; max_vel_y_ = p[7]; max_vel_theta_ = p[8];
    max_accel_x_ = p[9]; max_accel_y_ = p[10]; max_accel_theta_ = p[11];
    control_duration_ = p[15];
    geometry_msgs::msg::Twist current, desired;
    desired.linear.x = p[0]; desired.linear.y = p[1]; desired.angular.z = p[2];
    current.linear.x = p[12]; current.linear.y = p[13]; current.angular.z = p[14];
    const auto box = applyRegulationToDynamicWindow(computeDynamicWindow(current), p[16]);
    const auto u = computeScaledVelocity(desired, p[16], box);
    std::cout << std::setprecision(17) << box.min_vx << ' ' << box.min_vy << ' ' << box.min_w <<
      ' ' << box.max_vx << ' ' << box.max_vy << ' ' << box.max_w << ' ' << u.linear.x <<
      ' ' << u.linear.y << ' ' << u.angular.z << '\n';
  }
};

int main()
{
  FixtureController controller;
  std::array<double, 18> p;
  while (std::cin >> p[0]) {
    for (size_t i = 1; i < p.size(); ++i) {
      if (!(std::cin >> p[i])) {return 1;}
    }
    controller.run(p);
  }
  return 0;
}
