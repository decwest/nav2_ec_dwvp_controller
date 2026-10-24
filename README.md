# EC-DWVP for Nav2

ROS 2 Jazzy controller plugin for **Error-Compensated Dynamic Window Vector
Pursuit (EC-DWVP)** on omnidirectional robots.

EC-DWVP provides a time constant for adjusting the lateral response independently
of the lookahead distance and selects velocity commands within per-axis velocity
and acceleration limits. The reference path specifies both position and orientation.

The `jazzy` branch adapts the Nav2 interfaces while retaining the EC-DWVP solver
and numerical fixtures from Humble. The plugin identifier is unchanged.

## Installation

With ROS 2 Jazzy and Nav2 installed:

```bash
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/decwest/nav2_ec_dwvp_controller.git
cd ..
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --packages-select nav2_ec_dwvp_controller
source install/setup.bash
```

## Configuration

Merge [config/example_params.yaml](config/example_params.yaml) into your Nav2
configuration and adjust the velocity and acceleration limits to your robot.
The example enables error compensation with a lateral time constant of 0.75 s.

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_ec_dwvp_controller::ECDWVPController"
      lateral_time_constant: 0.75
      use_dynamic_window_vector_pursuit: true
```

Supply the desired robot orientation in each path pose. It may differ from the
path tangent, allowing sideways travel.

| Parameter | Purpose |
|---|---|
| `lateral_time_constant` | Lateral response time in seconds. Positive values enable compensation. |
| `lookahead_dist` | Fixed lookahead distance when velocity-scaled lookahead is disabled. |
| `lookahead_time`, `min_lookahead_dist`, `max_lookahead_dist` | Velocity-scaled lookahead settings. |
| `vp_translation_speed` | Desired translation speed. A nonpositive value selects the speed supported in every translation direction. |
| `min_vel_*`, `max_vel_*`, `max_accel_*` | Body-frame velocity and acceleration limits. |
| `velocity_feedback` | `OPEN_LOOP` uses the previous command, and `CLOSED_LOOP` uses measured velocity. |

The [configuration reference](docs/configuration.md) explains defaults, feedback,
speed regulation, comparison modes, and logging. This branch targets Jazzy; `main` retains the Humble implementation.

## Experiment configurations

The previously released experiment settings are in [config/paper/](config/paper/).
They include comparison controllers, the velocity smoother, and the local costmap.
Adapt the HSR topics and frames to your system. The Experiment 2 DWB configuration
also requires [dwb_path_tracking_critics](https://github.com/decwest/dwb_path_tracking_critics).

The standalone Python simulator is available in
[ec_dwvp](https://github.com/decwest/ec_dwvp).

## Testing

From the built workspace:

```bash
colcon test --packages-select nav2_ec_dwvp_controller --return-code-on-test-failure
colcon test-result --verbose
```

Tests cover command feasibility, Python/C++ numerical agreement, lifecycle behavior,
collision projection, and plugin loading.

See [Jazzy port verification](docs/jazzy_port.md) for interface changes, unchanged
fixture hashes and the scope of local validation.

For an isolated build with the Castella Jazzy image, run
`scripts/verify_jazzy.sh`. Set `DWVP_JAZZY_IMAGE` to use another image with the
Jazzy Nav2 and test dependencies installed.

## License

[Apache License 2.0](LICENSE).
