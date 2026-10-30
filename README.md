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

Optional raw cost regulation slows from the combined local costmap, including
costs supplied by semantic layers. Set
`use_raw_cost_regulated_linear_velocity_scaling: true` to enable it. See
[raw cost settings and behavior](docs/configuration.md#raw-cost-regulation) for
the sampling distance, speed mapping, and interaction with goal approach and
closed-loop collision prediction. It is disabled by default.

With `OPEN_LOOP`, path updates preserve the previous successful command, including
changed endpoints, one-point stop paths and new goals during continuous control.
Activation, reset/cancel, failed commands, invalid empty paths, or a command gap longer than
`max(0.1 s, 3 / controller_frequency)` reseeds from measured velocity. Measured
feedback outside the acceleration-feasible velocity limits still raises an error;
replanning does not relax the velocity or acceleration constraints.

Closed-loop collision prediction can be enabled with
`collision_prediction_mode: closed_loop`. It repeatedly runs EC-DWVP using virtual
poses and velocities, including moving preview targets and acceleration limits.
It maintains the candidate command while enough stopping distance remains, then
limits speed and verifies the resulting trajectory. The default horizon is maximum
stopping time plus 2 s, with 0.3 m additional stopping travel. See the
[prediction model and diagnostics](docs/configuration.md#closed-loop-prediction-and-braking)
for assumptions and parameters. Omitting the mode preserves constant-command checking.

When collision prediction reduces or rejects a command, the plugin also publishes
`<controller namespace>/collision_costmap` (`nav2_msgs/Costmap`) and
`<controller namespace>/collision_command` (`geometry_msgs/TwistStamped`) if subscribed.
Dots in the plugin namespace become slashes, e.g. `FollowPath.primary_controller` becomes
`/FollowPath/primary_controller/collision_costmap`. The costmap is copied under the control
cycle's costmap lock, so it contains the exact cells checked by the controller.
The command is the original body-frame candidate, not an actuator command.
Both share the control pose stamp with `/lookahead_collision_arc`. Costmap
`metadata.update_time` is unset because the layer update timestamp is unavailable.
These intervention snapshots allow analysis of transient obstacles that periodic
costmap publications can miss. Castella's `bagrec nav` profile includes them via
its `/FollowPath/.*` subscription.

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
collision projection, plugin loading, raw cost reduction, and agreement between
prediction and successive control commands across spatial cost bands.

See [Jazzy port verification](docs/jazzy_port.md) for interface changes, unchanged
fixture hashes and the scope of local validation.

For an isolated build with the Castella Jazzy image, run
`scripts/verify_jazzy.sh`. Set `DWVP_JAZZY_IMAGE` to use another image with the
Jazzy Nav2 and test dependencies installed.

## License

[Apache License 2.0](LICENSE).
