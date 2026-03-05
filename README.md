# nav2_dynamic_window_vector_pursuit_controller

**Nav2 controller plugin for Dynamic Window Vector Pursuit (DWVP) on omnidirectional robots**

Dynamic Window Vector Pursuit (DWVP) extends Nav2's Regulated Pure Pursuit (RPP) to omnidirectional motion (`vx`, `vy`, `w`) while considering per-axis velocity and acceleration constraints.

The controller computes a desired velocity vector toward a lookahead pose and selects the point in the dynamic window (the feasible set defined by the current velocity and acceleration limits) that is closest to the ray formed by scalar multiples of the desired velocity vector by solving a ray-box intersection and projection problem.

## Installation

1. Clone this repository into your ROS 2 workspace `src/` directory:

```bash
git clone https://github.com/decwest/nav2_dynamic_window_vector_pursuit_controller.git
```

2. Build the package:

```bash
colcon build --symlink-install
```

## Plugin Name

```yaml
plugin: "nav2_dynamic_window_vector_pursuit_controller::DynamicWindowVectorPursuitController"
```

## Example Configuration

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001

    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    FollowPath:
      plugin: "nav2_dynamic_window_vector_pursuit_controller::DynamicWindowVectorPursuitController"

      # DWVP-specific
      max_vel_x: 0.6
      min_vel_x: -0.6
      max_vel_y: 0.6
      min_vel_y: -0.6
      max_vel_theta: 1.0
      min_vel_theta: -1.0
      max_accel_x: 1.0
      max_accel_y: 1.0
      max_accel_theta: 2.0
      orientation_time_weight: 1.0
      min_orientation_time: 0.2
      velocity_feedback: "OPEN_LOOP"      # OPEN_LOOP or CLOSED_LOOP
      use_dynamic_window_vector_pursuit: true
      use_collision_detection_for_omni: false

      # CSV logging (optional)
      enable_csv_logging: false
      csv_log_directory: ""               # empty -> default package_share/data (fallback: /tmp)
      csv_filename_prefix: "dwvp_log"
      csv_pose_frame: "map"

      # other parameters (examples)
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      use_velocity_scaled_lookahead_dist: true
      max_robot_pose_search_dist: 10.0
      transform_tolerance: 0.1
      use_collision_detection: true
      use_interpolation: true

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    feedback: "OPEN_LOOP"
    max_velocity: [0.6, 0.6, 1.0]
    min_velocity: [-0.6, -0.6, -1.0]
    max_accel: [1.0, 1.0, 2.0]
    max_decel: [-1.0, -1.0, -2.0]
```

## DWVP-Specific Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `max_vel_x` | double | `0.6` | Maximum `vx` (m/s). |
| `min_vel_x` | double | `-0.6` | Minimum `vx` (m/s). |
| `max_vel_y` | double | `0.6` | Maximum `vy` (m/s). |
| `min_vel_y` | double | `-0.6` | Minimum `vy` (m/s). |
| `max_vel_theta` | double | `1.0` | Maximum yaw rate `w` (rad/s). |
| `min_vel_theta` | double | `-1.0` | Minimum yaw rate `w` (rad/s). |
| `max_accel_x` | double | `1.0` | Max acceleration limit for `vx` (m/s^2). |
| `max_accel_y` | double | `1.0` | Max acceleration limit for `vy` (m/s^2). |
| `max_accel_theta` | double | `2.0` | Max acceleration limit for `w` (rad/s^2). |
| `orientation_time_weight` | double | `1.0` | Relative importance of translation and rotation (value greater than 0) |
| `velocity_feedback` | string | `"OPEN_LOOP"` | `OPEN_LOOP`: use previous command. `CLOSED_LOOP`: use measured odometry velocity. |
| `use_dynamic_window_vector_pursuit` | bool | `true` | `true`: apply DWVP projection in dynamic window. `false`: output desired vector-pursuit velocity directly. |
| `use_collision_detection_for_omni` | bool | `false` | Enables approximate omni collision checking when `use_collision_detection` is also enabled. |
| `enable_csv_logging` | bool | `false` | Enable CSV logging for each command cycle. |
| `csv_log_directory` | string | package share `/data` (fallback `/tmp`) | Directory for CSV files. |
| `csv_filename_prefix` | string | `"dwvp_log"` | Prefix for generated CSV file names. |
| `csv_pose_frame` | string | `"map"` | Frame used for additional logged pose columns (`map_x,map_y,map_yaw`). |

### Parameter Notes

- `max_accel_x`, `max_accel_y`, and `max_accel_theta` are treated as absolute values internally. (Handling different deceleration rates is TODO)
- `velocity_feedback` is case-insensitive (`open_loop`, `OPEN_LOOP`, etc.).
- When `use_dynamic_window_vector_pursuit=false`, command feasibility is no longer guaranteed by DWVP projection.

## CSV Logging

When `enable_csv_logging=true`:

- A new CSV file is opened when a new plan is set (`setPlan`).
- File name format: `<csv_filename_prefix>_YYYYMMDD_HHMMSS_NNNNNNNNN.csv`
- Logging directory:
  - `csv_log_directory` if valid
  - otherwise package share `data/`
  - fallback to `/tmp` if directory creation fails

CSV header columns:

```text
sec,nsec,x,y,yaw,map_x,map_y,map_yaw,map_pose_valid,
v_real,w_real,v_now,w_now,v_cmd,w_cmd,v_nav,w_nav,
vx_real,vy_real,vx_now,vy_now,vx_cmd,vy_cmd,vx_nav,vy_nav,
velocity_violation,curvature,
dw_vx_max,dw_vx_min,dw_vy_max,dw_vy_min,dw_w_max,dw_w_min,
v_reg,lookahead_dist,lookahead_yaw,is_accel
```

## RPP Parameters

DWVP inherits from `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
Most RPP parameters can be used as-is.

Reference:
- https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html

## Collision Detection Note for Omni Motion

RPP collision checking is primarily designed for forward-arc motion (`vx`, `w`).
For omnidirectional motion (`vx`,`vy`,`w`), this plugin keeps `use_collision_detection_for_omni=false` by default.

To enable approximate omni collision checking, enable both:

- `use_collision_detection: true` (RPP side)
- `use_collision_detection_for_omni: true` (DWVP side)

## License

Apache License 2.0
