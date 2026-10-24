# Configuration reference

Set these parameters under the controller ID, such as `FollowPath`.
The complete [example](../config/example_params.yaml) enables compensation.
The values below are the plugin defaults when a setting is omitted.

## Controller parameters

| Parameter | Default | Meaning |
|---|---|---|
| `lateral_time_constant` | `0.0` | Lateral response time [s]. Positive values enable compensation, and nonpositive values retain the original VP demand. |
| `vp_translation_speed` | `0.0` | Translation demand [m/s]. A nonpositive value uses `min(max_vel_x, -min_vel_x, max_vel_y, -min_vel_y)`. |
| `max_vel_x`, `min_vel_x` | `0.6`, `-0.6` | Body x velocity bounds [m/s]. |
| `max_vel_y`, `min_vel_y` | `0.6`, `-0.6` | Body y velocity bounds [m/s]. |
| `max_vel_theta`, `min_vel_theta` | `1.0`, `-1.0` | Angular velocity bounds [rad/s]. |
| `max_accel_x`, `max_accel_y` | `1.0`, `1.0` | Positive symmetric acceleration/deceleration limits [m/s²]. |
| `max_accel_theta` | `2.0` | Positive symmetric angular acceleration/deceleration limit [rad/s²]. |
| `orientation_time_weight` | `1.0` | Orientation/translation time ratio. |
| `min_orientation_time` | `0.2` | Minimum orientation time [s]. |
| `velocity_feedback` | `OPEN_LOOP` | Previous command or measured velocity (`CLOSED_LOOP`). |
| `use_dynamic_window_vector_pursuit` | `true` | Select commands using the desired-velocity ray and dynamic window. |
| `use_uniform_velocity_scaling` | `false` | Enable the Scaled VP comparison when dynamic-window selection is disabled. |
| `use_collision_detection_for_omni` | `true` | Project the full holonomic motion when inherited collision checking is enabled. |
| `enable_csv_logging` | `false` | Record controller diagnostics. |
| `csv_log_directory` | `/tmp` | Writable log directory. |
| `csv_filename_prefix` | `dwvp_log` | Log filename prefix. |
| `csv_pose_frame` | `map` | Additional pose frame for logging. |

The velocity intervals must contain zero, and acceleration limits must be positive.
Package-specific parameters are read during lifecycle configuration, so reconfigure
the controller after changing them.

## Lateral response and preview

A positive `lateral_time_constant` adjusts the lateral response at the demand speed.
The realized response also depends on constraints, localization, and robot dynamics.
Choose the value with the robot's acceleration limits and localization noise in mind.

Fixed preview uses `lookahead_dist` with `use_velocity_scaled_lookahead_dist: false`.
Adaptive preview uses `lookahead_time`, bounded by `min_lookahead_dist` and
`max_lookahead_dist`. Lookahead follows the Jazzy Regulated Pure Pursuit circle
intersection. Desired yaw comes from the supplied path orientation.

The translation demand is separate from the `desired_linear_vel` speed-regulation
cap. A positive `vp_translation_speed` sets that demand directly. Automatic selection
uses the speed supported in all positive and negative x/y directions.

## Feedback and constraints

The control period is `1 / controller_frequency`. The dynamic window applies the
per-axis acceleration limits over that period.

`OPEN_LOOP` uses the previous command after the first cycle. A new plan, activation,
or failed command resets this feedback, and the first command uses measured velocity.
`CLOSED_LOOP` uses the measured twist every cycle. Jazzy `reset()` and cancellation
also invalidate the previous command. Cancellation completes immediately; Nav2
emits a stop command and downstream smoothing handles deceleration. Set
`use_cancel_deceleration: false`; RPP cancellation deceleration is not used. Configure the actual controller
rate and account for downstream smoothing when setting limits.

The bounds apply to computed commands relative to the selected feedback.
Physical tracking depends on the platform. If no command can satisfy both velocity
and acceleration bounds from the supplied feedback, the controller reports an error.

## Speed regulation and goal approach

The plugin inherits cost-based and goal-approach speed regulation from Nav2's
Regulated Pure Pursuit. Set `use_regulated_linear_velocity_scaling: false`, because
its PP curvature heuristic is not used by this omnidirectional controller.

`desired_linear_vel` and Nav2 speed limits cap the enclosing translation speed.
Cost and approach regulation reduce that cap and contract the x/y velocity bounds.
Angular bounds remain unchanged. If the reduced bounds cannot be reached in one
period, the controller selects a reachable braking endpoint.

Match `inflation_cost_scaling_factor` to the costmap inflation layer. The related
settings are `cost_scaling_dist`, `cost_scaling_gain`,
`regulated_linear_scaling_min_speed`, and `use_cost_regulated_linear_velocity_scaling`.

Keep `approach_velocity_scaling_dist` positive to slow near the goal. The example
uses 0.6 m. Inside the position tolerance, terminal control brakes translation and
aligns with the final path yaw. Position and yaw tolerances come from the goal checker.

## Comparison modes

| Mode | `use_dynamic_window_vector_pursuit` | `use_uniform_velocity_scaling` |
|---|---|---|
| DWVP selection | `true` | `false` |
| Clipped VP | `false` | `false` |
| Scaled VP | `false` | `true` |

The two switches cannot both be true. Error compensation is independently
controlled by `lateral_time_constant` in every mode. Set it to a nonpositive value
for an uncompensated comparison.

DWVP selection preserves the desired-velocity direction when the ray intersects
the feasible window. Otherwise, it selects the closest point using the unweighted
Euclidean distance in `(vx, vy, omega)`. Scaled VP first scales to velocity limits
and then clips to the acceleration window. All modes share terminal handling.

## Collision checking and logs

Enable both `use_collision_detection` and `use_collision_detection_for_omni` for
footprint projection that includes sideways motion and rotation. The projection
horizon uses `max_allowed_time_to_collision_up_to_carrot`. This check evaluates
the current commanded motion and does not plan an obstacle-avoidance path.

CSV diagnostics include the desired and selected velocities, physical and regulated
windows, and lateral-response geometry. `*_nav` columns are computed estimates,
so record downstream commands separately when inspecting the velocity smoother.
`response_rule` identifies `chord`, `tangent`, `goal`, or `inactive` handling.
The corresponding `lateral_error`, `chord_length`, and `response_distance` are
reported as `nan` when the response geometry is inactive.

## Existing installations

The package is now `nav2_ec_dwvp_controller`, and the plugin type is
`nav2_ec_dwvp_controller::ECDWVPController`. Update old build selections and YAML
plugin strings before rebuilding. Numerical comparison configurations retain their
recorded values, with the plugin identifier updated to the new name.

## Jazzy integration

A Lane Sampling Shim can load this plugin under `FollowPath.primary_controller`.
Place every EC-DWVP parameter under that same namespace. The shim supplies the
selected path, including pose orientations, and forwards Nav2 speed limits.

Enable lateral and reverse velocities in the downstream velocity smoother as
well as in this plugin. Match each axis's velocity and acceleration limits; do
not bypass the collision monitor or the robot's command watchdog. With a
velocity smoother, record both the controller command and the executed command
when evaluating the response.

The omni collision projection is published on `lookahead_collision_arc` and the
transformed path on `received_global_plan`. Neither topic indicates which
candidate the Lane Sampling Shim selected; use the shim's `selected_path` topic
for that purpose.
