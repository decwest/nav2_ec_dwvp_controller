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
| `use_raw_cost_regulated_linear_velocity_scaling` | `false` | Regulate the translation cap directly from combined local costmap values, replacing the inflation-distance heuristic. |
| `raw_cost_lookahead_dist` | `0.0` | Path sampling distance [m] from the robot. Zero samples only the current position. |
| `raw_cost_sample_step` | `0.05` | Distance [m] between raw cost samples. Must be positive. |
| `raw_cost_min_linear_velocity` | `0.1` | Minimum raw-cost-regulated cap [m/s], limited to the incoming cap. This is not a minimum command speed. |
| `use_collision_detection_for_omni` | `true` | Project the full holonomic motion when inherited collision checking is enabled. |
| `collision_prediction_mode` | `constant_velocity` | Legacy command extrapolation, or `closed_loop` tracking prediction with a stopping-distance governor. |
| `collision_prediction_time_buffer` | `2.0` | Extra prediction time [s] after the maximum configured or observed stopping time; closed-loop mode only. |
| `collision_stop_margin` | `0.3` | Extra swept travel [m] to reserve before predicted contact or the costmap boundary; closed-loop mode only. |
| `collision_prediction_max_steps` | `300` | Maximum control updates per rollout. Insufficient coverage rejects the command rather than silently shortening prediction. |
| `enable_csv_logging` | `false` | Record controller diagnostics. |
| `csv_log_directory` | `/tmp` | Writable log directory. |
| `csv_filename_prefix` | `dwvp_log` | Log filename prefix. |
| `csv_pose_frame` | `map` | Additional pose frame for logging. |

The velocity intervals must contain zero, and acceleration limits must be positive.
Package-specific parameters are read during lifecycle configuration, so reconfigure
the controller after changing them.
Raw cost distances and minimum velocity must be finite and nonnegative, and the
sample step must be finite and positive. Invalid values reject configuration.

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

`OPEN_LOOP` uses the previous successful command after the first cycle. All valid
path updates preserve this feedback, including changed goal positions/headings
and temporary one-point stop paths. The new target is evaluated using the dynamic
window around that command, so a stop path decelerates and a replacement goal
changes the command within the acceleration limits.
Activation, failed commands, invalid empty paths, or a command gap exceeding
`max(0.1 s, 3 / controller_frequency)` resets this feedback; the next command uses
measured velocity.
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

### Raw cost regulation

To slow directly from the combined local costmap, enable raw cost regulation under
the controller ID (including nested IDs such as `FollowPath.primary_controller`):

```yaml
use_raw_cost_regulated_linear_velocity_scaling: true
raw_cost_lookahead_dist: 1.0
raw_cost_sample_step: 0.05
raw_cost_min_linear_velocity: 0.1
```

When enabled, the controller bypasses RPP's inflation-distance cost heuristic,
even if `use_cost_regulated_linear_velocity_scaling` is true. Goal-approach
regulation and Nav2 speed limits still apply. The order is nominal cap, goal
approach, raw cost reduction, then the existing x/y window contraction and command
selection. Disabling raw cost regulation preserves the inherited behavior.

Sampling starts at the robot origin, follows the segment to the first local path
point, and continues along the path up to `raw_cost_lookahead_dist`. Sparse path
segments are interpolated; duplicate points do not add distance. As in
`nav2_raw_cost_rpp_controller`, the path endpoint and horizon are not added as
extra samples if they fall between sampling intervals. A step larger than the
horizon samples the horizon once; steps at or below 1e-6 m use the horizon as the
step. An empty or shorter-than-one-step path contributes only the robot origin.

The largest sampled cost other than `FREE_SPACE` (0) and `NO_INFORMATION` (255)
sets the reduction. `LETHAL_OBSTACLE` (254) is capped at 253 for this calculation:

```text
v = cap after goal-approach regulation
min_v = min(v, raw_cost_min_linear_velocity)
v_reg = v - (v - min_v) * min(cost, 253) / 253.0
```

Out-of-map samples are skipped. If the robot pose cannot be transformed into the
costmap frame, or no regulating cost is found, this stage leaves the incoming cap
unchanged. It does not fall back to the inflation-distance heuristic. Collision
checking remains separate and can reject a command even when this stage returns
a positive cap. In particular, unknown and lethal cells retain their existing
collision-checker treatment.

The regulated cap contracts the axis velocity bounds; it is not a lower bound on
the command magnitude. For equal x/y limits, an axis-aligned command can be as low
as `v_reg / sqrt(2)` even without acceleration or goal constraints. After a sudden
cost increase, acceleration limits can also require several cycles to reach the
new cap. Terminal rotation and collision braking retain their existing behavior.

The lateral compensation demand continues to use
`lateral_time_constant * demand_speed` as its response distance. Raw cost reduction
does not recompute that demand, so the realized lateral convergence can take
longer while moving slowly. The existing feasible-command selection still
determines whether the desired velocity direction can be maintained.

Closed-loop prediction samples the same frozen costmap from each virtual robot
pose, including that pose's orientation, using the same reduction as actual control.
These are path-point samples, not a swept footprint evaluation. Semantic layers
remain responsible for converting attributes to cost values; the controller has
no semantic-layer dependency and cannot distinguish the sources of combined costs.

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
footprint projection that includes sideways motion and rotation.

`constant_velocity` retains the previous behavior: hold the candidate command,
project for `max_allowed_time_to_collision_up_to_carrot` (also capped at the
initial carrot), and reject on collision. Existing configurations keep this mode
unless explicitly changed. The example and Castella lane configuration select
`closed_loop`.

### Closed-loop prediction and braking

At every simulated control period, recompute the same EC-DWVP control step used
for the real command: preview, reference projection, lateral compensation,
cost/approach regulation, dynamic window, velocity selection and terminal control.
The preview moves along the path; it does not terminate prediction. TF, reference
path, costmap and tolerances are fixed at the real control cycle. Prediction has
its own pruning cursor and never updates real feedback, GoalChecker or plan state.

The horizon is `max(max_axis_speed / axis_acceleration, observed_stop_time) +
collision_prediction_time_buffer`, with at least one control period. Both selected
feedback and measured velocity contribute to the horizon. With limits of 0.5 on
all axes and a 2 s buffer, this is 3 s (90 updates at 30 Hz). The legacy
`max_allowed_time_to_collision_up_to_carrot` is unused in this mode.

The governor measures swept travel as `integral(hypot(vx, vy) + R * abs(omega)) dt`,
where R is the footprint's circumscribed radius. Absolute angular travel avoids
cancellation when rotation reverses. The stopping model scales all three velocity
components to zero together in `T = max(abs(component) / axis_acceleration)`.
Required travel is `swept_speed * (control_period + T / 2)`, including one command
period. For straight motion this reduces to `v * dt + v^2 / (2a)`.

If the first contact is farther away than required stopping travel plus the
margin, the first command is unchanged by this governor. Otherwise solve the
quadratic stopping bound for the largest common velocity scale, then enforce the
reachable regulated window. The default 0.3 m margin is additional **travel along
the predicted motion**, not a guaranteed 0.3 m Euclidean clearance on every side.
Pure rotation uses outside-footprint travel in the same units.

A second rollout verifies the governed commands and updates their remaining stop
budget after every step. If window clipping prevents sufficient braking, the
modified trajectory collides, TF/path calculation fails, or the required horizon
exceeds the step limit, report `NoValidControl`. Ordinary governed slowing/stopping
returns a normal reachable command, not an exception. The costmap boundary is a
stopping boundary; a partially uncovered footprint is not assumed clear. Unknown
cell and lethal-cost handling otherwise retain the inherited collision checker.

Each command is held for one period and integrated with holonomic motion.
Collision samples are added only when swept motion exceeds half a grid cell;
they do not recompute the controller. First contact is refined with eight bisection
checks to prevent sampling jitter in the stopping target. There are at most two
rollouts. Remaining stop budgets within `resolution / 512` are rounded toward an
earlier stop, still subject to acceleration limits, to avoid microscopic restart
commands from contact-refinement noise. A stationary terminal or governed stop
can finish early.

This predicts ideal command following. It does not model downstream smoother
timing, actuator lag, future pedestrian motion, or future lane replanning. The
cost/goal speed regulation and the downstream Collision Monitor remain active.
The method does not search for a different obstacle-avoidance path.

### Diagnostics

`lookahead_collision_arc` is the final checked trajectory; an unsuccessful
verification publishes its checked prefix, not a claim of safety. The controller
namespace also contains:

* `nominal_collision_arc` (`nav_msgs/Path`): ungoverned tracking prediction.
* `collision_prediction` (`diagnostic_msgs/DiagnosticArray`): status (`clear`,
  `obstacle_ahead`, `limited`, or a rejection reason), nominal event, collision
  travel, candidate/selected stopping travel, stop margin, requested velocity
  scale, horizon, rollout update counts and prediction processing time in ms.
* `collision_costmap` and `collision_command`: the exact frozen grid and nominal
  command when reduced or rejected, published only when subscribed. These are
  diagnostic inputs, not actuator commands. The command can be reduced rather
  than rejected in closed-loop mode.

Headers share the real control pose time. Predicted poses do not request future
TF transforms. An infinite collision distance means no contact was found inside
the checked horizon. `speed_scale` is the requested common scale before window
clipping; the selected stopping distance reports the reachable result.
`compute_time_ms` measures prediction/governing, excluding the initial real control
step and TF snapshot. Use end-to-end controller timing for deadline checks.
Castella's existing `bagrec nav` profile records these namespace topics.

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
