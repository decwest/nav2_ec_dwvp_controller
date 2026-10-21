# Nav2 Omnidirectional DWVP Controller

ROS 2 **Humble** controller plugin for acceleration-constrained omnidirectional
path tracking in body-frame `(vx, vy, omega)` velocity space.

DWVP selects a command from the per-axis dynamic window by intersecting the
reference-velocity ray with its box. When there is no intersection, it selects
the closest point using the original **unweighted Euclidean metric** in the raw
velocity coordinates. No velocity-limit normalization or sampling is used in
this selection. The ray scale preserves translation/yaw ratios when feasible.

## Build and use

```bash
cd ~/ros2_ws/src
git clone https://github.com/decwest/nav2_omnidirectional_dwvp_controller.git
cd ..
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --packages-select nav2_omnidirectional_dwvp_controller
source install/setup.bash
```

Plugin type:

```yaml
plugin: "nav2_omnidirectional_dwvp_controller::OmnidirectionalDWVPController"
```

Merge [config/example_params.yaml](config/example_params.yaml) into a complete
Nav2 configuration. This package supplies a controller, not a robot launch file.
The example is installed under the package share directory. Input paths are
`nav_msgs/Path` poses with valid quaternions specifying the desired robot yaw;
yaw can differ from the path tangent. Commands are `geometry_msgs/TwistStamped`.

Only Humble is supported and tested. RPP internal APIs differ in newer Nav2
versions; a Jazzy or Rolling build is not claimed.

## Paper experiments

The following files contain the controller-server, velocity-smoother and local-costmap
settings from the recorded HSR experiments. Each file is directly under `config/`
and is installed in the package share directory. Merge one file into your robot's
Nav2 configuration and adapt the HSR topics and frames to your system.

| Experiment | Parameter file | Conditions |
|---|---|---|
| 1a | [paper_experiment1a_params.yaml](config/paper_experiment1a_params.yaml) | EC-DWVP, L = 0.11 or 0.33 m and tau_y = 0.5 or 1.5 s |
| 1b | [paper_experiment1b_a0p6_params.yaml](config/paper_experiment1b_a0p6_params.yaml) | Clipped VP, Scaled VP and EC-DWVP, x/y acceleration limit 0.22 m/s² and angular acceleration limit 0.6 rad/s² |
| 1b | [paper_experiment1b_a0p3_params.yaml](config/paper_experiment1b_a0p3_params.yaml) | The same three methods, x/y acceleration limit 0.11 m/s² and angular acceleration limit 0.3 rad/s² |
| 1b | [paper_experiment1b_a0p15_params.yaml](config/paper_experiment1b_a0p15_params.yaml) | The same three methods, x/y acceleration limit 0.055 m/s² and angular acceleration limit 0.15 rad/s² |
| 2 | [paper_experiment2_params.yaml](config/paper_experiment2_params.yaml) | Clipped VP, EC-DWVP, MPPI and DWB on the indoor path |

Select a controller with `FollowPath.controller_id`. Experiment 1a provides four
IDs, such as `EC_DWVP_L0p11_T0p5`. Experiment 1b provides `VP_CLIP`, `VP_SCALED`
and `EC_DWVP`, with lateral compensation disabled for all three. Experiment 2
provides `VP_CLIP`, `EC_DWVP`, `MPPI` and `DWB`. Use the matching Experiment 1b
file for each acceleration condition so the controller and smoother limits agree.
The path geometry and initial conditions are described in each file's header.

The files load the controllers directly. The experiments used a timing wrapper
that delegated to these same plugins with the same controller parameters.
The wrapper and its experiment package are not needed to load these settings.
Experiment 2 also requires the standard Nav2 MPPI and DWB packages and the
[DWB path tracking critics](https://github.com/decwest/dwb_path_tracking_critics).
Its DWB settings use the final position and orientation critic weights of 64 and
16, respectively. Experiment 1 disables collision projection and cost-based speed
regulation, as recorded in the tracking trials. Experiment 2 keeps collision
projection enabled for Clipped VP and EC-DWVP.

## Motion and regulation

The physical velocity box must contain zero on every axis. Each positive
`max_accel_*` sets a symmetric acceleration/deceleration bound over one configured
controller period. The guarantee concerns commanded velocities relative to the
selected feedback; it does not assert that the physical robot follows them
without delay or error. Nav2 action cancellation/goal-stop commands and downstream
safety stops are outside this per-cycle controller guarantee. If a feedback sample is so far outside the physical
limits that no acceleration-feasible command exists, the controller reports an
error rather than returning a command that violates one of the limits.

`v_box = hypot(max(abs(min_vel_x), abs(max_vel_x)),
max(abs(min_vel_y), abs(max_vel_y)))` defines the enclosing translational speed.
The VP demand magnitude is `vp_translation_speed`, with automatic inscribed-speed
selection for values <= 0 (see below).
Translation points toward the lookahead position. Reference yaw rate is the
wrapped lookahead yaw error divided by
`max(min_orientation_time, orientation_time_weight * distance / vp_translation_speed_effective)`;
the time floor also handles zero distance and a pure-rotation configuration.

The inherited `desired_linear_vel` and Nav2 absolute/percentage speed limits
always constrain the nominal translational speed to `min(desired_linear_vel,
v_box)`. Cost proximity and goal approach regulation reduce this speed. For a
resulting limit `v_reg`, the physical x/y bounds are scaled by
`rho = clamp(v_reg / v_box, 0, 1)` before intersecting the dynamic window. The yaw
bounds remain unchanged. The resulting rectangle is a conservative
translational speed envelope, not an exact circular speed constraint.

If a suddenly reduced envelope cannot be reached in one step, the offending
axis collapses to the nearest **original dynamic-window endpoint**. Acceleration
limits therefore take precedence over an instantaneous regulation target;
speed can temporarily exceed that target during braking. Pure rotation remains
available. The nominal speed and cap constrain the feasible box, not merely the
reference vector: scaling an unbounded ray alone would have no effect.

Cost regulation uses the inherited RPP inflated-cost distance estimate (robot
center to obstacle), speed floor, and scaling distance/gain. Approach scaling
acts on the already cost-regulated speed and applies its own minimum approach
speed. The final value is capped by the nominal speed. Match
`inflation_cost_scaling_factor` to the costmap inflation layer.
**Curvature-based RPP regulation is not applied.** Set
`use_regulated_linear_velocity_scaling: false`; body heading and translational
path direction need not coincide for an omnidirectional platform.

At the actual final goal's position tolerance, translation targets zero and yaw
uses a finite target bounded by both the orientation-time rule and angular
stopping speed. This target is clipped into the same feasible window, including
translational braking. The configured goal checker supplies position/yaw
tolerances. Goal orientation is taken from the final path pose.

To reproduce the unregulated ray/box algorithm, use `desired_linear_vel >=
v_box`, disable cost regulation, and set `approach_velocity_scaling_dist: 0.0`.
The corrected finite-distance/goal behavior still applies.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `max_vel_x`, `min_vel_x` | `0.6`, `-0.6` | Body x velocity bounds (m/s). |
| `max_vel_y`, `min_vel_y` | `0.6`, `-0.6` | Body y velocity bounds (m/s). |
| `max_vel_theta`, `min_vel_theta` | `1.0`, `-1.0` | Yaw-rate bounds (rad/s). |
| `max_accel_x`, `max_accel_y` | `1.0`, `1.0` | Positive symmetric acceleration limits (m/s²). |
| `max_accel_theta` | `2.0` | Positive symmetric angular acceleration limit (rad/s²). |
| `orientation_time_weight` | `1.0` | Positive orientation/translation time ratio. |
| `min_orientation_time` | `0.2` | Positive minimum orientation time (s). |
| `velocity_feedback` | `OPEN_LOOP` | Previous command, or measured velocity with `CLOSED_LOOP`. |
| `use_dynamic_window_vector_pursuit` | `true` | Set false for component-clipped VP using the same regulated feasible box. |
| `use_uniform_velocity_scaling` | `false` | With DWVP disabled, uniformly scale the VP demand to regulated velocity bounds before dynamic-window clipping. |
| `use_collision_detection_for_omni` | `true` | Full holonomic footprint projection when inherited `use_collision_detection` is true. |
| `enable_csv_logging` | `false` | Optional controller diagnostics. |
| `csv_log_directory` | `/tmp` | Writable CSV destination. |
| `csv_filename_prefix` | `dwvp_log` | Filename prefix. |
| `csv_pose_frame` | `map` | Optional additional logged pose frame. |

Supported inherited settings include `desired_linear_vel`, fixed/adaptive
lookahead parameters, `use_interpolation`, transform/path-search settings,
`use_cost_regulated_linear_velocity_scaling`, cost settings,
`regulated_linear_scaling_min_speed`, `approach_velocity_scaling_dist`,
`min_approach_linear_velocity`, `use_collision_detection`, and
`max_allowed_time_to_collision_up_to_carrot`. The PP-only rotate-to-heading,
reversing gate, and curvature speed heuristic do not control DWVP behavior.
DWVP-specific parameters are read on lifecycle configuration; reconfigure to
change them. Inherited RPP runtime parameters and `setSpeedLimit()` retain the
Humble interfaces.

Every new plan, activation, and failed command invalidates stale open-loop feedback. The first
command uses measured velocity; subsequent open-loop cycles use the previous
command. Closed-loop mode uses the measured twist every cycle. The period is
`1 / controller_frequency`, so configure the actual command rate and record
missed deadlines in experiments.

Lookahead position uses Humble RPP's first outer point in a robot-centered
lookahead circle, with optional circle/segment interpolation. Yaw uses the first
outer pose quaternion. It is not recomputed from the tangent. DWVP always selects
the largest alpha at an intersection, or the closest point to the ray with
larger alpha breaking equal-distance ties. Goal approach regulation narrows the
translational window; terminal control handles final braking and yaw alignment.
Keep `approach_velocity_scaling_dist > 0` for DWVP, clipped VP and uniformly scaled VP (the example
and experiment profiles use 0.6 m). Disabling it can cause goal overshoot: the
nominal straight-path Python model overshoots by about 0.09 m without it. This is
simulation evidence, not a measured HSR stopping distance.

## Collision checking and recording

Collision prediction integrates the complete constant body twist, including
lateral and reverse translation and rotation. Sample spacing accounts for
translational motion plus footprint rotational sweep. The projection checks the
current footprint and continues up to the configured time/carrot distance;
leaving costmap coverage is treated as a collision. This is a finite-horizon
collision check, not a global obstacle-avoidance planner or braking-distance
certificate.

Optional CSV diagnostics retain the existing columns and their order. `dw_*` columns are
the original physical/acceleration window; `v_reg` is the requested scalar cap.
`*_nav` columns are computed clipping estimates, **not observed downstream
commands**. Use a common external recorder for raw controller output, actual
smoothed output, measured odometry, reference poses, result status, and
configuration/revision metadata when comparing different controllers.

Four columns are appended: `response_rule` (`chord`, `tangent`, `goal`, or
`inactive`), `lateral_error` (signed `e`, m), `chord_length` (`L_prime`, m), and
`response_distance` (`D_y`, m). Inactive and terminal cycles report `nan` for
these three geometry values. Compatibility with the external experiment
reader has not been verified; readers must accept additional trailing columns.

## Verification

```bash
colcon test --packages-select nav2_omnidirectional_dwvp_controller --return-code-on-test-failure
colcon test-result --verbose
```

The tests cover ray intersection and projection against an independent convex
oracle, acceleration/velocity invariants, regulation and sudden caps, external
speed limits, goal-only rotation, lifecycle feedback, footprint collisions, and
pluginlib loading. Shared fixtures in `test/solver_cases.json` also compare C++
results with the Python simulator in
[omnidirectional_dwvp](https://github.com/decwest/omnidirectional_dwvp).
The `solver_parity` and `vp_scaled_parity` checks remain enabled.
`desired_vector_parity` exercises the plugin's projection, Humble RPP preview,
response-rule selection and desired velocity on all 192 cases in
`test/desired_vector_cases.json`, copied unchanged from simulator commit
`b09ba8e3a2a5feced5ba317adf793b0e33e69591`. It checks the selected target, nearest
pose, goal-region flag, rule and body-frame demand with absolute and relative
tolerances of `1e-9`. GTests also cover exact disabled-demand arithmetic,
straight-path compensation, duplicate segments, retention across RPP pruning,
and unchanged terminal commands in all three controller modes.

For an existing Humble development image containing Nav2 and test dependencies:

```bash
DWVP_HUMBLE_IMAGE=docker-hsr:latest ./scripts/verify_humble.sh
```

The verification container has no network, GPU, robot access, or writable source
mount. Set `DWVP_VERIFY_WORKSPACE` to reuse its build artifacts. GitHub Actions
builds and tests against the public `ros:humble-ros-base` image with rosdep.

Licensed under [Apache License 2.0](LICENSE).

### VP translation demand

`vp_translation_speed` (double, m/s, default `0.0`) sets the translation
magnitude in both the desired vector and the position-to-orientation travel
time. A finite value <= 0 selects `min(max_vel_x, -min_vel_x, max_vel_y,
-min_vel_y)`, including asymmetric reverse limits. A positive value is a demand,
not an additional speed cap. It is read at configure time.

DWVP selects on a ray and permits alpha > 1. Changing this
magnitude preserves its output when the entire desired vector scales uniformly.
Near the lookahead point, `min_orientation_time` can clamp angular demand; then
translation changes without the same angular scaling, so the ray and DWVP output
can change. The finite final-goal rotation branch is unchanged. The VP_CLIP
comparison uses the existing component clipping branch, followed by the common
external velocity smoother; clipping still uses the plugin's regulated window.

The Python simulator uses the same positive-value meaning and default inscribed
speed. Its explicit `0` means zero translation and negative values are rejected;
this plugin follows the hardware contract of automatic selection for all <= 0.
The simulator has not been modified.

### Lateral response time

`lateral_time_constant` (finite double, seconds, default `0.0`) is read at
configure time. Values <= 0 preserve the original VP computation and commands.
A positive value enables error compensation for DWVP, VP_CLIP and VP_SCALED,
using `D_y = lateral_time_constant * v_vp`. Here `v_vp` is the demand magnitude
from `vp_translation_speed`, or the inscribed speed when that parameter is <= 0.
A zero demand keeps translation zero and compensation inactive.

Let `p_r` be the closest-segment projection, `t_hat` its unit tangent,
`n_hat = (-t_hat.y, t_hat.x)` its left normal, and
`e = dot(n_hat, robot.xy - p_r)`. With the unchanged lookahead target `p_L`,
`L_prime = norm(p_L - p_r)` and `c_hat = (p_L - p_r) / L_prime`, the demand is

```text
u = unit(c_hat - (e / D_y) * n_hat)
(vx_des, vy_des) = v_vp * u   # expressed in the robot frame
```

On a straight path at small lateral error, this parameter is the first-order
lateral-error time constant at the demand speed. Actual response also depends
on the selected command and its limits. At an interior orthogonal projection,
`D_y = L_prime` gives the original VP direction; this identity does not generally
hold at a clamped endpoint with tangential error.

Outside the goal preview region, a chord shorter than `1e-9` m or a preview
selecting the nearest path pose uses `t_hat` in place of `c_hat`. If no retained
pose reaches the lookahead radius and the retained path includes the actual
plan end, the goal rule uses `min(D_y, L_prime)`. A vanishing goal chord instead
aims directly at the target, keeping zero translation within `1e-6` m. A local
costmap crop alone does not enter the goal rule. The angular demand still uses
the robot-to-target distance and the original orientation time calculation.
Terminal control inside the goal tolerance is unchanged.

Projection searches the complete plan saved by `setPlan`, independently of
RPP's pruned preview suffix. Thus the segment ending at the closest pose is
retained even on subsequent cycles. Projection fractions are clamped to [0,1],
and equal squared distances select the earliest segment, including duplicates.
A duplicate inherits the nearest nonzero segment's tangent by index distance,
with ties toward the earlier segment; a constant path uses the plan-frame +x
direction. Geometry is projected in the plan frame, then expressed in the robot
frame for the demand calculation.

For tuning, start with a time constant at least
`max(v_vp / (2 * a_y), sigma_xy / (a_y * dt))`, where `sigma_xy` is position-noise
standard deviation and `dt` is the controller period. These are acceleration
and noise estimates, not closed-loop guarantees. For example, `0.75` s exceeds
both estimates at `v_vp = 0.22` m/s, `a_y = 0.22` m/s², `sigma_xy = 0.005` m and
`dt = 1/30` s.

### Uniformly scaled VP comparison

| Mode | `use_dynamic_window_vector_pursuit` | `use_uniform_velocity_scaling` |
|---|---|---|
| DWVP | true | false |
| VP_CLIP | false | false |
| VP_SCALED | false | true |

Enabling both switches is rejected at configuration. VP_SCALED uses exactly the
simulator's `vp_scaled` sequence: compute the common VP demand, choose the largest
common multiplier in [0,1] that fits the **regulated physical velocity box**
(cost/approach caps included, acceleration excluded), then clip each component
to the same regulated dynamic window used by VP_CLIP/DWVP. A sudden cap may be
unreachable; the common window retains its braking endpoint. Thus direction is
preserved by the first scaling step, but subsequent acceleration clipping can
change it. Final-goal handling is shared with VP_CLIP.

`test/vp_scaled_cases.json` adds seven `compute_command(method='vp_scaled')`
references from the unmodified simulator working tree, including angular
saturation, half acceleration, cost/approach regulation and a sudden cap.
`test/generate_vp_scaled_fixtures.py` regenerates them using the simulator's
locked Python 3.11.11 environment (`uv run --offline --locked --python 3.11.11`)
inside Docker with `--network none`, read-only simulator/source mounts and an
output path under `/tmp`. The fixture records the simulator controller SHA-256.
The existing `test/solver_cases.json` is unchanged. GTests additionally cover
unchanged in-limit demand, a shared scale for angular saturation, 1000 random
reachable-window checks, and identical clipped/scaled terminal commands.
