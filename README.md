# nav2_dynamic_window_vector_pursuit_controller

Nav2 controller plugin for Dynamic Window Vector Pursuit (DWVP) on omni-directional robots.

## Plugin name

```yaml
plugin: "nav2_dynamic_window_vector_pursuit_controller::DynamicWindowVectorPursuitController"
```

## Main parameters

- `max_vel_x`, `min_vel_x`
- `max_vel_y`, `min_vel_y`
- `max_vel_theta`, `min_vel_theta`
- `max_accel_x`, `max_accel_y`, `max_accel_theta`
- `orientation_time_weight`
- `min_orientation_time`
- `velocity_feedback` (`OPEN_LOOP` or `CLOSED_LOOP`)
- `use_dynamic_window_vector_pursuit` (`true` / `false`)
- `use_collision_detection_for_omni` (`true` / `false`)
- `enable_csv_logging` (`true` / `false`)
- `csv_log_directory` (directory path for csv files, default: `nav2_dynamic_window_vector_pursuit_controller/data`)
- `csv_filename_prefix` (file name prefix)

When CSV logging is enabled, each row contains:
- `sec,nsec,x,y,yaw`
- scalar velocities: `v_real,w_real,v_now,w_now,v_cmd,w_cmd,v_nav,w_nav`
- omni components: `vx_real,vy_real,vx_now,vy_now,vx_cmd,vy_cmd,vx_nav,vy_nav`
- constraint / control terms: `velocity_violation,curvature,dw_vx_max,dw_vx_min,dw_vy_max,dw_vy_min,dw_w_max,dw_w_min,v_reg,lookahead_dist,lookahead_yaw,is_accel`

The plugin also reuses RPP parameters such as:
- `lookahead_dist`, `min_lookahead_dist`, `max_lookahead_dist`, `lookahead_time`
- `use_velocity_scaled_lookahead_dist`
- `max_robot_pose_search_dist`, `transform_tolerance`

## Notes

- `use_collision_detection_for_omni` is `false` by default because Nav2's default RPP collision model is based on forward arc motion (`vx`, `w`) and does not model lateral motion (`vy`) exactly.
