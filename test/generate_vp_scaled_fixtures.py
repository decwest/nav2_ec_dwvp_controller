#!/usr/bin/env python3
# Copyright (c) 2026 Fumiya Ohnishi
# Licensed under the Apache License, Version 2.0.
"""Run with the read-only simulator's locked Python environment; output under /tmp."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from ec_dwvp.config import Config
from ec_dwvp.controller import compute_command
from ec_dwvp.paths import orientation_ramp_path
import ec_dwvp.controller as controller


def main():
    base = Config(lookahead_time=0.75, goal_xy=0.1, goal_yaw=0.3)
    path = orientation_ramp_path(0.3, length=2.5, start=1.0, spacing=0.01)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
    inputs = [
        ('within_limits', [0.5, 0., 0.], [.1, 0., 0.], base),
        ('angular_saturation', [1.12, 0., 0.], [.16, 0., .58], base),
        ('negative_lateral', [1.2, .05, .7], [.12, -.08, .6], base),
        ('half_acceleration', [1.2, .05, .7], [.12, -.08, .6],
         replace(base, ax=.11, ay=.11, aw=.3)),
        ('approach_regulation', [2.3, .1, 1.5], [.02, -.2, .1], base),
        ('sudden_velocity_cap', [1.12, 0., 0.], [.2, 0., .1],
         replace(base, desired_linear_vel=.04)),
        ('obstacle_regulation', [1.12, 0., 0.], [.2, 0., .1],
         replace(base, use_cost_regulation=True)),
    ]
    cases = []
    for name, pose, current, config in inputs:
        # Circle representation follows geometry.surface_distance.
        obstacles = []
        if name == 'obstacle_regulation':
            obstacles = [SimpleNamespace(x=1.12, y=.4, radius=.1)]
        result = compute_command(
            np.array(pose), np.array(current), path, arc, 'vp_scaled', config, obstacles)
        cases.append(dict(
            name=name, pose=pose, desired=result.desired.tolist(),
            physical_min=config.lower.tolist(), physical_max=config.upper.tolist(),
            acceleration=config.acceleration.tolist(), current=current, dt=config.dt,
            cap=result.speed_cap, prefer_large=True, expected_min=result.lower.tolist(),
            expected_max=result.upper.tolist(), expected_command=result.command.tolist()))
    source = Path(controller.__file__)
    Path(sys.argv[1]).write_text(json.dumps(dict(
        source=('ec_dwvp.controller.compute_command(method=vp_scaled), '
                'uncommitted working tree'),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        path='orientation_ramp_path(0.3, length=2.5, start=1.0, spacing=0.01)',
        cases=cases), indent=2) + '\n')


if __name__ == '__main__':
    main()
