#!/usr/bin/env python3
# Copyright (c) 2026 Fumiya Ohnishi
# Licensed under the Apache License, Version 2.0.
"""
Check desired velocities, RPP targets and rules against 192 simulator fixtures.

Copied unchanged from omnidirectional_dwvp commit
b09ba8e3a2a5feced5ba317adf793b0e33e69591, fixtures/desired_vector_cases.json.
"""
import json
import math
from pathlib import Path
import subprocess
import sys


def main():
    cases = json.loads(Path(sys.argv[2]).read_text())['cases']
    assert len(cases) == 192, len(cases)
    lines = []
    for case in cases:
        values = [len(case['path'])] + case['pose'] + [
            case['lookahead_distance'], case['response_distance'] or 0.0,
            case['translation_speed'], case['orientation_time_weight'],
            case['min_orientation_time']] + sum(case['path'], [])
        lines.append(' '.join(format(value, '.17g') for value in values))
    result = subprocess.run(
        [sys.argv[1]], input='\n'.join(lines) + '\n', text=True,
        capture_output=True, check=True)
    output = result.stdout.strip().splitlines()
    assert len(output) == len(cases), (len(output), len(cases), result.stderr)
    for case, line in zip(cases, output):
        actual = line.split()
        assert len(actual) == 9, (case['id'], actual)
        x, y, yaw = case['pose']
        tx, ty, heading = case['target']
        c, s = math.cos(yaw), math.sin(yaw)
        expected = case['expected_desired_vector'] + [
            c * (tx - x) + s * (ty - y), -s * (tx - x) + c * (ty - y),
            (heading - yaw + math.pi) % (2 * math.pi) - math.pi]
        for index, (a, b) in enumerate(zip(map(float, actual[:6]), expected)):
            if index == 5:
                a = (a + math.pi) % (2 * math.pi) - math.pi
            assert math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9), (
                case['id'], index, a, b)
        assert actual[6] == (case['response_rule'] or 'inactive'), (case['id'], actual)
        assert int(actual[7]) == case['goal_region'], (case['id'], actual)
        assert int(actual[8]) == case['nearest_pose'], (case['id'], actual)
    print(f'PASS: {len(cases)}/{len(cases)} desired-vector, preview and rule fixtures; '
          'abs_tol = rel_tol = 1e-9')


if __name__ == '__main__':
    main()
