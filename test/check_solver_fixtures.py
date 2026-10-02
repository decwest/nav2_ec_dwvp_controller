#!/usr/bin/env python3
# Copyright (c) 2026 Fumiya Ohnishi
# Licensed under the Apache License, Version 2.0.
"""Compare C++ commands against independently generated Python simulator fixtures."""
import json
import math
from pathlib import Path
import subprocess
import sys


def main():
    cases = json.loads(Path(sys.argv[2]).read_text())["cases"]
    lines = []
    for case in cases:
        values = sum((case[key] for key in (
            "desired", "physical_min", "physical_max", "acceleration", "current")), [])
        values += [case["dt"], case["cap"], int(case["prefer_large"])]
        lines.append(" ".join(format(value, ".17g") for value in values))
    result = subprocess.run(
        [sys.argv[1]], input="\n".join(lines) + "\n", text=True,
        capture_output=True, check=True)
    output = result.stdout.strip().splitlines()
    assert len(output) == len(cases), (len(output), len(cases))
    for case, line in zip(cases, output):
        actual = list(map(float, line.split()))
        expected = case["expected_min"] + case["expected_max"] + case["expected_command"]
        assert len(actual) == 9, (case["name"], actual)
        for a, b in zip(actual, expected):
            assert math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9), (case["name"], a, b)
    print(f"PASS: {len(cases)} shared Python/C++ box and command fixtures")


if __name__ == "__main__":
    main()
