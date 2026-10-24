# Jazzy port verification

The `jazzy` branch starts at Humble `main` commit
`61f4a6ac7415d56bc24f084bed3caf288b8c5a09`. The plugin identifier and EC-DWVP
parameters are unchanged. The Humble implementation remains on `main`.

## Interface changes

- RPP parameters, path transforms and footprint checks use Jazzy's
  `ParameterHandler`, `PathHandler` and `CollisionChecker`.
- EC-DWVP serializes plan/command history with its own mutex and takes the RPP
  parameter mutex while computing commands. `setSpeedLimit()` delegates to the
  base class without acquiring that same mutex twice.
- The omnidirectional collision projection retains its own lifecycle publisher;
  Jazzy's RPP collision checker owns a separate publisher internally.
- Controller errors use Jazzy exception types. Empty input plans are rejected.
- `reset()` and immediate `cancel()` discard command history. The next command
  is initialized from measured velocity. RPP cancellation deceleration is unused;
  the downstream velocity smoother handles the stop command.

Path orientations, error compensation, dynamic-window selection, omnidirectional
collision projection and terminal control calculations are retained.

## Local verification, 2026-10-24

Built against the installed Jazzy Nav2 in `decwest/ytlab_castella_jazzy`.
All 32 C++ tests passed, including plugin loading, nested controller parameters,
speed limits, path updates, reset/cancel/resume, lateral/reverse motion and collision
checks. All three numerical fixture suites passed at their existing tolerance:

| Fixture | Cases | SHA-256 (unchanged from Humble) |
|---|---:|---|
| `solver_cases.json` | 107 | `e46075ce9983059fdbcc292a22e9594955145aa604dae6eaa88747b8cd499dce` |
| `vp_scaled_cases.json` | 7 | `33c025d272e369bd5980078611bb662ce7105dd2951230b72d1583c95adb107e` |
| `desired_vector_cases.json` | 192 | `a3bf9b8911f75afe25356bf0b11ab32e9f8dc8f9a417607373b1d701ab011d8a` |

The inherited cost-regulation test uses Jazzy's cost factor 253 instead of Humble's
252. Its expected speeds remain 0.4 and 0.2 m/s; the tolerance is 1e-7 because
Jazzy evaluates `log(253.0f)` in single precision. This affects the inherited cost
conversion, not the EC-DWVP numerical fixtures.

Run `scripts/verify_jazzy.sh` to build and test in a separate container. The source
mount is read-only, and no robot network is used. The Jazzy workflow runs the same
build and test commands on GitHub; local success does not certify a remote CI run.

Castella's integration report records simulator tests separately from these package
tests. Neither establishes real-robot tracking performance or comfort.
