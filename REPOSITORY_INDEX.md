# Repository Index

This package contains the complete project archive assembled from the drone project history and surviving files.

## Main implementation

- `software/final/phase7_depth_pulse_follow.py` — latest large Phase 7 movement/follow-control script.
- `software/companion/keep_person_centered.py` — surviving companion-computer yaw-centering/altitude-guard source.
- `software/companion/full_follow_mission.py` — integrated follow-mission implementation from the project test progression.

## Test progression

- `tests/diagnostics/` — MAVLink, arming, RC, EKF and safety diagnostics.
- `tests/real_flight/` — takeoff, landing, yaw, centering, altitude and GPS-denied experiments.
- `tests/motion/` — forward/back and Phase 7 pulse-follow experiments.
- `tests/vision/` — OAK-D person, depth and obstacle/depth-grid tests.
- `tests/sitl/` — ArduCopter software-in-the-loop tests.

## Flight-controller history

- `ardupilot/parameters/` — representative ArduPilot parameter snapshots for GPS, optical-flow and calibration stages.

## Evidence

- `evidence/console-logs/` — preserved OAK-D and RC dry-run console output.

## Notes on older test files

Some early standalone files survived directly, while others were restored from the project's recorded code, console output and chat history when the original standalone file was no longer present. `tests/TEST_PROVENANCE.json` records that distinction.
