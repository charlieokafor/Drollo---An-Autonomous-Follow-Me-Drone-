# Recovered Code and Project Artifacts

This folder set was assembled from the archived files and test outputs that remained available from the autonomous-drone project.

## Recovered source code

### `software/companion/keep_person_centered.py`

This is the strongest complete Python source artifact recovered from the project archive. It contains the real companion-computer architecture used during the later prototype stage:

- DepthAI/OAK-D person detection;
- `pymavlink` connection to the flight controller;
- LiDAR `DISTANCE_SENSOR` altitude input;
- battery/status-message monitoring;
- ArduPilot mode switching and arming;
- guided takeoff to approximately 1 m;
- `SET_POSITION_TARGET_LOCAL_NED` commands in `MAV_FRAME_BODY_NED`;
- proportional yaw centering;
- last-seen-side target reacquisition;
- altitude guards;
- autonomous LAND behavior.

The archived artifact did not preserve its original filename, so it has been given a descriptive repository filename. Only the provenance comment at the top was added; the recovered control body is unchanged.

## Recovered ArduPilot configuration

The `ardupilot/parameters/` directory contains three original `.param` snapshots from different stages of development. Keeping more than one snapshot is useful here because the project deliberately moved between GPS-supported and optical-flow/GPS-denied experiments.

## Recovered test evidence

The `evidence/console-logs/` directory contains original console output from:

- `oak_person_test.py` — OAK-D person detection and stereo-depth testing;
- `real_oak_rc6_dry_run.py` — real-FC/RC6/OAK-D dry-run integration that printed intended yaw and forward/back decisions without arming or moving motors.

These are **test outputs**, not restored from project history source files.

## Source files known to have existed but not present as recoverable standalone files

Project history shows that the following files were written or run during development, but their complete original source was not present in the available file archive during this recovery pass:

- `oak_person_test.py`
- `real_oak_rc6_dry_run.py`
- `real_oak_motor_response_test.py`
- `arm_failure_debug.py`
- `phase7_depth_pulse_follow.py`
- `testtest.py`
- earlier SITL mission scripts
- Raspberry Pi Flask/web-control server source
- Expo/React Native **Drolo** mobile-app source

I have intentionally **not fabricated replacements and presented them as originals**. If copies are later recovered from the Raspberry Pi, old Windows/WSL folders, backups, or the original Git working directory, they should be added as their real source files.

## Third-party code intentionally excluded

A large Luxonis DepthAI demo/framework file was also present in the archive. It was not included here because it is upstream/third-party code rather than project-authored drone autonomy code.


## Deep historical test restoration from project history pass

A later recovery pass searched the project conversation history for individual staged tests, not only archived file bytes. The `tests/` tree now contains the restored from project history phase progression, including arm-only, 1 m/2 m/3 m takeoff variants, CH6 recovery, yaw search/rate tuning, vision centering, Phase 6 distance experiments, Phase 7 forward/back and pulse-control iterations, the GUIDED_NOGPS branch, real OAK-D dry/motor tests, and the ArduPilot SITL scripts.

The repository deliberately distinguishes recovered source from restoration from project historys in `tests/README.md` and `tests/TEST_PROVENANCE.json`; restored from project history files should not be described as byte-for-byte originals.
