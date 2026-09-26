# Historical Test Suite

This directory reconstructs the staged Python tests used while developing the autonomous follow-me drone. It is intentionally broader than the final production-style companion script: the purpose is to preserve the engineering progression, failed experiments, bench tests, SITL work, and safety iterations that led to the later integrated controller.

> **Important:** these are historical prototype tests, not a drop-in flight package for another aircraft. Many scripts can arm, command movement, or change flight modes. Check the connection string, airframe configuration, ArduPilot version, sensors, RC recovery switch, and test environment before running anything.

## Fidelity labels

The source archive did not retain every original `.py` byte-for-byte. Files therefore contain a provenance header:

- **Recovered source** — original source body was available and is copied directly.
- **Restored from project history test artifact** — the original conversation retained the filename, constants, control flow, outputs, and/or complete code description, but not the raw file. Boilerplate was rebuilt to create a standalone `.py` file.
- **Isolated reconstruction** — a behavior that originally lived inside a larger script was split into its own runnable test file because the project explicitly tested/tuned that behavior separately.
- **Proposed safety test** — a next-stage safety file was designed in the project but was not confirmed as a flown test. It is retained because it documents the intended safety contract.

## Test progression

| Stage | File | Purpose |
|---|---|---|
| Bench | `real_flight/initial_coms.py` | OAK-D person detection → FC arm/verify → 5 s → disarm |
| Bench | `real_flight/rc6_trigger_test.py` | Validate RC6 three-position values and DOWN trigger |
| Bench | `real_flight/real_rc6_arm_disarm_test.py` | RC6-gated real-FC arm/disarm |
| Diagnostics | `diagnostics/vision_arm_debug.py` | Vision-triggered GUIDED arm with STATUSTEXT/ACK diagnostics |
| Diagnostics | `diagnostics/vision_stabilize_arm_test.py` | Compare STABILIZE arming behavior |
| Diagnostics | `diagnostics/gps_ekf_status_test.py` | Inspect GPS fix, EKF flags/variance, FC status text |
| Safety | `diagnostics/rc_override_safety_test.py` | CH6: 999 autonomy, 1503 LAND request, 2000 manual recovery |
| Phase 0 | `real_flight/phase0_arm_only.py` | Simplest props-off arm/hold/disarm test |
| Phase 1 | `real_flight/phase1_guided_takeoff_land.py` | Original 3 m GUIDED takeoff/hover/LAND |
| Phase 1A | `real_flight/phase1a_takeoff_1m_land.py` | 1 m takeoff with altitude confirmation and LAND |
| Phase 1A | `real_flight/phase1_altitude_hover_land.py` | 1 m altitude-confirmed hover; CH6 LAND used operationally |
| Phase 1B | `real_flight/phase1b_guided_takeoff_2m_land.py` | Standalone 2 m variant restored from project history from the 2 m test discussion |
| Phase 2 | `real_flight/phase2_land_switch_override.py` | CH6 middle-position LAND override while airborne |
| Phase 3 | `real_flight/arm_takeoff_yaw_test.py` | Isolated arm → 1 m takeoff → yaw → LAND sequence |
| Phase 3 | `real_flight/phase3_yaw_search_fixed.py` | OAK-D search yaw; fixed degree/radian bug; 3 → 12 deg/s |
| Tuning | `real_flight/yaw_rate_sweep_test.py` | Isolated yaw-speed comparison |
| Phase 4 | `real_flight/phase4_yaw_center_simple.py` | Simple yaw-until-person-centered behavior |
| Phase 5 | `real_flight/phase5_altitude_guard_yaw_center.py` | Add altitude correction and tighter vision centering |
| Phase 5 later | `real_flight/phase5_keep_center_yaw_optflow.py` | Recovered later keep-centered source with LiDAR/OF-assisted setup |
| Vision | `vision/vision_decision_test.py` | Vision decisions only: left/right/center and far/close/good |
| Vision | `vision/oak_person_test.py` | OAK-D person/depth validation restored from project history from preserved output |
| Vision | `vision/oak_person_stable_test.py` | Median-smoothed person offset + stereo depth |
| Phase 6 | `vision/phase6_bbox_distance_only.py` | Collect bbox size as distance proxy |
| Phase 6 | `vision/phase6_distance_data_collection.py` | Attempt concurrent YOLO + stereo depth; exposed SHAVE limit |
| Phase 6 | `vision/phase6_distance_split_test.py` | Split bbox and stereo-depth tests to avoid resource conflict |
| Phase 7A | `motion/phase7a_depth_forward_back.py` | Continuous depth-proportional forward/back movement |
| Phase 7A safe | `motion/phase7a_safe_decision_only.py` | Compute movement decision but transmit no horizontal movement |
| Phase 7B | `motion/phase7b_one_pulse_forward_back.py` | Exactly one tiny movement pulse |
| Phase 7C | `motion/phase7c_repeated_pulse_forward_back.py` | Repeated tiny pulses with altitude rechecks |
| Phase 7 final | `motion/phase7_depth_pulse_follow.py` | Tuned depth-only pulse follow with attitude/battery/altitude gates |
| Dry run | `real_flight/real_oak_rc6_dry_run.py` | RC6 + OAK-D decisions, explicitly no arming/motor movement |
| Motor response | `real_flight/real_oak_motor_response_test.py` | Props-off OAK-D → RC override motor-response test |
| Safety design | `real_flight/real_oak_follow_safety_test.py` | RC6 kill/recovery, person-loss stop, timeout disarm contract |
| Debug | `diagnostics/arm_failure_debug.py` | Diagnose `Arm: Need Position Estimate` |
| GPS-denied branch | `real_flight/testtest_guided_nogps.py` | GUIDED_NOGPS velocity-climb experiment with LiDAR and yaw centering |
| SITL | `sitl/sitl_takeoff_test.py` | Minimal 3 m Software-In-The-Loop takeoff |
| SITL | `sitl/sitl_follow_test.py` | Early fake-person scan/face/follow test |
| SITL | `sitl/sitl_follow_sim.py` | Smarter fake tracking sequence with yaw + forward/back control |
| SITL | `sitl/sitl_yaw_rate_test.py` | Isolated yaw-rate test in SITL |
| SITL | `sitl/sitl_forward_backward_test.py` | Isolated BODY_NED forward/back test in SITL |
| SITL + OAK-D | `sitl/oak_sitl_follow.py` | Real OAK-D perception driving simulated ArduCopter |
| Logic simulation | `sitl/simulated_autonomy_loop.py` | Vision decisions + FC state without commanding the aircraft |

## Key historical values

The test history records the progression particularly well:

- RC6: **DOWN ≈ 999**, **MIDDLE ≈ 1503**, **UP ≈ 2000**.
- Initial follow-distance work: **3.0 m ± 0.4 m**.
- Later depth-follow target: **5.5 m**, initially ±0.4/0.45 m, finally **±0.25 m**.
- Yaw search: initially **3 deg/s**, fixed to use radians correctly, then increased to **12 deg/s**.
- Later centering: target x **0.50**, tolerance **0.03**, proportional yaw gain **30**, max **10 deg/s**, min **2.5 deg/s**.
- Phase-7 movement evolved from continuous ±0.25 m/s control to 0.05 m/s / 0.25 s micro-pulses, then a tuned pulse controller capped at **0.38 m/s**.
- Phase-7 final safety included altitude, roll/pitch, angular-rate, battery/failsafe and pulse-abort checks.

## Software versions

The project crossed multiple DepthAI and ArduPilot generations. Early OAK-D files use the older `ColorCamera` / `MonoCamera` / `MobileNetDetectionNetwork` style. Later files use the newer DepthAI builder API and `yolov6-nano`. This is historically accurate, but it means one Python environment may not run every archived test unchanged.
