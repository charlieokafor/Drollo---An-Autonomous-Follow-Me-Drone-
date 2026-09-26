# ArduPilot Parameter Snapshots

These files are **historical configuration snapshots from the prototype**, not universal settings for another aircraft.

- `gps_baseline_full_params.param` — a GPS-supported baseline snapshot from the SpeedyBee F405 V3 stage.
- `arducopter_4.4.4_optical_flow.param` — the ArduCopter 4.4.4 optical-flow experiment. It contains the MTF01 optical-flow configuration and EKF source experiments.
- `optical_flow_calibration_experiment.param` — a later optical-flow/rangefinder calibration-stage snapshot, including the RC auxiliary calibration function and Raspberry Pi MAVLink serial link used during that branch.

Do **not** flash these files blindly onto another aircraft. They include aircraft-specific calibration, orientation, battery, compass, motor, RC, serial, EKF and failsafe values.
