# Encoder-only platform baseline

The evaluator runs the v0.4 production firmware against signed, quantized wheel counters. Velocity courses use `SET_PLATFORM_TARGET_VELOCITY` and the release wheel PIDs. Position courses use `SET_PLATFORM_POSITION` and the release pose and wheel controllers. The differential, omni and mecanum presets share a 0.1 m wheel diameter. The frozen case file gives their other dimensions, gains, targets, contact windows and seeds. Differential velocity courses omit direct lateral translation. Differential position courses retain lateral world targets, reachable by turning.

## Collect and score

From the simulator root, point the collector at the release firmware Git checkout. It requires Python and the native firmware toolchain used by the simulator. Supply a new output directory for each collection.

```sh
python tools/encoder_baseline.py --firmware ../kinisi-motor-controller-firmware --out build/encoder-run
python tools/score_encoder_baseline.py build/encoder-run
```

The default matrix contains three presets, two courses, three surfaces and five paired seeds. It repeats one complete run in a fresh firmware instance. The reserved courses, seeds and slip pattern in `held_out` are excluded from collection. Preserve the source revisions, case file and build ID with any comparison. The collector records simulator and firmware source diffs and their hashes. Each case records firmware readback of the platform geometry, encoder scale and direction before motion. Use clean, pinned source checkouts for release evidence. [Case definitions](../tools/encoder_baseline_cases.json)

## Optional stationary warm-up for development clients

Call `run_case(directory, firmware, spec, preset, mode, surface, seed, setup=configure_client, stationary_warmup=True)` from `tools/encoder_baseline.py`. The collector calls `configure_client(ask)` once after its usual board and controller configuration at time zero. Use `ask(command, **fields)` to issue synchronous firmware commands and check their replies. The client chooses its commands. For the original release comparison arm, omit `setup` and enable the same stationary warm-up.

The course starts at 600 ms. The BMI088 model runs from boot in both modes. With stationary warm-up enabled, the board also receives the independent accel and gyro events before the course starts. Each die has a 10 ms digital delivery delay. Acquisition time zero is the HAL sentinel for an unseen die.

## What the plant measures

Each seed fixes motor free speed (9.4–10.6 rad/s), motor response time (45–55 ms) and body translation response time (75–105 ms, yaw 0.8 times that). The four wheels share motor parameters within a run. Geometry and encoder resolution of 4096 counts/rev are treated as known. Wheel shaft motion drives the firmware's wrapping 16-bit counters. Independently, wheel contact transmission scales the unquantized shaft travel delivered to the chassis. Its wheel mounting constraints determine a body-twist demand, and its twist state follows that demand with finite response time. Body translation and rotation are integrated in a relative world frame from a stationary origin. The contact model leaves shaft load unchanged, permits common and asymmetric slip, and has no load transfer, collision, tire-force identification or measured friction calibration.

The level board sits at the chassis reference point. The 2.5 ms integration grid resolves both BMI088 sampling phases. Each die filters and quantizes its own input and has an independent delivery queue. The sensor records carry separate accel_us and gyro_us timestamps alongside at_us for the board read. The reference file includes 2.5 ms truth rows before the first read for exact source-time lookup. Earlier recordings with imu_acquired_us retain their original timing contract. [BMI088 model, conditions and limitations](bmi088.md)

## Evidence boundaries

Each case contains independent files. For a future estimator replay, supply only `sensors.csv`, the public geometry and the command schedule it needs. The record contains counter readings at observation time and the delayed six-axis IMU readings with acquisition and delivery times. `firmware.csv` contains encoder odometry, sample time, clock status, PWM and controller dispatch counts. `reference.csv` holds simulated chassis pose and twist for offline scoring. `case.json` includes hidden plant parameters and injected biases for auditing. Keep those values and the reference file outside estimators and controllers. Closed-loop comparisons must rerun the plant for each controller because their actions can change subsequent measurements.

The scorer computes errors at unique odometry acquisition times. It compares estimated relative world position and wrapped heading against the chassis reference. The release has no implemented platform current-velocity response. Estimated interval-average body twist is therefore derived from successive timestamped firmware poses, rotating translation into the midpoint body frame. It is compared with the corresponding interval-average reference. End drift is also normalized by actual ground distance. Tracking uses actual chassis pose and twist. In velocity mode the desired path integrates the commanded body twist. In position mode the piecewise world goal defines the pose tracking error. No desired instantaneous velocity is inferred from a position step.

Response latency is simulated time from the first nonzero target to the first observed nonzero PWM, reference motion or published nonzero pose, on a 10 ms observation grid. These are onset times, distinct from settling time, packet round-trip time or processor cost. The scorer reports odometry sample age and observed cadence, controller task dispatches per second, and descriptive mean, sample SD and range across seeds. Ordinary unreached goals retain their tracking error. Structurally invalid recordings are listed separately. Deterministic repeats and matched encoder-only firmware traces across surfaces are checked explicitly. Physical accuracy, power, latency and numerical fusion acceptance remain open.
