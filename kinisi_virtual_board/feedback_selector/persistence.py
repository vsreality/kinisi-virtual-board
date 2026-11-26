"""Public-wire migration, cold-start safety and unsupported-board demonstrations."""
import hashlib
import math

FACTORY_SHA = "7d2c7ac4888bfd75cd5f56e8d61f69595121183afc81556c876732fd3782c62f"
PRESET_SHA = "d492e8f032c427407fb232d459b65d0fa23d13ffebef2c7026764bc4de66231e"
PRESET = [3., .12, .48, .31, 4096., 2000., 2000., 2000.,
          1., 1., 1., 0., 1., 0., 1., 0., 25000]


def _sha(image):
    return hashlib.sha256(image).hexdigest()


def _snapshot(s):
    fields = [s.ask("GET_CONFIGURATION_FIELD", field=i)["value"] for i in range(4)]
    fields += [s.ask("GET_CONFIGURATION_FIELD", field=field, index=i)["value"]
               for field in (4, 5, 6) for i in range(4)]
    return fields + [s.ask("GET_PWM_FREQUENCY")["frequency_hz"]]


def _unavailable(s, requested, *, reason=None):
    status = s.status()
    assert status["requested"] == requested and status["active"] == 255 and status["valid"] == 0, status
    assert status["sequence"] == status["timestamp_us"] == 0 and status["age_us"] == 0xffffffff, status
    if reason is not None:
        assert status["reason"] == reason, status
    for kind in (0, 1):
        s.error("GET_FEEDBACK_SOURCE_VECTOR", "SAMPLE_NOT_AVAILABLE", kind=kind,
                origin_id=status["origin_id"], sequence=status["sequence"])
    assert not any(s.board.motor_pwm)
    return status


def _boot_safety(s, requested):
    assert s.board.now_us == 0 and not any(s.board.motor_pwm)
    assert s.board.motor_velocity == [0.] * 4
    assert s.board.fram_transactions[1] == 0
    assert _snapshot(s) == PRESET
    status = _unavailable(s, requested, reason=1)
    assert s.ask("GET_IMU_TRACKING_STATUS")["state"] == 0
    mount = s.ask("GET_FEEDBACK_FUSION_MOUNT")
    assert mount == dict(confirmed=0, mounting_yaw_rad=0.)
    s.error("SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", x=.1)
    s.error("GET_PLATFORM_ODOMETRY", "ODOMETRY_NOT_INITIALIZED")
    s.mark("cold_boot_safe", status=status, configuration=PRESET, mount=mount,
           fram_transactions=s.board.fram_transactions)
    return status


def _quiet_encoder_samples(s):
    # Selection requires measured rest even when the configured controller has
    # never been armed. Start public odometry, then observe a genuine quiet span.
    s.ask("SET_ODOMETRY_FREQUENCY", frequency=100)
    s.ask("START_PLATFORM_ODOMETRY")
    s.board.advance(.15)
    s.ask("GET_PLATFORM_ODOMETRY")
    assert not any(s.board.motor_pwm)
    s.mark("measured_rest_for_selection", duration_us=150000)


def _fresh_imu(s):
    before = s.status()
    s.ask("SET_IMU_TRACKING", action=0, stationary_mount_confirmed=1, mounting_yaw_rad=math.pi)
    calibrating = _unavailable(s, 1, reason=2)
    assert calibrating["origin_id"] != before["origin_id"]
    start = s.board.now_us
    for _ in range(400):
        s.board.advance(.002)
        status = s.status()
        if status["valid"]:
            break
    else:
        raise AssertionError("IMU tracking did not calibrate within 800 ms")
    pose = s.ask("GET_FEEDBACK_SOURCE_VECTOR", kind=0,
                 origin_id=status["origin_id"], sequence=status["sequence"])
    twist = s.ask("GET_FEEDBACK_SOURCE_VECTOR", kind=1,
                  origin_id=status["origin_id"], sequence=status["sequence"])
    assert status["requested"] == status["active"] == 1 and status["reason"] == 0
    assert pose["sequence"] == twist["sequence"] == status["sequence"]
    assert max(abs(pose[k]) for k in ("x", "y", "z")) < 1e-12, pose
    assert status["timestamp_us"] > start and status["age_us"] == s.board.now_us-status["timestamp_us"]
    assert not any(s.board.motor_pwm)
    s.mark("fresh_imu_origin", status=status, pose=pose, twist=twist, start_us=start)
    return status


def factory(s):
    """The erased legacy image boots with encoder preference and no producers."""
    original = s.board.fram_image
    assert _sha(original) == FACTORY_SHA
    assert s.ask("GET_CONFIGURATION_FIELD", field=0)["value"] == 0
    assert s.ask("GET_PWM_FREQUENCY")["frequency_hz"] == 100000
    s.error("GET_CONFIGURATION_FIELD", "PLATFORM_NOT_INITIALIZED", field=1)
    s.error("SET_PLATFORM_TARGET_VELOCITY", "PLATFORM_NOT_INITIALIZED", x=.1)
    s.error("SAVE_CONFIGURATION", "PLATFORM_NOT_INITIALIZED")
    s.error("GET_PLATFORM_ODOMETRY", "ODOMETRY_NOT_INITIALIZED")
    status = _unavailable(s, 0, reason=1)
    assert s.ask("GET_FEEDBACK_FUSION_MOUNT") == dict(confirmed=0, mounting_yaw_rad=0.)
    s.board.advance(.1)
    assert not any(s.board.motor_pwm)
    assert s.board.fram_image == original and s.board.fram_transactions[1] == 0
    s.mark("legacy_factory_unchanged", fixture_sha256=FACTORY_SHA, status=status,
           fram_transactions=s.board.fram_transactions)


def persistence(s):
    """Migrate a retained v1 preset, then prove volatile state and journal safety."""
    original = s.board.fram_image
    assert _sha(original) == PRESET_SHA
    old = _boot_safety(s, 0)
    assert s.board.fram_image == original
    s.mark("legacy_preset_preserved", fixture_sha256=PRESET_SHA, status=old, configuration=_snapshot(s))
    _quiet_encoder_samples(s)
    s.ask("SET_FEEDBACK_FUSION_MOUNT", mounting_yaw_rad=.37)
    assert s.ask("GET_FEEDBACK_FUSION_MOUNT") == dict(confirmed=1, mounting_yaw_rad=.37)
    s.ask("SET_FEEDBACK_SOURCE", source=1)
    _unavailable(s, 1, reason=1)
    s.ask("SAVE_CONFIGURATION")
    saved = s.board.fram_image
    assert saved != original and saved[:256] == original[:256]
    assert _snapshot(s) == PRESET
    s.mark("nondefault_preference_saved", requested=1, image_sha256=_sha(saved), configuration=PRESET)

    _fresh_imu(s)
    # Exact native-axis input is a simulated sensor stimulus, not an estimator
    # write. Build a visibly nonzero old coordinate frame before the cold boot.
    for index in range(20):
        # First leave the automatic model's exact acquisition boundary before
        # taking sensor ownership; never replace its payload under the same ID.
        s.board.advance(.001 if index == 0 else .01)
        at = s.board.now_us
        s.board.set_imu((-.3, 0., 9.80665), (0., 0., .15), accel_us=at, gyro_us=at)
        s.mark("inertial_stimulus", accel_us=at, gyro_us=at,
               acceleration=[-.3, 0., 9.80665], angular_velocity=[0., 0., .15])
        status = s.status()
        assert status["valid"] and status["active"] == 1, status
    old_pose = s.ask("GET_FEEDBACK_SOURCE_VECTOR", kind=0,
                     origin_id=status["origin_id"], sequence=status["sequence"])
    assert max(abs(old_pose[k]) for k in ("x", "y", "z")) > .001, old_pose
    assert not any(s.board.motor_pwm)
    s.mark("old_runtime_nonzero_origin", status=status, pose=old_pose)

    s.reboot(saved)
    boot = _boot_safety(s, 1)
    assert s.board.fram_image == saved
    fresh = _fresh_imu(s)
    s.mark("preference_only_survives_power_cycle", requested=1, boot_status=boot,
           fresh_status=fresh, prior_pose=old_pose,
           origin_scope="origin_id is board-local; reboot starts a new coordinate lifetime")

    # Retained journal layout: invalidation byte +212-byte record +commit byte.
    # Exercise first, intermediate and final precommit cuts on fresh runtimes.
    for budget in (0, 1, 100, 212, 213):
        s.reboot(saved)
        _boot_safety(s, 1)
        _quiet_encoder_samples(s)
        s.ask("SET_FEEDBACK_SOURCE", source=2)
        s.board.set_fram_fault(write_after_bytes=budget)
        s.error("SAVE_CONFIGURATION", "INTERNAL_ERROR")
        torn = s.board.fram_image
        assert torn[256:512] == saved[256:512]  # Prior committed slot remains intact.
        s.mark("torn_save_rejected", write_budget=budget, image_sha256=_sha(torn))
        s.reboot(torn)
        _boot_safety(s, 1)
        assert _snapshot(s) == PRESET
        s.mark("torn_save_restored_prior_preference", write_budget=budget, requested=1)

    s.reboot(saved)
    _quiet_encoder_samples(s)
    s.ask("SET_FEEDBACK_SOURCE", source=2)
    s.board.set_fram_fault(write_after_bytes=214)
    s.ask("SAVE_CONFIGURATION")
    complete = s.board.fram_image
    s.reboot(complete)
    # Passive fusion requires a new volatile mount declaration this boot.
    assert _snapshot(s) == PRESET
    status = _unavailable(s, 2, reason=9)
    assert s.ask("GET_FEEDBACK_FUSION_MOUNT") == dict(confirmed=0, mounting_yaw_rad=0.)
    s.error("SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", x=.1)
    assert s.board.fram_transactions[1] == 0
    s.mark("complete_journal_commit_restored", write_budget=214, status=status, image_sha256=_sha(complete))


def unsupported(s):
    """v0.3 and Nano reject selector/storage without touching the supplied image."""
    original = s.board.fram_image
    assert _sha(original) == PRESET_SHA
    calls = [("GET_FEEDBACK_SOURCE_STATUS", {}),
             ("GET_FEEDBACK_SOURCE_VECTOR", {"kind": 0, "origin_id": 0, "sequence": 0}),
             ("GET_FEEDBACK_SOURCE_VECTOR", {"kind": 1, "origin_id": 0, "sequence": 0}),
             ("SET_FEEDBACK_SOURCE", {"source": 0}),
             ("SET_FEEDBACK_SOURCE", {"source": 1}),
             ("SET_FEEDBACK_SOURCE", {"source": 2}),
             ("SET_FEEDBACK_FUSION_MOUNT", {"mounting_yaw_rad": 0.}),
             ("GET_FEEDBACK_FUSION_MOUNT", {}),
             ("GET_CONFIGURATION_FIELD", {"field": 0}),
             ("SAVE_CONFIGURATION", {}),
             ("GET_FUSION_CONFIGURATION", {}),
             ("GET_IMU_TRACKING_STATUS", {})]
    for command, values in calls:
        s.error(command, "SENSOR_NOT_AVAILABLE", **values)
    s.board.advance(.1)
    assert s.board.fram_transactions == (0, 0)
    assert s.board.fram_image == original and not any(s.board.motor_pwm)
    s.mark("unsupported_without_storage_or_motion", fixture_sha256=PRESET_SHA,
           rejected_commands=len(calls), fram_transactions=s.board.fram_transactions)
