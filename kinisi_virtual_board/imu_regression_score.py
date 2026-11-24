"""External public IMU comparator. Pure data in/out; no firmware or release imports.

Truth is used only here, never in the estimator. References use world x/y and
continuous yaw, with body-frame vx/vy and angular rate in radians per second.
Missing observations cannot pass and do not erase independently witnessed errors.
"""
import math
import re
from collections import Counter

START_US = 600_000
END_US = 5_120_000
RESTART_US = 3_100_000
SOURCE_PERIOD_US = 10_000
POLL_US = 1_000
REFERENCE_PERIOD_US = 2_500
WINDOW_US = 1_000_000
FIRST_SOURCE_AGE_LIMIT_US = 35_000
PUBLIC_LATENCY_LIMIT_US = 50_000
EXPIRY_LATENCY_LIMIT_US = 50_000
FAULT_RESPONSE_LIMIT_US = 100_000
LIMITS = dict(position_m=0.050, body_speed_m_s=0.020,
              yaw_rad=math.radians(2.0), yaw_rate_rad_s=0.020)
PROFILES = ("stationary", "translate", "brake", "reverse", "turn", "stale")
VECTORS = ("x", "y", "yaw", "body_vx", "body_vy", "wz")
PUBLIC_FIELDS = ("public_us", "state", "fault", "flags", "sequence", "timestamp_us",
                 "elapsed_us", "accel_age_us", "gyro_age_us", *VECTORS,
                 "pose_sequence", "twist_sequence", "pose_error", "twist_error",
                 "motor_pwm_max", "consumed_accel_us", "consumed_gyro_us")
REFERENCE_FIELDS = ("at_us", "x", "y", "yaw", "vx", "vy", "wz")
SENSOR_FIELDS = ("delivered_us", "accel_us", "gyro_us", "ax", "ay", "az", "gx", "gy", "gz")
OPTIONAL = (*VECTORS, "pose_sequence", "twist_sequence", "consumed_accel_us", "consumed_gyro_us")
MAX_EXAMPLES = 5
INTEGERS = {key for key in PUBLIC_FIELDS + REFERENCE_FIELDS + SENSOR_FIELDS
            if key.endswith("_us") or key in ("state", "fault", "flags", "sequence", "pose_sequence",
                                               "twist_sequence", "pose_error", "twist_error")}


def coherent(row):
    return (row["state"] == 2 and row["fault"] == 0 and row["flags"] == 11
            and row["pose_error"] == row["twist_error"] == 0
            and row["sequence"] == row["pose_sequence"] == row["twist_sequence"]
            and all(row[key] is not None for key in VECTORS))


def _rows(raw, fields, clock, grid, issues):
    parsed = []
    for index, row in enumerate(raw or []):
        try:
            item = {}
            for key in fields:
                value = row[key]
                if fields == PUBLIC_FIELDS and key in OPTIONAL and value is None:
                    item[key] = None
                    continue
                if isinstance(value, bool):
                    raise ValueError("boolean number")
                if key in INTEGERS:
                    number = value
                    if type(number) is not int or not 0 <= number <= 0xffffffffffffffff:
                        raise ValueError("nonintegral/unsigned clock or code")
                else:
                    number = value
                    if type(number) not in (int, float) or not math.isfinite(number):
                        raise ValueError("nonfinite vector")
                item[key] = number
            parsed.append(item)
        except (KeyError, TypeError, ValueError, OverflowError):
            issues.append(f"Malformed {clock} row {index}")
    if [row[clock] for row in parsed] != grid:
        issues.append(f"Incomplete or unordered {clock} grid: expected {len(grid)} rows")
    groups = {}
    allowed = set(grid)
    for row in parsed:
        if row[clock] in allowed:
            groups.setdefault(row[clock], []).append(row)
    result = []
    for time, group in sorted(groups.items()):
        if any(row != group[0] for row in group[1:]):
            issues.append(f"Conflicting {clock} rows at {time}")
        else:
            result.append(group[0])
    return result


def score_case(capture):
    """Score one capture_case result; every expected timestamp remains accounted.

    Result outcome is PASS, FAIL or INCOMPLETE. FAIL takes precedence when a
    product violation coexists with missing evidence; `incomplete` is retained.
    Epoch `anchor_verified` gates position/relative-yaw measurements. Absolute
    body velocity and yaw-rate errors remain measurable without an anchor.
    """
    meta = capture.get("metadata", {})
    profile, delay = meta.get("profile"), meta.get("delay_us")
    issues, failures, epochs, peaks = [], [], [], {}
    if profile not in PROFILES or delay not in (10_000, 13_000, 17_000):
        return dict(outcome="INCOMPLETE", passed=False, case=meta.get("case"), profile=profile,
                    failures=[], incomplete=["Unknown profile or delivery phase"], epochs=[], peaks={},
                    failure_counts={}, incomplete_counts={"Unknown profile or delivery phase": 1},
                    counts=dict(polls=0, reference_rows=0, sensor_rows=0, epochs=0, samples=0, failures=0, incomplete=1), limits=dict(LIMITS))
    if meta.get("error"):
        issues.append(f"Capture error: {meta['error']}")
    restart = RESTART_US if profile in ("stationary", "stale") else None
    if meta.get("start_us") != START_US or meta.get("restart_us") != restart:
        issues.append("Capture action schedule differs from profile")
    if meta.get("end_us") != END_US or meta.get("poll_us") != POLL_US:
        issues.append("Capture duration/polling differs from profile")
    rows = _rows(capture.get("public_rows"), PUBLIC_FIELDS, "public_us", list(range(START_US, END_US+1, POLL_US)), issues)
    refs = _rows(capture.get("reference_rows"), REFERENCE_FIELDS, "at_us", list(range(START_US-20_000, END_US+1, REFERENCE_PERIOD_US)), issues)
    sensors = _rows(capture.get("sensor_rows"), SENSOR_FIELDS, "delivered_us",
                    [START_US+k*SOURCE_PERIOD_US+die+delay for k in range(451) for die in (0, 2500)], issues)
    refmap = {row["at_us"]: row for row in refs}
    available, payloads = ({}, {}), ({}, {})
    for row in sensors:
        index_time = row["delivered_us"]-START_US-delay
        k, phase = divmod(index_time, SOURCE_PERIOD_US)
        for channel, nominal, keys in ((0, k, ("ax", "ay", "az")), (1, k if phase else k-1, ("gx", "gy", "gz"))):
            index = 79 if profile == "stale" and 80 <= nominal <= 109 else nominal
            expected = START_US+index*SOURCE_PERIOD_US+(2500 if channel else 0)
            source = row["gyro_us" if channel else "accel_us"]
            values = tuple(row[key] for key in keys)
            if source != expected or source > row["delivered_us"]:
                issues.append(f"Unexpected sensor identity at {row['delivered_us']}")
            if source in payloads[channel] and payloads[channel][source] != values:
                issues.append(f"Changed held sensor payload at {row['delivered_us']}")
            payloads[channel][source] = values
            available[channel].setdefault(source, row["delivered_us"])

    def fail(kind, row=None, **detail):
        failures.append(dict(kind=kind, **({"public_us": row["public_us"]} if row is not None else {}), **detail))

    def measure(metric, value, row, start):
        if metric not in peaks or value > peaks[metric]["value"]:
            peaks[metric] = dict(value=value, limit=LIMITS[metric], source_us=row["timestamp_us"],
                                 public_us=row["public_us"], start_us=start)
        if value > LIMITS[metric]:
            fail("accuracy", row, metric=metric, value=value, limit=LIMITS[metric], source_us=row["timestamp_us"])

    for row in rows:
        state, p = row["state"], row["public_us"]
        flags = {0: 0, 1: 8, 2: 11, 3: 12, 4: 12}.get(state)
        if row["flags"] != flags or (state != 3 and row["fault"] != (7 if state == 4 else 0)):
            fail("state_flags_fault", row)
        if row["motor_pwm_max"] != 0:
            fail("diagnostic_motor_output", row)
        if state == 2:
            if not coherent(row): fail("incoherent_publication", row)
        elif any(row[key] is not None for key in (*VECTORS, "pose_sequence", "twist_sequence")) or row["pose_error"] != 10 or row["twist_error"] != 10:
            fail("unavailable_vectors", row)
        if row["elapsed_us"] > WINDOW_US or (state == 1 and row["elapsed_us"] != 0) or (state == 4 and row["elapsed_us"] != WINDOW_US):
            fail("elapsed_time", row)
        for channel in ("accel", "gyro"):
            age = row[channel+"_age_us"]
            inferred = None if age == 0xffffffff else p-age
            if row["consumed_"+channel+"_us"] != inferred or (inferred is not None and inferred < 0):
                fail("consumed_age_identity", row, channel=channel)

    old_sequences = set()
    starts = [START_US] + ([restart] if restart is not None else [])
    for epoch_index, start in enumerate(starts):
        end_segment = starts[epoch_index+1] if epoch_index+1 < len(starts) else END_US+POLL_US
        segment = [r for r in rows if start <= r["public_us"] < end_segment]
        valid = [r for r in segment if coherent(r)]
        times = {r["public_us"] for r in segment}
        horizon = max(times, default=start-POLL_US)
        prefix = profile == "stale" and epoch_index == 0
        confirmed_key = "start_command_us" if epoch_index == 0 else "restart_command_us"
        confirmed = meta.get(confirmed_key) == start
        epoch = dict(start_us=start, complete=not prefix, anchor_us=None, anchor_verified=False,
                     expected_samples=0, observed_samples=0, measured_samples=0,
                     missing_source_count=0, missing_source_examples=[])
        epochs.append(epoch)
        if not confirmed:
            issues.append(f"Unconfirmed action at {start}")
            continue
        lo, hi = start+482_500, start+592_500
        first_possible = lo+7500+delay

        def covered(a, b):
            return all(t in times for t in range(((a+999)//1000)*1000, (b//1000)*1000+1, POLL_US))

        first = valid[0] if valid else None
        if first and covered(first_possible, first["public_us"]):
            epoch.update(anchor_us=first["timestamp_us"], anchor_verified=True)
            if not lo <= first["timestamp_us"] <= hi or (first["timestamp_us"]-START_US-2500) % SOURCE_PERIOD_US:
                fail("startup_anchor", first)
        elif first:
            issues.append(f"Unverified first source anchor at action {start}")
        if not first or first["public_us"] > hi+PUBLIC_LATENCY_LIMIT_US:
            if horizon >= hi+PUBLIC_LATENCY_LIMIT_US and covered(first_possible, hi+PUBLIC_LATENCY_LIMIT_US):
                fail("startup_missing", start_us=start)
            else: issues.append(f"Startup evidence incomplete at {start}")
        anchor = epoch["anchor_us"]
        anomaly = START_US+80*SOURCE_PERIOD_US+delay
        last_prefix = START_US+78*SOURCE_PERIOD_US+2500
        expected = []
        if anchor is not None:
            last = last_prefix if prefix else anchor+WINDOW_US-SOURCE_PERIOD_US
            expected = list(range(anchor, last+1, SOURCE_PERIOD_US))
            epoch["expected_samples"] = len(expected)
            calibration = [r for r in refs if start <= r["at_us"] <= anchor]
            if any(math.hypot(r["vx"], r["vy"]) > .003 or abs(r["wz"]) > .01 for r in calibration):
                issues.append(f"Reference is not stationary during calibration at {start}")
        seen, sequences, terminal = {}, {}, None
        previous, previous_source, previous_sequence = None, None, None
        for row in segment:
            p, source, sequence = row["public_us"], row["timestamp_us"], row["sequence"]
            if p == start and row["state"] != 1: fail("action_not_calibrating", row)
            if row["state"] == 0: fail("stopped_after_start", row)
            if row["state"] == 3 and not prefix: fail("unexpected_fault", row)
            if previous_sequence is not None and sequence < previous_sequence: fail("sequence_regression", row)
            previous_sequence = sequence
            if terminal and (row["state"], row["fault"]) != terminal: fail("terminal_not_latched", row)
            if row["state"] in (3, 4): terminal = (row["state"], row["fault"])
            if not coherent(row):
                if first and first["public_us"] <= p < (anomaly if prefix else (anchor or lo)+WINDOW_US): fail("premature_invalidity", row)
                previous = row
                continue
            if sequence in old_sequences: fail("restart_reused_sequence", row)
            if (source-START_US-2500) % SOURCE_PERIOD_US or source < lo or source > (last_prefix if prefix else hi+WINDOW_US-SOURCE_PERIOD_US):
                fail("source_outside_window", row)
            if anchor is not None:
                if source not in expected: fail("source_outside_window", row)
                if row["elapsed_us"] != min(max(p-anchor, 0), WINDOW_US): fail("elapsed_time", row)
            if previous_source is not None and source < previous_source: fail("source_regression", row)
            previous_source = source
            if sequence in sequences and sequences[sequence] != source: fail("sequence_reused", row)
            sequences[sequence] = source
            if row["consumed_gyro_us"] != source or row["consumed_accel_us"] != source+7500:
                fail("source_pair_identity", row)
            if source <= start or source+7500 <= start: fail("pre_action_source", row)
            if source in seen:
                if sequence != seen[source]["sequence"]: fail("duplicate_source", row)
                if any(row[key] != seen[source][key] for key in (*VECTORS, "consumed_accel_us", "consumed_gyro_us")):
                    fail("changed_cached_snapshot", row)
            else:
                seen[source] = row
                proven = previous is not None and previous["public_us"] == p-POLL_US
                if proven and (p-source > PUBLIC_LATENCY_LIMIT_US or p < source): fail("public_latency", row)
                if proven and max(p-source, p-(source+7500), p-(source-2500)) > FIRST_SOURCE_AGE_LIMIT_US:
                    fail("first_source_age", row)
                for channel, stamp in ((0, source-2500), (0, source+7500), (1, source)):
                    if stamp not in available[channel]:
                        issues.append(f"Missing delivered source evidence {stamp}")
                    elif available[channel][stamp] > p:
                        fail("source_not_delivered", row, source_us=stamp)
                ref = refmap.get(source)
                if ref is None:
                    issues.append(f"Missing exact reference at source {source}")
                else:
                    errors = dict(body_speed_m_s=math.hypot(row["body_vx"]-ref["vx"], row["body_vy"]-ref["vy"]),
                                  yaw_rate_rad_s=abs(row["wz"]-ref["wz"]))
                    if anchor is not None and anchor in refmap:
                        origin = refmap[anchor]
                        c, s = math.cos(origin["yaw"]), math.sin(origin["yaw"])
                        dx, dy = ref["x"]-origin["x"], ref["y"]-origin["y"]
                        errors.update(position_m=math.hypot(row["x"]-(c*dx+s*dy), row["y"]-(-s*dx+c*dy)),
                                      yaw_rad=abs(row["yaw"]-(ref["yaw"]-origin["yaw"])))
                    elif anchor is not None:
                        issues.append(f"Missing exact anchor reference {anchor}")
                    for metric, value in errors.items(): measure(metric, value, row, start)
                    epoch["measured_samples"] += 1
            previous = row
        old_sequences.update(sequences)
        epoch["observed_samples"] = len(seen)
        if anchor is None:
            eligible = [s for s in seen if lo <= s <= (last_prefix if prefix else hi+WINDOW_US-SOURCE_PERIOD_US)
                        and not (s-START_US-2500) % SOURCE_PERIOD_US]
            if eligible:
                # A missing initial anchor does not erase later fully witnessed
                # source gaps between known epochs (or in the fixed fault prefix).
                expected = list(range(min(eligible), (last_prefix if prefix else max(eligible))+1, SOURCE_PERIOD_US))
        first_fault = next((r for r in segment if r["state"] == 3), None)
        if first_fault:
            prior_valid = [r for r in valid if r["public_us"] < first_fault["public_us"]]
            lower = prior_valid[-1]["elapsed_us"] if prior_valid else 0
            if anchor is not None and not lower <= first_fault["elapsed_us"] <= min(max(first_fault["public_us"]-anchor, 0), WINDOW_US):
                fail("fault_elapsed_entry", first_fault)
            for row in segment:
                if row["state"] == 3 and row["public_us"] >= first_fault["public_us"] and row["elapsed_us"] != first_fault["elapsed_us"]:
                    fail("fault_elapsed_latch", row)
        for source in expected:
            observed = seen.get(source)
            if observed is None:
                epoch["missing_source_count"] += 1
                if len(epoch["missing_source_examples"]) < MAX_EXAMPLES:
                    epoch["missing_source_examples"].append(source)
            if observed is None or observed["public_us"] > source+PUBLIC_LATENCY_LIMIT_US:
                deadline = source+PUBLIC_LATENCY_LIMIT_US
                if horizon >= deadline and covered(source+7500+delay, deadline):
                    fail("missing_source_slot", source_us=source, start_us=start)
                else: issues.append(f"Source slot unverified {source}")
        if prefix:
            terminal_row = next((r for r in segment if r["state"] == 3), None)
            deadline = anomaly+FAULT_RESPONSE_LIMIT_US
            wrong_after = any(r["public_us"] >= deadline and (r["state"], r["fault"]) != (3, 3) for r in segment)
            wrong_seen = terminal_row and (terminal_row["public_us"] < anomaly or terminal_row["fault"] != 3)
            late = terminal_row is None or terminal_row["public_us"] > deadline
            if wrong_seen or wrong_after or (late and horizon >= deadline and covered(anomaly, deadline)):
                fail("fault_deadline_or_code", start_us=start)
            elif late: issues.append("Fault deadline not observed")
        else:
            window_end = (anchor or lo)+WINDOW_US
            deadline = (anchor or hi)+WINDOW_US+EXPIRY_LATENCY_LIMIT_US
            terminal_row = next((r for r in segment if r["state"] == 4), None)
            wrong_after = any(r["public_us"] >= deadline and (r["state"], r["fault"]) != (4, 7) for r in segment)
            early = terminal_row is not None and terminal_row["public_us"] < window_end
            late = terminal_row is None or terminal_row["public_us"] > deadline
            if early or wrong_after or (late and horizon >= deadline and covered(window_end, deadline)):
                fail("expiry_deadline", start_us=start)
            elif late: issues.append(f"Expiry deadline not observed at {start}")
            if horizon < deadline+100_000: issues.append(f"Expiry latch tail incomplete at {start}")
    issues = list(dict.fromkeys(issues))
    outcome = "FAIL" if failures else "INCOMPLETE" if issues else "PASS"
    failure_counts = dict(Counter(f["kind"] + ("."+f["metric"] if "metric" in f else "") for f in failures))
    incomplete_counts = dict(Counter(re.sub(r"\d+", "<n>", issue) for issue in issues))
    return dict(outcome=outcome, passed=outcome == "PASS", case=meta.get("case", profile), profile=profile,
                seed=meta.get("seed"), delay_us=delay, limits=dict(LIMITS),
                failures=failures[:MAX_EXAMPLES], incomplete=issues[:MAX_EXAMPLES],
                failure_counts=failure_counts, incomplete_counts=incomplete_counts,
                peaks=peaks, epochs=epochs,
                counts=dict(polls=len(rows), reference_rows=len(refs), sensor_rows=len(sensors), epochs=len(epochs),
                            samples=sum(e["measured_samples"] for e in epochs),
                            failures=len(failures), incomplete=len(issues)))
