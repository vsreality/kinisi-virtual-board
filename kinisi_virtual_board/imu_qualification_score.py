"""Qualification comparator and measured chassis coverage; no acquisition or I/O.

The public 144-case matrix uses actual chassis reference and executed targets.
Coverage failures make qualification incomplete; independent measured diagnostic
violations remain failures even when coverage or input evidence is incomplete.
"""
import hashlib
import json
import math
from collections import Counter

from . import imu_regression_score as base

PRESETS = ("differential", "omni", "mecanum")
ROLES = ("stationary", "accelerate", "brake", "reverse", "turn")
FAULTS = base.FAULT_PROFILES
PHASES = (10_000, 13_000, 17_000)


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def coverage(reference, targets, role, direction, anchor_us, *, samples=100):
    """Exact public event predicates at A+j*10000; no nearest-time lookup."""
    lookup = {r["at_us"]: r for r in reference}
    times = [anchor_us+j*10_000 for j in range(samples)]
    if samples <= 0 or len(lookup) != len(reference) or any(t not in lookup for t in times):
        return dict(passed=False, reason="Missing exact coverage reference", anchor_us=anchor_us, samples=samples)
    rows = [lookup[t] for t in times]
    speed = lambda r: math.hypot(r["vx"], r["vy"])
    path = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"]) for a, b in zip(rows, rows[1:]))
    yaw = max(r["yaw"] for r in rows)-min(r["yaw"] for r in rows)
    result = dict(anchor_us=anchor_us, samples=samples, path_m=path, yaw_excursion_rad=yaw, observed_blocks=[])
    if role in ("stationary", *FAULTS):
        result["passed"] = all(speed(r) <= .003 and abs(r["wz"]) <= .01 for r in rows) and path <= .001 and yaw <= .005
        return result
    if (samples != 100 or type(direction) not in (list, tuple) or len(direction) != 2
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in direction)
            or abs(math.hypot(*direction)-1) > 1e-9):
        return dict(result, passed=False, reason="100 samples and declared unit body direction required")

    def blocks(predicate):
        return [(i, i+4) for i in range(samples-4) if all(predicate(r) for r in rows[i:i+5])]

    signed = lambda r: r["vx"]*direction[0]+r["vy"]*direction[1]
    rest = lambda r: speed(r) <= .012 and abs(r["wz"]) <= .04
    forward = blocks(lambda r: signed(r) >= .05)
    pairs = []
    if role == "accelerate":
        pairs = [(0, 4, a, b) for a, b in forward if a > 4 and all(rest(r) for r in rows[:5])]
    elif role == "brake":
        stops = [b["at_us"] for a, b in zip(targets, targets[1:])
                 if all(b[k] == 0 for k in ("x", "y", "yaw")) and any(a[k] != 0 for k in ("x", "y", "yaw"))]
        pairs = [(a, b, c, d) for a, b in forward for c, d in blocks(rest)
                 if b < c and any(rows[a]["at_us"] < t <= rows[c]["at_us"] for t in stops)]
    elif role == "reverse":
        pairs = [(a, b, c, d) for a, b in forward for c, d in blocks(lambda r: signed(r) <= -.05) if b < c]
        crossings = [[a["at_us"], b["at_us"]] for a, b in zip(rows, rows[1:]) if signed(a) >= 0 > signed(b)]
        result["sign_crossing_brackets_us"] = crossings[:5]
        if not crossings: pairs = []
    elif role == "turn":
        # Translation and yaw rate must occur in the SAME five-slot block.
        pairs = blocks(lambda r: abs(r["wz"]) >= .20 and speed(r) >= .05)
    result["observed_blocks"] = [[rows[i]["at_us"] for i in pair] for pair in pairs[:1]]
    result["passed"] = bool(pairs) and path >= .02 and (role != "turn" or yaw >= .10)
    return result


def _binding(capture, case):
    issues = []
    actual, nominal = capture.get("sensor_rows", []), capture.get("nominal_sensor_rows", [])
    refs, targets = capture.get("reference_rows", []), capture.get("targets", [])
    role, delay = case["role"], case["delay_us"]
    # Strict public schema normalization also rejects duplicates and omitted slots.
    normalized = base._rows(nominal, base.SENSOR_FIELDS, "delivered_us",
                            [600_000+k*10_000+d+10_000 for k in range(451) for d in (0, 2500)], issues)
    actual = base._rows(actual, base.SENSOR_FIELDS, "delivered_us",
                       [600_000+k*10_000+d+delay for k in range(451) for d in (0, 2500)], issues)
    refs = base._rows(refs, base.REFERENCE_FIELDS, "at_us", list(range(580_000, 5_120_001, 2500)), issues)
    fields = ("at_us", "x", "y", "yaw")
    try:
        planned = [dict(zip(fields, (600_000+t*1000, *vector))) for t, vector in case["targets_ms"]]
        if (not planned or any(type(r["at_us"]) is not int for r in planned)
                or [r["at_us"] for r in planned] != sorted(set(r["at_us"] for r in planned))):
            raise ValueError("Invalid planned targets")
        executed = [{k: r[k] for k in fields} for r in targets]
        if executed != planned:
            issues.append("Executed targets differ from planned command times or values")
        if any(type(r[k]) not in (int, float) or not math.isfinite(r[k]) for r in executed for k in fields):
            raise ValueError("Invalid executed targets")
    except (KeyError, TypeError, ValueError):
        executed = []
        issues.append("Missing or malformed executed/planned targets")
    if len(normalized) == len(actual) == 902:
        values = ({}, {-1: tuple(normalized[0][k] for k in ("gx", "gy", "gz"))})
        for k in range(451):
            values[0][k] = tuple(normalized[k*2][f] for f in ("ax", "ay", "az"))
            values[1][k] = tuple(normalized[k*2+1][f] for f in ("gx", "gy", "gz"))
        for name, rows in (("nominal", normalized), ("actual", actual)):
            for i, row in enumerate(rows):
                k, die = divmod(i, 2)
                for channel, index, keys in ((0, k, ("ax", "ay", "az")), (1, k if die else k-1, ("gx", "gy", "gz"))):
                    if name == "actual":
                        if 80 <= index <= 109 and (role == "producer_freeze" or (role == "accel_freeze" and channel == 0)):
                            index = 79
                        if role == "gyro_reorder" and channel == 1 and index == 80:
                            index = 76
                    clock = "gyro_us" if channel else "accel_us"
                    if row[clock] != 600_000+index*10_000+(2500 if channel else 0) or tuple(row[f] for f in keys) != values[channel][index]:
                        issues.append(f"{name} sensor event {i} changed expected held source/payload")
    hashes = dict(nominal_sha256=_hash(normalized), reference_sha256=_hash(refs), targets_sha256=_hash(executed))
    return issues, hashes, refs, executed


def score_qualification(capture, case):
    """Return a compact per-case JSON result, preserving smoke score semantics."""
    role = case["role"]
    if role not in (*ROLES, *FAULTS) or case["preset"] not in PRESETS or case["delay_us"] not in PHASES:
        raise ValueError("Unknown qualification case")
    result = base.score_case(dict(capture, metadata=dict(capture.get("metadata", {}), profile=role)))
    extras, hashes, refs, targets = _binding(capture, case)
    metadata = capture.get("metadata", {})
    if metadata.get("source_error"):
        extras.append("Source producer error: "+str(metadata["source_error"]))
    for key in ("seed", "delay_us"):
        if metadata.get(key) != case[key]: extras.append("Capture/case identity mismatch: "+key)
    if metadata.get("profile") != role:
        extras.append("Capture/case role mismatch")
    planned, actual = [], []
    for start in ([600_000, 3_100_000] if role == "stationary" or role in FAULTS else [600_000]):
        cov_role = role if start == 600_000 else "stationary"
        for offset in (480_000, 600_000, 482_500, 592_500):
            item = coverage(refs, targets, cov_role, case.get("direction_body_xy"), start+offset)
            item["start_us"] = start
            planned.append(item)
    for epoch in result["epochs"]:
        if not epoch["anchor_verified"]:
            extras.append("Actual coverage anchor unavailable")
            continue
        cov_role = role if epoch["start_us"] == 600_000 else "stationary"
        count = epoch["expected_samples"] if not epoch["complete"] else 100
        item = coverage(refs, targets, cov_role, case.get("direction_body_xy"), epoch["anchor_us"], samples=count)
        item.update(start_us=epoch["start_us"], complete=epoch["complete"])
        actual.append(item)
    if any(not item["passed"] for item in planned): extras.append("Planned anchor-extreme motion coverage not realized")
    if any(not item["passed"] for item in actual): extras.append("Actual scored-window motion coverage not realized")
    extras = list(dict.fromkeys(extras))
    result["counts"]["incomplete"] += len(extras)
    result["incomplete"] = (result["incomplete"]+extras)[:base.MAX_EXAMPLES]
    result["incomplete_counts"].update(dict(Counter(extras)))
    if extras and result["outcome"] == "PASS": result["outcome"], result["passed"] = "INCOMPLETE", False
    result.update(case=case["case"], preset=case["preset"], role=role, category=case["class"],
                  source_id=case["source_id"], control_case=case["control_case"], binding=hashes,
                  coverage=dict(passed=len(actual) == len(result["epochs"]) and all(i["passed"] for i in planned+actual), planned=planned, actual=actual))
    return result


def summarize_qualification(results, plan):
    """Validate complete ordered matrix and phase/control pairing; report groups."""
    cases = plan["cases"] if isinstance(plan, dict) else plan
    results = list(results)
    issues = []
    expected_ids, actual_ids = [c["case"] for c in cases], [r["case"] for r in results]
    if len(expected_ids) != 144 or len(set(expected_ids)) != 144 or actual_ids != expected_ids:
        issues.append("Expected all 144 unique attempts in planned order")
    healthy = [c for c in cases if c["class"] == "healthy"]
    faults = [c for c in cases if c["class"] == "fault"]
    seeds = {c["seed"] for c in healthy}
    expected_matrix = {(p, r, s, d) for p in PRESETS for r in ROLES for s in seeds for d in PHASES}
    if len(seeds) != 3 or len(healthy) != 135 or {(c["preset"], c["role"], c["seed"], c["delay_us"]) for c in healthy} != expected_matrix:
        issues.append("Healthy matrix differs from 3x5x3x3")
    if len(faults) != 9 or {(c["preset"], c["role"]) for c in faults} != {(p, r) for p in PRESETS for r in FAULTS}:
        issues.append("Fault matrix differs from 3x3")
    seed_for_fault = {role: {c["seed"] for c in faults if c["role"] == role} for role in FAULTS}
    if any(len(s) != 1 for s in seed_for_fault.values()) or set().union(*seed_for_fault.values()) != seeds:
        issues.append("Fault seed pairing differs")
    identities = {(c["preset"], c["role"], c["seed"], c["delay_us"]): c["case"] for c in cases}
    found = {r["case"]: r for r in results}
    for case in cases:
        is_fault = case["role"] in FAULTS
        key = (case["preset"], "stationary" if is_fault else case["role"], case["seed"], case["delay_us"] if is_fault else 10_000)
        expected_control = identities.get(key)
        if case["control_case"] != expected_control:
            issues.append("Wrong phase/stationary control: "+case["case"])
        if is_fault and case["delay_us"] != PHASES[(PRESETS.index(case["preset"])+FAULTS.index(case["role"])) % 3]:
            issues.append("Fault phase rotation differs: "+case["case"])
        result, control = found.get(case["case"]), found.get(expected_control)
        if result and control and result.get("binding") != control.get("binding"):
            issues.append("Nominal/reference/executed-target pairing differs: "+case["case"])

    def group(subset):
        worst = {}
        for result in subset:
            for metric, peak in result.get("peaks", {}).items():
                if metric not in worst or peak["value"] > worst[metric]["value"]:
                    worst[metric] = dict(peak, case=result["case"])
        return dict(cases=len(subset), passed=sum(r["outcome"] == "PASS" for r in subset),
                    product_failed=sum(r["outcome"] == "FAIL" for r in subset),
                    incomplete=sum(r["counts"]["incomplete"] > 0 for r in subset), peaks=worst,
                    samples=sum(r["counts"]["samples"] for r in subset))
    summary = dict(all=group(results), healthy=group([r for r in results if r["category"] == "healthy"]),
                   fault=group([r for r in results if r["category"] == "fault"]),
                   presets={p: group([r for r in results if r["preset"] == p]) for p in PRESETS},
                   phases={str(d): group([r for r in results if r["delay_us"] == d]) for d in PHASES},
                   pairing_issues=issues, limits=dict(base.LIMITS))
    summary["outcome"] = "FAIL" if summary["all"]["product_failed"] else "INCOMPLETE" if issues or summary["all"]["incomplete"] else "PASS"
    return summary
