"""Check actual protocol 2.22 source-health and die-clock replies on board time."""

from enum import IntFlag

NONE_AGE = 0xFFFFFFFF
HEALTHY, SUSPECT, UNAVAILABLE, INDETERMINATE = range(4)


class Flag(IntFlag):
    PROVENANCE = 1
    FRESH = 2
    DISAGREEMENT = 4
    COMPARABLE = 8
    RECOVERING = 16
    RESTART = 32
    TIMESTAMP_ERROR = 64
    SATURATION = 128


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def age(now, stamp):
    return now - stamp if stamp and 0 <= now - stamp < NONE_AGE else NONE_AGE


def sources(session):
    if "GET_FEEDBACK_SOURCE_HEALTH" not in session.schema.commands:
        return {}  # Historical baseline. The separate protocol case records the gap.
    result = {}
    now = session.board.now_us
    for index, name in enumerate(("encoder", "imu", "fused")):
        row = session.ask("GET_FEEDBACK_SOURCE_HEALTH", source=index)
        require(
            row["source"] == index and row["health"] in range(4),
            f"Invalid {name} identity: {row}",
        )
        stamp = row["timestamp_us"]
        require(
            bool(stamp) == bool(row["flags"] & Flag.PROVENANCE),
            f"{name} provenance: {row}",
        )
        require(row["age_us"] == age(now, stamp), f"{name} age at {now}: {row}")
        if row["flags"] & Flag.FRESH:
            require(
                0 < stamp <= now and row["age_us"] <= row["max_age_us"],
                f"False freshness: {row}",
            )
        result[name] = row
    dies = session.ask("GET_FEEDBACK_IMU_CHANNELS")
    for die in ("accel", "gyro"):
        stamp = dies[die + "_timestamp_us"]
        require(dies[die + "_age_us"] == age(now, stamp), f"{die} age at {now}: {dies}")
    imu = result["imu"]
    if imu["health"] == HEALTHY:
        stamps = [dies[k + "_timestamp_us"] for k in ("accel", "gyro")]
        require(
            min(stamps) == imu["timestamp_us"]
            and abs(stamps[0] - stamps[1]) <= 15000
            and imu["age_us"] <= 35000,
            f"IMU pair outside source-time limits: {imu}, {dies}",
        )
    result["dies"] = dies
    return result


def available(rows, names=("encoder", "imu", "fused")):
    for name in names:
        row = rows[name]
        require(
            row["health"] == HEALTHY
            and row["reason"] == row["detail"] == 0
            and row["flags"] & Flag.FRESH
            and not row["flags"] & Flag.DISAGREEMENT,
            f"Expected directly healthy {name}: {row}",
        )


def unattributed_conflict(rows):
    flags = Flag.DISAGREEMENT | Flag.COMPARABLE | Flag.FRESH | Flag.PROVENANCE
    for name in ("encoder", "imu", "fused"):
        row = rows[name]
        require(
            (row["health"], row["reason"], row["detail"]) == (INDETERMINATE, 0, 0)
            and row["flags"] & flags == flags,
            f"Expected conflicting fresh inputs with no identified failed sensor: {name} {row}",
        )
