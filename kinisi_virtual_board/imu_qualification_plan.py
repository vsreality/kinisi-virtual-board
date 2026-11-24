"""Disclosed public chassis experiment, fixed before any fresh seed draw/run."""
import hashlib
import math
import random

DESIGN_VERSION = "kinisi-public-chassis-imu-v1"
PRESETS = ("differential", "omni", "mecanum")
ROLES = ("stationary", "accelerate", "brake", "reverse", "turn")
FAULTS = ("producer_freeze", "accel_freeze", "gyro_reorder")
DELAYS = (10000, 13000, 17000)
PUBLIC_SEED_RANGE = (0xc0000000, 0xffff0000)
GEOMETRY = {
    "differential": dict(wheel_diameter=.1, wheel_base=.3),
    "omni": dict(wheel_diameter=.1, robot_radius=.17),
    "mecanum": dict(wheel_diameter=.1, width=.26, length=.34),
}
WHEEL_PID = dict(kp=10, ki=20, kd=0, integral_limit=100)
ENCODER_RESOLUTION = 4096
# Inclusive discrete bounds, sampled with an independent role/preset schedule
# stream. They are not adjusted after examining a simulation or diagnostic.
SCHEDULE_BOUNDS = dict(start_ms=(630,690,10), moving_start_min_ms=640,
    brake_ms=(950,1000,10), reverse_ms=(950,1020,10), stop_ms=(1700,1780,10),
    vx_mm_s=(165,190), vy_mm_s=(30,55), turn_centirad_s=(56,70))


def schedule(preset, role, seed):
    if preset not in PRESETS or role not in ROLES:
        raise ValueError("Unknown public chassis schedule")
    if type(seed) is not int or not 0 <= seed <= 0xffffffff-10000:
        raise ValueError("Seed must permit unsigned32 sensor seed+10000")
    if role == "stationary":
        return [[0, [0.,0.,0.]]], None
    rng = random.Random(f"{DESIGN_VERSION}:{preset}:{role}:{seed}")
    start_lo, start_hi, step = SCHEDULE_BOUNDS["start_ms"]
    if role in ("accelerate", "turn"):
        start_lo = SCHEDULE_BOUNDS["moving_start_min_ms"]
    start = rng.randrange(start_lo, start_hi+1, step)
    low,high,step = SCHEDULE_BOUNDS["brake_ms" if role=="brake" else "reverse_ms"]
    switch = rng.randrange(low,high+1,step)
    low,high,step = SCHEDULE_BOUNDS["stop_ms"]
    stop = rng.randrange(low,high+1,step)
    vx = rng.choice((-1,1))*rng.randint(*SCHEDULE_BOUNDS["vx_mm_s"])/1000
    vy = 0. if preset == "differential" else rng.choice((-1,1))*rng.randint(*SCHEDULE_BOUNDS["vy_mm_s"])/1000
    wz = rng.choice((-1,1))*rng.randint(*SCHEDULE_BOUNDS["turn_centirad_s"])/100 if role == "turn" else 0.
    direction = [vx/math.hypot(vx,vy), vy/math.hypot(vx,vy)]
    program = [[0,[0.,0.,0.]], [start,[vx,vy,wz]]]
    if role == "brake":
        program.append([switch,[0.,0.,0.]])
    else:
        if role == "reverse":
            program.append([switch,[-vx,-vy,0.]])
        program.append([stop,[0.,0.,0.]])
    return program, direction


def source_key(case):
    return (case["preset"], "stationary" if case["role"] in FAULTS else case["role"], case["seed"])


def make_plan(seeds):
    """Return all144 cases; caller draws/persists exactly three explicit seeds.

    Disclosed seeds are also accepted for tests. Fresh public execution uses
    PUBLIC_SEED_RANGE. There is no random draw, file access or native execution.
    """
    if type(seeds) not in (list,tuple) or len(seeds) != 3 or len(set(seeds)) != 3:
        raise ValueError("Supply three distinct public seeds")
    if any(type(seed) is not int or not 0 <= seed <= 0xffffffff-10000 for seed in seeds):
        raise ValueError("Each seed must permit unsigned32 seed+10000")
    cases = []
    def add(preset, role, seed, delay):
        healthy = "stationary" if role in FAULTS else role
        targets, direction = schedule(preset, healthy, seed)
        cases.append(dict(case=f"q-{len(cases)+1:03d}", preset=preset, role=role,
            seed=seed, delay_us=delay, targets_ms=targets, direction_body_xy=direction,
            source_id=f"{preset}-{healthy}-{seed:08x}",
            **{"class":"fault" if role in FAULTS else "healthy"}))
    for index,preset in enumerate(PRESETS):
        for role in ROLES:
            for seed in seeds:
                for delay in DELAYS:
                    add(preset,role,seed,delay)
        for fault_index,role in enumerate(FAULTS):
            add(preset,role,seeds[fault_index],DELAYS[(index+fault_index)%3])
    by_variant = {(c["preset"],c["role"],c["seed"],c["delay_us"]):c["case"] for c in cases}
    for case in cases:
        preset,healthy,seed = source_key(case)
        delay = case["delay_us"] if case["role"] in FAULTS else 10000
        case["control_case"] = by_variant[preset,healthy,seed,delay]
    # Random-looking predetermined execution permutation; independent of seeds,
    # wall clock and every observed outcome, reproducible across Python versions.
    cases.sort(key=lambda c: hashlib.sha256((DESIGN_VERSION+":"+c["case"]).encode()).digest())
    return cases
