"""Public analytic stimuli independent of firmware estimates and motor models.

World velocity increments follow cubic smoothsteps. Position and yaw are their
closed-form integrals. The BMI088 receives interval-mean specific force/rate in
the body frame; references use exact endpoint motion. No reference enters the
firmware. These disclosed profiles are development regressions, not qualification.
"""
import math

from .bmi088 import BMI088, G

START_US = 600000
END_US = 5120000
MODEL_STEP_US = 250
REFERENCE_STEP_US = 2500
IMU_FIELDS = ("delivered_us", "accel_us", "gyro_us", "ax", "ay", "az", "gx", "gy", "gz")
REFERENCE_FIELDS = ("at_us", "x", "y", "yaw", "vx", "vy", "wz")
# Public independent-analytic-v1 shapes, declared before comparing firmware.
# Each tuple is (absolute start seconds, ramp seconds, world dvx, dvy, dyaw_rate).
PROFILES = {
    "stationary": (),
    "translate": ((1.3, .06, .22, 0., 0.),),
    "brake": ((1.3, .12, .2, 0., 0.), (1.66, .1, -.2, 0., 0.)),
    "reverse": ((1.3, .12, .15, 0., 0.), (1.66, .16, -.3, 0., 0.)),
    "turn": ((1.3, .13, .17, 0., .75), (1.73, .16, 0., 0., -1.5)),
    "stale": (),
}


def motion(at_seconds, commands):
    """Return world x,y,yaw,vx,vy,wz at an absolute analytic time."""
    velocity, position = [0., 0., 0.], [0., 0., 0.]
    for start, duration, *change in commands:
        u = max(0., min(1., (at_seconds-start)/duration))
        weight = u*u*(3.-2.*u)
        integral = duration*(u**3-.5*u**4)+max(0., at_seconds-start-duration)
        for axis, delta in enumerate(change):
            velocity[axis] += weight*delta
            position[axis] += integral*delta
    return (*position, *velocity)


def into_body(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x+s*y, -s*x+c*y


def generate_case(*, profile, seed, delay_us=10000):
    """Generate an explicit disclosed stimulus with independent die timestamps."""
    if profile not in PROFILES:
        raise ValueError("Unknown public IMU regression profile")
    if type(seed) is not int or not 0 <= seed <= 0xffffffff:
        raise ValueError("Provide an explicit unsigned 32-bit sensor seed")
    if type(delay_us) is not int or delay_us not in (10000, 13000, 17000):
        raise ValueError("Transport delay must be 10000, 13000 or 17000 us")
    commands = PROFILES[profile]
    model = BMI088(seed=seed, noise_seed=seed, temperature_c=25.)
    reference, events = [], []
    prior_stamps = (0, 0)
    previous = motion(0., commands)
    dt = MODEL_STEP_US/1e6
    for t in range(MODEL_STEP_US, END_US+1, MODEL_STEP_US):
        now = motion(t/1e6, commands)
        middle = motion((t-MODEL_STEP_US/2)/1e6, commands)
        ax, ay = into_body((now[3]-previous[3])/dt, (now[4]-previous[4])/dt, middle[2])
        model.advance(t, body_accel=(ax, ay, G), body_gyro=(0., 0., (now[2]-previous[2])/dt))
        stamps = (model.accel_us, model.gyro_us)
        if stamps != prior_stamps and START_US <= t <= 5102500:
            events.append(dict(zip(IMU_FIELDS, (t+delay_us, *stamps, *model.values), strict=True)))
        if t >= START_US-20000 and t % REFERENCE_STEP_US == 0:
            reference.append(dict(zip(REFERENCE_FIELDS,
                (t, *now[:3], *into_body(now[3], now[4], now[2]), now[5]), strict=True)))
        prior_stamps, previous = stamps, now
    if len(events) != 902:
        raise RuntimeError("Unexpected independent sensor event count")
    if profile == "stale":
        # Freeze each die after its own slot79. Repeated deliveries retain exact
        # acquisition identities and values, then resume the original clocks.
        original = [dict(row) for row in events]
        for index, row in enumerate(events):
            slot, gyro = divmod(index, 2)
            if 80 <= slot < 110:
                for key in ("accel_us", "ax", "ay", "az"):
                    row[key] = original[158][key]
            if 80 <= slot-(1-gyro) < 110:
                for key in ("gyro_us", "gx", "gy", "gz"):
                    row[key] = original[159][key]
    metadata = dict(profile=profile, seed=seed, delay_us=delay_us, start_us=START_US,
                    restart_us=3100000 if profile in ("stationary", "stale") else None,
                    end_us=END_US, poll_us=1000)
    return dict(metadata=metadata, sensor_rows=events, reference_rows=reference)
