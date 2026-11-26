"""Evidence identity and explicit public-development input validation."""
import hashlib
import json
import math
from itertools import product
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ARMS = tuple(f"{source}-{mode}" for source in ("release", "candidate")
             for mode in ("default", "encoder", "fused"))
IDENTITY = ("preset", "course", "surface", "seed")
FILES = ("case.json", "sensors.csv", "firmware.csv", "reference.csv", "targets.csv", "status.csv")
METRICS = {"velocity": "tracking_linear_velocity_rmse_m_s", "position": "tracking_position_rmse_m"}
BASELINE_SHA = "a3270391e63c4fc4e29c58deae08932da2260c23e091ac588a88decbc4a72ce3"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text(encoding="utf8"))


def spec_identity(spec):
    return hashlib.sha256(json.dumps(spec,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()


def save(path, value):
    with Path(path).open("x", encoding="utf8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def spec_at(path):
    """Validate a caller's already materialized public workload; never draw one."""
    spec = load(path)
    known = load(HERE / "development.json")
    if set(spec) != set(known):
        raise ValueError("Use an explicit materialized spec (no held_out or unknown fields)")
    require(spec["format"] == 1 and spec["revision"] == "0.4.0" and spec["step_ms"] == 10,
            "This recorder requires format1, v0.4.0 and 10ms polls")
    seeds = spec["seeds"]
    require(seeds and all(type(x) is int and 0 <= x <= 2**32-10001 for x in seeds) and len(set(seeds)) == len(seeds),
            "Supply distinct uint32 seeds leaving room for the IMU seed offset")
    require(type(spec["encoder_counts_per_rev"]) is int and 0 < spec["encoder_counts_per_rev"] <= 65535,
            "Invalid encoder resolution")
    require(spec["presets"] and set(spec["presets"]) <= set(known["presets"]), "Unknown/empty presets")
    for preset,geometry in spec["presets"].items():
        require(set(geometry)==set(known["presets"][preset]) and all(finite(x) and x>0 for x in geometry.values()),
                "Geometry must contain the positive public dimensions in metres")
    for field in ("wheel_pid","position_pid"):
        require(set(spec[field])==set(known[field]) and all(finite(x) and x>=0 for x in spec[field].values()),
                "Invalid controller configuration")
    require(all(spec["position_pid"][k]>0 for k in ("max_linear_speed","max_angular_speed","position_tolerance","heading_tolerance")),
            "Positive position limits and tolerances required")
    require(spec["courses"] and set(spec["courses"]) <= {"velocity","position"}, "Unknown/empty courses")
    for name,course in spec["courses"].items():
        require(set(course)=={"duration_ms","targets"}, "Invalid course fields")
        duration=course["duration_ms"]
        require(type(duration) is int and duration>=100 and duration%100==0, "Course duration must be whole 100ms periods")
        schedule=course["targets"]
        require(schedule and all(isinstance(row,list) and len(row)==2 for row in schedule), "Invalid target rows")
        times=[row[0] for row in schedule]
        require(times[0]==0 and all(type(t) is int and 0<=t<duration and t%10==0 for t in times)
                and times==sorted(set(times)), "Targets must be unique ordered 10ms grid times starting at0")
        require(all(isinstance(v,list) and len(v)==3 and all(finite(x) for x in v) for _,v in schedule), "Targets require three finite SI coordinates")
        require(any(any(v) for _,v in schedule), "At least one nonzero target is required for response scoring")
    require(spec["surfaces"] and "nominal" in spec["surfaces"] and spec["surfaces"]["nominal"]==[], "Nominal surface must be present and unmodified")
    import re
    for name,intervals in spec["surfaces"].items():
        require(isinstance(name,str) and re.fullmatch(r"[a-z][a-z0-9_]*",name), "Unsafe surface name")
        end=0
        for row in intervals:
            require(isinstance(row,list) and len(row)==3, "Invalid surface interval")
            a,b,grip=row
            require(type(a) is int and type(b) is int and end<=a<b and a%10==b%10==0
                    and b<=max(c["duration_ms"] for c in spec["courses"].values()), "Unordered/off-grid surface interval")
            require(isinstance(grip,list) and len(grip)==4 and all(finite(g) and 0<=g<=1 for g in grip), "Four physical grip factors in [0,1] required")
            end=b
    return spec


def keys(spec):
    return list(product(spec["presets"], spec["courses"], spec["surfaces"], spec["seeds"]))


def case_key(row):
    values = tuple(row[k] for k in IDENTITY)
    if type(values[-1]) is not int:
        raise ValueError("Case seed is not an integer")
    return values


def case_name(key):
    return "-".join(map(str, key))


def case_hashes(directory):
    return {name: sha(directory/name) for name in FILES if (directory/name).is_file()}


def checkout(path):
    path = Path(path).resolve()
    def git(*args):
        return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()
    if git("status", "--porcelain", "--untracked-files=all"):
        raise ValueError(f"Supply a clean firmware checkout: {path}")
    return dict(revision=git("rev-parse", "HEAD"), tree=git("rev-parse", "HEAD^{tree}"))


def source_hashes():
    paths = list(HERE.glob("*.py")) + list(HERE.glob("*.json"))
    paths += [ROOT/"tools"/name for name in ("encoder_baseline.py", "score_encoder_baseline.py")]
    paths += list((ROOT/"kinisi_virtual_board").glob("*.py"))
    paths += list((ROOT/"kinisi_virtual_board/firmware_support").glob("*"))
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in sorted(paths) if p.is_file()}


def nominal_sd():
    path = HERE/"nominal_dispersion.json"
    require(hashlib.sha256(path.read_bytes().replace(b"\r\n",b"\n")).hexdigest()==
            "4b40bc10de70cebca13008507faaf10a115c57d1eceadf89637c806feda0454b",
            "Bundled original nominal dispersion content changed")
    data=load(path)
    if data["source_sha256"] != BASELINE_SHA:
        raise ValueError("Bundled original nominal dispersion changed")
    values={(r["preset"],r["course"]):r["sd"] for r in data["entries"]}
    require(len(data["entries"])==6 and set(values)==set(product(("differential","omni","mecanum"),METRICS)), "Incomplete nominal dispersion")
    require(all(finite(v) and v>=0 for v in values.values()), "Invalid nominal dispersion")
    return values


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)
