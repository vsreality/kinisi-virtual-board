"""Developer-run v0.4 IMU qualification. Publish inputs first and retain each attempt."""
import csv
from datetime import datetime, timezone
import gzip
import hashlib
import io
import json
from pathlib import Path
import secrets
import shutil
import sys

from .bmi088 import MODEL_ID
from .firmware_build import build_firmware
from .imu_regression import checkout_info
from .imu_regression_capture import PUBLIC_FIELDS, capture_sensor_stream
from .imu_regression_motion import IMU_FIELDS, REFERENCE_FIELDS
from .imu_regression_score import LIMITS
from .imu_qualification_plan import DESIGN_VERSION, PUBLIC_SEED_RANGE, make_plan
from .schema import Schema

DESIGN = DESIGN_VERSION
SOURCE_FILES = ("reference.csv.gz", "nominal.csv.gz", "targets.json")


def write_json(path, value):
    # A reader or interrupted process sees the previous complete checkpoint.
    tmp = path.with_suffix(".tmp")
    tmp.write_bytes((json.dumps(value, indent=2, allow_nan=False) + "\n").encode())
    tmp.replace(path)


def read_json(path):
    return json.loads(path.read_bytes())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_csv(path, fields, rows):
    with path.open("xb") as file, gzip.GzipFile(filename="", fileobj=file, mode="wb", compresslevel=1, mtime=0) as zipped:
        with io.TextIOWrapper(zipped, encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)


def read_source_rows(path, fields):
    with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != list(fields):
            raise ValueError(f"Wrong columns in {path.name}")
        return [{key: int(row[key]) if key.endswith("_us") else float(row[key]) for key in fields} for row in reader]


def firmware_identity(path, compiler):
    schema = Schema(path / "commands.json")
    _, build = build_firmware(schema, path, compiler=compiler)
    return dict(checkout_info(path), build_id=build["build_id"], compiler=build["compiler"],
                protocol=schema.document["version"], schema_sha256=schema.sha256)


def create_source(folder, previous, producer_path, producer, case, compiler, expected_sha=None):
    from .imu_qualification_motion import source_for_case
    folder.mkdir()
    if previous is not None:
        original = previous / "sources" / case["source_id"]
        if digest(original / "source.json") != expected_sha:
            raise ValueError("Archived source receipt differs from the original run report")
        receipt = read_json(original / "source.json")
        if receipt["source_id"] != case["source_id"] or receipt["metadata"]["build_id"] != producer["build_id"]:
            raise ValueError("Archived motion source identity differs from the plan")
        for name in SOURCE_FILES:
            shutil.copyfile(original / name, folder / name)
            if digest(folder / name) != receipt["files"][name]:
                raise ValueError(f"Archived motion source changed: {name}")
        source = dict(reference_rows=read_source_rows(folder / "reference.csv.gz", REFERENCE_FIELDS),
                      sensor_rows=read_source_rows(folder / "nominal.csv.gz", IMU_FIELDS),
                      targets=read_json(folder / "targets.json"), metadata=receipt["metadata"])
    else:
        source = source_for_case(producer_path, case, compiler=compiler)
        write_csv(folder / "reference.csv.gz", REFERENCE_FIELDS, source["reference_rows"])
        write_csv(folder / "nominal.csv.gz", IMU_FIELDS, source["sensor_rows"])
        write_json(folder / "targets.json", source["targets"])
    if source["metadata"]["build_id"] != producer["build_id"]:
        raise ValueError("Motion source build changed")
    write_json(folder / "source.json", dict(source_id=case["source_id"], metadata=source["metadata"],
                                            files={name: digest(folder / name) for name in SOURCE_FILES}))
    return source


def run_case(folder, firmware, candidate, case, source, compiler):
    from .imu_qualification_motion import capture_inputs
    from .imu_qualification_score import score_qualification
    inputs = capture_inputs(source, case)
    write_csv(folder / "imu.csv.gz", IMU_FIELDS, inputs["sensor_rows"])
    replay = capture_sensor_stream(firmware, inputs["sensor_rows"], inputs["metadata"], compiler=compiler)
    write_csv(folder / "public.csv.gz", PUBLIC_FIELDS, replay["public_rows"])
    metadata = replay["metadata"]
    if metadata.get("build_id") != candidate["build_id"]:
        raise ValueError(metadata.get("error", "Diagnostic build changed"))
    result = score_qualification(dict(inputs, **replay), case)
    result["capture_metadata"] = metadata
    return result


def counts(rows, planned):
    complete = sum(row["outcome"] == "PASS" for row in rows)
    return dict(planned=planned, attempted=len(rows), pending=planned-len(rows), passed=complete,
                product_failed=sum(row["outcome"] == "FAIL" for row in rows),
                incomplete=sum(row["incomplete"] for row in rows))


def checkpoint(folder, plan, plan_sha, candidate, rows, source_receipts, replay_origin):
    tallies = counts(rows, len(plan["cases"]))
    overall = ("FAIL" if tallies["product_failed"] else "INCOMPLETE" if tallies["incomplete"] or tallies["pending"]
               else "PASS")
    groups = {}
    for field in ("preset", "class"):
        groups[field] = {key: counts([r for r in rows if r[field] == key],
                                     sum(c[field] == key for c in plan["cases"]))
                         for key in sorted({c[field] for c in plan["cases"]})}
    peaks = {}
    for row in rows:
        for metric, peak in row["peaks"].items():
            if metric not in peaks or peak["value"] > peaks[metric]["value"]:
                peaks[metric] = dict(peak, case=row["case"])
    report = dict(outcome=overall, design=DESIGN, model=MODEL_ID, plan_sha256=plan_sha, candidate=candidate,
                  producer=plan["producer"], seeds=plan["seeds"], counts=tallies, groups=groups, limits=LIMITS,
                  peaks=peaks, cases=rows, source_receipts=source_receipts, replay_origin=replay_origin)
    write_json(folder / "report.json", report)
    return report


def prepare(args, firmware):
    candidate = firmware_identity(firmware, args.compiler)
    previous = None
    previous_receipts, replay_origin = {}, None
    producer_path = (args.motion_firmware or firmware).resolve()
    if args.replay:
        previous = args.replay.resolve()
        if previous.is_file():
            if previous.name != "plan.json":
                raise ValueError("--replay expects a run directory or its plan.json")
            previous = previous.parent
        plan = read_json(previous / "plan.json")
        previous_report = read_json(previous / "report.json")
        replay_origin = dict(plan_sha256=digest(previous / "plan.json"), report_sha256=digest(previous / "report.json"))
        if previous_report["plan_sha256"] != replay_origin["plan_sha256"]:
            raise ValueError("Archived plan differs from its run report")
        previous_receipts = previous_report["source_receipts"]
        if plan["design"] != DESIGN or plan["limits"] != LIMITS or plan["cases"] != make_plan(plan["seeds"]):
            raise ValueError("Replay plan differs from the published design and schedules")
    else:
        producer = candidate if producer_path == firmware else firmware_identity(producer_path, args.compiler)
        # Select the tested build before drawing three public, disjoint-range seeds.
        seeds = secrets.SystemRandom().sample(range(*PUBLIC_SEED_RANGE), 3)
        plan = dict(design=DESIGN, limits=LIMITS, model=MODEL_ID, candidate=candidate, producer=producer,
                    simulator=checkout_info(Path(__file__).resolve().parent.parent), seeds=seeds, cases=make_plan(seeds))
    return candidate, plan, previous, previous_receipts, replay_origin, producer_path


def execute(args):
    from .imu_qualification_score import summarize_qualification
    firmware = args.firmware.resolve()
    folder = args.out or Path("build/imu-qualification") / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    folder = folder.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    try:
        candidate, plan, previous, previous_receipts, replay_origin, producer_path = prepare(args, firmware)
    except Exception as error:
        message = dict(outcome="INCOMPLETE", stage="setup", attempted=0, error=f"{type(error).__name__}: {error}")
        write_json(folder / "report.json", message)
        print(json.dumps(dict(message, report=str(folder / "report.json")), indent=2))
        return 2
    write_json(folder / "plan.json", plan)
    plan_sha = digest(folder / "plan.json")
    print(f"Published {folder / 'plan.json'} (seeds {plan['seeds']})", file=sys.stderr, flush=True)
    (folder / "sources").mkdir()
    (folder / "cases").mkdir()
    rows, sources, details, source_receipts = [], {}, [], {}
    checkpoint(folder, plan, plan_sha, candidate, rows, source_receipts, replay_origin)
    for case in plan["cases"]:
        name, source_id = case["case"], case["source_id"]
        case_folder = folder / "cases" / name
        case_folder.mkdir()
        identity = {field: case[field] for field in ("case", "source_id", "preset", "class", "role", "seed", "delay_us")}
        try:
            if source_id not in sources:
                try:
                    sources[source_id] = create_source(folder / "sources" / source_id, previous, producer_path,
                                                       plan["producer"], case, args.compiler, previous_receipts.get(source_id))
                    source_receipts[source_id] = digest(folder / "sources" / source_id / "source.json")
                except Exception as error:
                    sources[source_id] = f"{type(error).__name__}: {error}"
                    write_json(folder / "sources" / source_id / "error.json", {"error": sources[source_id]})
            source = sources[source_id]
            if isinstance(source, str):
                raise ValueError(source)
            result = run_case(case_folder, firmware, candidate, case, source, args.compiler)
        except Exception as error:
            result = dict(outcome="INCOMPLETE", failures=[], incomplete=[f"{type(error).__name__}: {error}"],
                          peaks={}, counts=dict(samples=0, incomplete=1))
        result.update(identity, files={p.name: digest(p) for p in case_folder.glob("*.csv.gz")})
        result["category"] = case["class"]
        details.append(result)
        write_json(case_folder / "result.json", result)
        rows.append(dict(identity, outcome=result["outcome"], incomplete=bool(result["incomplete"]),
                         reason=(result["failures"] or result["incomplete"] or [None])[0],
                         samples=result["counts"].get("samples", 0), peaks=result["peaks"],
                         result_sha256=digest(case_folder / "result.json")))
        report = checkpoint(folder, plan, plan_sha, candidate, rows, source_receipts, replay_origin)
        if result["outcome"] != "PASS" or len(rows) % 12 == 0:
            print(f"{len(rows)}/144 {name}: {result['outcome']}", file=sys.stderr, flush=True)
    summary = summarize_qualification(details, plan)
    report.update(outcome=summary["outcome"], matrix=summary)
    write_json(folder / "report.json", report)
    print(json.dumps(dict(outcome=report["outcome"], counts=report["counts"], firmware=candidate,
                          seeds=plan["seeds"], report=str(folder / "report.json")), indent=2))
    return {"PASS": 0, "FAIL": 1, "INCOMPLETE": 2}[report["outcome"]]
