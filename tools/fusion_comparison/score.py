"""Re-read hashed public traces; retain missing evidence and product failures separately."""
import csv
import math
from statistics import fmean

import score_encoder_baseline as native_score
from core import (ARMS, FILES, HERE, IDENTITY, METRICS, case_hashes, case_key, case_name,
                  keys, load, nominal_sd, require, sha, spec_identity)


def status_rows(path):
    with path.open(newline="", encoding="utf8") as stream:
        reader = csv.DictReader(stream, strict=True)
        fields = reader.fieldnames or []
        needed = "at_us mode health fault flags sequence sample_us accel_age_us gyro_age_us encoder_age_us x y yaw vx vy wz".split()
        require(fields == needed, "Missing, duplicate or unexpected status fields")
        rows = list(reader)
    for row in rows:
        require(None not in row and None not in row.values(), "Malformed status row")
        for k in needed[:10]:
            require(str(int(row[k])) == row[k], "Noninteger status field")
            limit = 0xffffffffffffffff if k in ("at_us","sample_us") else 0xff if k in ("mode","health","fault","flags") else 0xffffffff
            require(0<=int(row[k])<=limit, "Out-of-range status field")
        for k in needed[10:]:
            require(row[k] == "" or math.isfinite(float(row[k])), "Nonfinite status vector")
        valid = int(row["flags"]) & 1
        require(all((row[k] != "") == bool(valid) for k in needed[10:]), "Status/vector availability mismatch")
    return rows


def score_case(directory, spec, key, mode):
    meta = load(directory/"case.json")
    require(tuple(meta[k] for k in ("preset", "mode", "surface", "seed")) == key, "Case identity mismatch")
    require(meta["origin_us"] == 600000 and meta["duration_ms"] == spec["courses"][key[1]]["duration_ms"], "Case time extent mismatch")
    require(meta["rates"] == dict(controller_hz=10, odometry_hz=50), "Effective rates differ")
    geometry=spec["presets"][key[0]]
    wheels={"differential":2,"omni":3,"mecanum":4}[key[0]]
    expected=[[0,0,wheels-1],[1,0,geometry["wheel_diameter"]],
              [2,0,geometry.get("wheel_base",geometry.get("length",0.))],
              [3,0,geometry.get("robot_radius",geometry.get("width",0.))]]
    for i in range(wheels):
        expected.extend([[4,i,spec["encoder_counts_per_rev"]],[5,i,0],[6,i,0]])
    require(meta["configuration_readback"]==expected,"Actual geometry/encoder configuration differs from supplied spec")
    scores = native_score.score_case(directory, spec)
    fw = native_score.read_csv(directory/"firmware.csv")
    ref = native_score.read_csv(directory/"reference.csv")
    imu = native_score.read_csv(directory/"sensors.csv")
    targets = native_score.read_csv(directory/"targets.csv")
    times = list(range(600000, 600000+meta["duration_ms"]*1000+1, 10000))
    require([r["at_us"] for r in ref] == list(range(580000, times[-1]+1, 2500)), "Reference grid differs")
    require([r["at_us"] for r in fw] == [r["at_us"] for r in imu] == times, "Public observation grid differs")
    require(all(r["accel_us"] == t-10000 and r["gyro_us"] == t-17500 for r,t in zip(imu,times)), "IMU clock contract differs")
    require(all(0 <= t-r["odom_us"] <= 20000 for r,t in zip(fw,times)), "Odometry age outside contract")
    require(all(abs(r[f"pwm{i}"]) <= 100 for r in fw for i in range(4)), "PWM outside actuator bound")
    dispatches = []
    for before, after in zip(fw, fw[1:]):
        change = after["controller_dispatches"]-before["controller_dispatches"]
        require(change in (0,1), "Missing/duplicate controller dispatch")
        if change:
            dispatches.append(after["at_us"])
    require(len(dispatches) == meta["duration_ms"]//100 and
            all(b-a == 100000 for a,b in zip(dispatches, dispatches[1:])), "Controller cadence differs")
    truth = {r["at_us"]: r for r in ref}
    actual = {r["at_us"]: (r["x"],r["y"],r["yaw"]) for r in targets}
    wanted = (0.,0.,0.)
    squared = []
    for before, now in zip(times,times[1:]):
        wanted = actual.get(before, wanted)
        fields = ("vx","vy") if key[1] == "velocity" else ("x","y")
        squared.append(sum((truth[now][f]-wanted[i])**2 for i,f in enumerate(fields)))
    primary = math.sqrt(fmean(squared))
    require(math.isclose(primary, scores[METRICS[key[1]]], rel_tol=1e-10, abs_tol=1e-15), "Independent primary RMSE mismatch")
    health = None
    if mode != "default":
        rows = status_rows(directory/"status.csv")
        require([int(r["at_us"]) for r in rows] == times, "Missing/duplicate status polls")
        require(all(int(r["mode"]) == (1 if mode == "encoder" else 2) for r in rows), "Wrong feedback mode")
        health = dict(samples=len(rows), invalid=sum(not(int(r["flags"])&1) for r in rows),
                      faults=sum(int(r["fault"]) != 0 for r in rows),
                      unhealthy=sum(int(r["health"]) != 2 for r in rows),
                      health_states=sorted({int(r["health"]) for r in rows}),
                      ages={field:dict(unavailable=sum(int(r[field])==0xffffffff for r in rows),
                                      maximum_us=max((int(r[field]) for r in rows if int(r[field])!=0xffffffff),default=None))
                            for field in ("encoder_age_us","accel_age_us","gyro_age_us")})
    return dict(scores=scores, primary=primary, health=health, dispatches=len(dispatches))


def read_arm(root, spec, arm):
    """Read portable and archived evidence-v2 ledgers without workspace imports."""
    run = load(root/"run.json")
    matrix_problems=[]
    if run.get("complete") is not True:
        matrix_problems.append("Run is not complete")
    if "materialized_spec_sha256" in run:
        require(run["materialized_spec_sha256"]==spec_identity(spec), "Materialized spec differs from capture")
    else:
        require(run.get("cases_sha256")=="0bc9380512477110587d8aaf34967c2e99d5e721f34d08f5f8fae71e3e74e9ae"
                and spec==load(HERE/"development.json"), "Legacy archive requires its original public development spec")
    mode = arm.split("-")[1]
    require(run["variant"] == ("release" if mode == "default" else mode), "Arm mode mismatch")
    require(sha(root/"results.json") == run["results_sha256"], "Changed result ledger")
    require(sha(root/"firmware-build.json") == run["build_sha256"], "Changed native build receipt")
    build = load(root/"firmware-build.json")
    results = load(root/"results.json")
    rows = results["valid"] + results["invalid"]
    expected = set(keys(spec))
    row_keys = [case_key(r) for r in rows]
    duplicates={k for k in row_keys if row_keys.count(k)>1}
    if duplicates or set(row_keys)!=expected:
        matrix_problems.append("Missing/duplicate/substituted cases")
    if set(results["case_files"]) != {case_name(k) for k in expected}:
        matrix_problems.append("Case manifest differs")
    planned = [case_key(r) for r in run["planned"]]
    if len(planned) != len(set(planned)) or set(planned) != expected:
        matrix_problems.append("Planned matrix differs")
    out = {}
    invalid = [dict(arm=arm,error=reason) for reason in matrix_problems]
    for row in rows:
        key = case_key(row)
        if key not in expected or key in duplicates:
            continue
        folder = root/case_name(key)
        try:
            require(case_hashes(folder) == results["case_files"][case_name(key)], "Changed case evidence")
            if "error" in row:
                raise ValueError(f"Recorded attempt error: {row['error']}")
            require(set(FILES[:5]).issubset(case_hashes(folder)), "Missing trace files")
            meta = load(folder/"case.json")
            require(meta["firmware_build_id"] == build["build_id"] and meta["schema_sha256"] == build["schema_sha256"], "Case/build identity mismatch")
            result = score_case(folder, spec, key, mode)
            # The original ledger remains visible; a mismatch never becomes a new pass.
            for metric,value in result["scores"].items():
                require(metric in row and ((value is None and row[metric] is None) or
                        (value is not None and row[metric] is not None and
                         math.isclose(value, row[metric], rel_tol=1e-10, abs_tol=1e-12))),
                        f"Retained score differs: {metric}")
            out[key] = result
        except (OSError,ValueError,KeyError,TypeError,ZeroDivisionError,StopIteration,csv.Error) as exc:
            invalid.append(dict(arm=arm,case=case_name(key),error=f"{type(exc).__name__}: {exc}"))
    return out, invalid, results["case_files"], run


def compare(arms, spec, primary_surface="wheel0_slip"):
    require(set(arms) == set(ARMS), "Exactly six named arms are required")
    require(primary_surface != "nominal", "Primary disturbance cannot be nominal")
    rows, hashes, provenance = {}, {}, {}
    invalid = []
    for arm in ARMS:
        try:
            data, bad, manifest, run = read_arm(arms[arm],spec,arm)
            rows.update({(arm,key): value for key,value in data.items()})
            hashes[arm] = manifest
            provenance[arm] = dict(firmware=run["firmware"], simulator=run["simulator"],
                                   run_sha256=sha(arms[arm]/"run.json"))
            invalid.extend(bad)
        except (OSError,ValueError,KeyError,TypeError,csv.Error) as exc:
            invalid.append(dict(arm=arm,error=f"{type(exc).__name__}: {exc}"))
    # Revisions within each source and simulator identity across all arms must pair.
    for source in ("release","candidate"):
        revisions={r["firmware"] for arm,r in provenance.items() if arm.startswith(source+"-")}
        if len(revisions)>1:
            invalid.append(dict(error=f"Mixed firmware revisions within {source}"))
    if len({r["simulator"] for r in provenance.values()})>1:
        invalid.append(dict(error="Unmatched simulator revisions"))
    parity = {}
    for mode in ("default","encoder"):
        a,b = "candidate-"+mode,"release-"+mode
        changed = []
        for key in keys(spec):
            name = case_name(key)
            for file in FILES[1:] if mode == "encoder" else FILES[1:-1]:
                x,y = hashes.get(a,{}).get(name,{}),hashes.get(b,{}).get(name,{})
                if file in x and file in y and x[file] != y[file]:
                    changed.append(dict(case=name,file=file))
        parity[mode] = changed
    health_faults = [dict(arm=arm,case=case_name(key),**r["health"])
                     for (arm,key),r in rows.items() if r["health"] and (r["health"]["invalid"] or r["health"]["faults"] or r["health"]["unhealthy"])]
    sd = nominal_sd()
    groups = []
    for preset in spec["presets"]:
        for course in spec["courses"]:
            for surface in spec["surfaces"]:
                selected = [(preset,course,surface,seed) for seed in spec["seeds"]]
                group = dict(preset=preset,course=course,surface=surface,metric=METRICS[course],
                             seeds=spec["seeds"],primary_gate=surface in ("nominal",primary_surface), comparisons={})
                group["side"] = {arm:{metric:fmean(rows[arm,k]["scores"][metric] for k in selected)
                                      for metric in ("tracking_end_position_m","tracking_end_heading_rad","tracking_heading_rmse_rad",
                                                     "est_position_rmse_m","est_linear_velocity_rmse_m_s","held_sample_age_max_ms")}
                                 for arm in ARMS if all((arm,k) in rows for k in selected)}
                for new,old,label in (("candidate-fused","candidate-encoder","primary"),
                                      ("candidate-fused","release-default","release_comparison"),
                                      ("candidate-fused","release-fused","candidate_increment"),
                                      ("release-fused","release-encoder","existing_fusion")):
                    result = group["comparisons"][label] = {}
                    if not all((arm,k) in rows for arm in (new,old) for k in selected):
                        result["verdict"] = "UNASSESSABLE"
                        continue
                    a,b = ([rows[arm,k]["primary"] for k in selected] for arm in (old,new))
                    av,bv=fmean(a),fmean(b)
                    if av <= 0:
                        result.update(old=a,new=b,old_mean=av,new_mean=bv,verdict="UNASSESSABLE",
                                      reason="Nonpositive reference mean makes relative comparison undefined")
                        invalid.append(dict(preset=preset,course=course,surface=surface,comparison=label,
                                            error=result["reason"]))
                        continue
                    allowance=max(.05*av,2*sd[preset,course])
                    wins=sum(y<x for x,y in zip(a,b))
                    good = bv <= av+allowance if surface == "nominal" else bv <= .9*av and wins > len(a)//2
                    result.update(old=a,new=b,old_mean=av,new_mean=bv,
                                  reduction_pct=100*(1-bv/av) if av else None,favorable=wins,
                                  regression=bv>av,nominal_allowance=allowance if surface=="nominal" else None,
                                  verdict="PASS" if good else "FAIL")
                groups.append(group)
    failed = [dict(preset=g["preset"],course=g["course"],surface=g["surface"])
              for g in groups if g["primary_gate"] and g["comparisons"]["primary"]["verdict"] == "FAIL"]
    full = (set(spec["presets"]) == {"differential","omni","mecanum"} and
            set(spec["courses"]) == {"velocity","position"} and
            {"nominal",primary_surface} <= set(spec["surfaces"]) and
            bool(spec["surfaces"].get(primary_surface)) and len(spec["seeds"])>=3)
    known = load(HERE/"development.json")
    known_subset = (all(spec[k]==known[k] for k in set(known)-{"seeds","presets","courses","surfaces"})
                    and set(spec["seeds"]) <= set(known["seeds"])
                    and all(all(k in known[field] and v==known[field][k] for k,v in spec[field].items())
                            for field in ("presets","courses","surfaces")))
    if known_subset and spec != known:
        full = False
    if not full:
        invalid.append(dict(error="Explicit development subset: full-matrix acceptance is unassessed"))
    failures = bool(failed or health_faults or any(parity.values()))
    return dict(outcome="FAIL" if failures else "INCOMPLETE" if invalid else "PASS",
                tracking_gate="UNASSESSABLE" if invalid else "FAIL" if failures else "PASS",
                verified_cases=len(rows),expected_cases=6*len(keys(spec)),invalids=invalid,
                failed_tracking_groups=failed,health_faults=health_faults,parity=parity,
                groups=groups,provenance=provenance,primary_surface=primary_surface,
                scope="bundled-development-subset" if known_subset and spec!=known else "materialized-public-matrix",
                cases=[dict(arm=a,case=case_name(k),**r) for (a,k),r in rows.items()])
