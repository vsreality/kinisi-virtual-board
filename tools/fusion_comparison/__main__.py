"""Run or re-score the six-arm public development comparison (no fresh draw)."""
import argparse
import json
from pathlib import Path
import sys
import uuid

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent), str(HERE.parents[1])]
from core import ARMS, IDENTITY, case_hashes, case_name, checkout, keys, save, sha, source_hashes, spec_at, spec_identity
from capture import collect
from score import compare


def execute(candidate, reference, spec_path, out, primary_surface="wheel0_slip"):
    """Collect explicit inputs into a new directory; every attempted case is retained."""
    candidate,reference,spec_path,out = [Path(p).resolve() for p in (candidate,reference,spec_path,out)]
    spec = spec_at(spec_path)
    if primary_surface == "nominal":
        raise ValueError("Primary disturbance cannot be nominal")
    pins = dict(candidate=checkout(candidate),release=checkout(reference))
    if any(out == p or out.is_relative_to(p) for p in (candidate,reference)):
        raise ValueError("Output must be outside the firmware checkouts")
    sources = source_hashes()
    spec_sha = sha(spec_path)
    out.mkdir(parents=True,exist_ok=False)
    save(out/"spec.json",spec)
    save(out/"inputs.json",dict(firmware=pins,spec_sha256=spec_sha,sources_sha256=sources,
                               python=sys.version,scope="known public development only; no qualification draw"))
    arms = {}
    simulator_identity = "sha256:"+__import__('hashlib').sha256(json.dumps(sources,sort_keys=True).encode()).hexdigest()
    for arm in ARMS:
        source,mode = arm.split("-")
        fw = reference if source == "release" else candidate
        variant = "release" if mode == "default" else mode
        folder = out/arm
        folder.mkdir()
        arms[arm] = folder
        valid,invalid,files = [],[],{}
        run = dict(complete=False,run_id=uuid.uuid4().hex,variant=variant,
                   firmware=pins[source]["revision"],simulator=simulator_identity,
                   planned=[dict(zip(IDENTITY,k)) for k in keys(spec)],spec_sha256=spec_sha,
                   materialized_spec_sha256=spec_identity(spec))
        save(folder/"started.json",run)
        for key in keys(spec):
            name = case_name(key)
            row = dict(zip(IDENTITY,key))
            try:
                row.update(collect(folder/name,fw,spec,*key,variant))
                valid.append(row)
            except Exception as exc:
                row["error"] = f"{type(exc).__name__}: {exc}"
                invalid.append(row)
            files[name] = case_hashes(folder/name)
            save(folder/(name+".receipt.json"),dict(row,files=files[name]))
            print(arm,name,"INVALID: "+row["error"] if "error" in row else "recorded",flush=True)
        save(folder/"results.json",dict(valid=valid,invalid=invalid,case_files=files))
        run.update(complete=True,results_sha256=sha(folder/"results.json"),
                   build_sha256=sha(folder/"firmware-build.json") if (folder/"firmware-build.json").exists() else None)
        save(folder/"run.json",run)
    provenance_error = None
    try:
        unchanged = (checkout(candidate)==pins["candidate"] and checkout(reference)==pins["release"]
                     and sha(spec_path)==spec_sha and source_hashes()==sources)
    except (OSError,ValueError,__import__('subprocess').CalledProcessError) as exc:
        unchanged = False
        provenance_error = str(exc)
    result = compare(arms,spec,primary_surface)
    if not unchanged:
        result["invalids"].append(dict(error="Input source changed during capture",detail=provenance_error))
        result["tracking_gate"] = "UNASSESSABLE"
        if result["outcome"] == "PASS":
            result["outcome"] = "INCOMPLETE"
    save(out/"comparison.json",result)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="action",required=True)
    run=sub.add_parser("run",help="Collect six arms using clean caller-supplied firmware")
    run.add_argument("--candidate",type=Path,required=True)
    run.add_argument("--reference",type=Path,required=True)
    run.add_argument("--spec",type=Path,required=True)
    run.add_argument("--out",type=Path,required=True)
    run.add_argument("--primary-surface",default="wheel0_slip")
    score=sub.add_parser("score",help="Recompute all metrics from retained public traces")
    score.add_argument("--spec",type=Path,required=True)
    score.add_argument("--arm",action="append",required=True,metavar="NAME=PATH")
    score.add_argument("--out",type=Path,required=True)
    score.add_argument("--primary-surface",default="wheel0_slip")
    args=parser.parse_args(argv)
    if args.action=="run":
        result=execute(args.candidate,args.reference,args.spec,args.out,args.primary_surface)
    else:
        arms={}
        for item in args.arm:
            name,sep,path=item.partition("=")
            if not sep or name in arms or name not in ARMS:
                parser.error("Each of the six arm names must occur exactly once")
            arms[name]=Path(path).resolve()
        spec=spec_at(args.spec)
        args.out.mkdir(parents=True,exist_ok=False)
        save(args.out/"inputs.json",dict(spec_sha256=sha(args.spec),arms={k:str(v) for k,v in arms.items()},
                                        sources_sha256=source_hashes()))
        result=compare(arms,spec,args.primary_surface)
        save(args.out/"comparison.json",result)
    print(json.dumps({k:result[k] for k in ("outcome","tracking_gate","verified_cases","expected_cases")},indent=2))
    print("failed groups",len(result["failed_tracking_groups"]),"invalid evidence",len(result["invalids"]))
    return 0 if result["outcome"]=="PASS" else 1 if result["outcome"]=="FAIL" else 2


if __name__=="__main__":
    raise SystemExit(main())
