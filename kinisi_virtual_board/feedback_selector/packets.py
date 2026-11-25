"""Export four coherent native examples in the staged clients' raw-payload format."""
import json
import hashlib
import struct
from kinisi_virtual_board.schema import Schema


def read_packets(path):
    for line in path.read_text(encoding="utf8").splitlines():
        row = json.loads(line)
        if row.get("role") or row["event"] != "wire":
            continue
        tx, rx = bytes.fromhex(row["tx"]), bytes.fromhex(row["rx"])
        assert tx[0] + 1 == len(tx)
        while rx:
            n = rx[0]+1
            assert 4 <= n <= len(rx)
            frame, rx = rx[:n], rx[n:]
            if frame[1:4] == tx[1:4]:
                yield (row["boot"],row["time_us"]), tx, frame, row["command"]


def extract(schema, folder, case, source, name, reason=0):
    groups, mounts = {}, []
    for moment, tx, frame, cmd in read_packets(folder/(case+".jsonl")):
        if cmd == "GET_FEEDBACK_FUSION_MOUNT":
            mounts.append((moment,frame))
        if cmd in ("GET_FEEDBACK_SOURCE_STATUS","GET_FEEDBACK_SOURCE_VECTOR"):
            groups.setdefault(moment,[]).append((cmd,tx,frame))
    choices = []
    for moment, rows in groups.items():
        for i,(cmd,_,first) in enumerate(rows):
            if cmd != "GET_FEEDBACK_SOURCE_STATUS":
                continue
            fields = schema.objects["feedback_source_status"]
            st = dict(zip((f["name"] for f in fields),schema.codec(fields).unpack(first[4:]),strict=True))
            req,active,valid,why,seq,stamp,origin,age = (st[k] for k in
                ("requested","active","valid","reason","sequence","timestamp_us","origin_id","age_us"))
            if req != source or why != reason:
                continue
            if reason:
                assert (active,valid,seq,stamp,age) == (255,0,0,0,0xffffffff)
                return dict(name=name,status=first[4:].hex(),pose="",twist="",mount="",
                            boot=moment[0],wall_us=moment[1],case=case)
            assert active == req and valid and age == moment[1]-stamp
            vectors = {}
            for cmd2,tx,frame in rows[i+1:]:
                if cmd2 == "GET_FEEDBACK_SOURCE_STATUS":
                    if set(vectors) == {0,1} and first[4:] == frame[4:]:
                        samples = [struct.unpack("<Iddd",vectors[k][4:]) for k in (0,1)]
                        assert all(s[0] == seq for s in samples)
                        mount = next((f[4:] for t,f in reversed(mounts) if t[0]==moment[0] and t<=moment), b"")
                        if source == 2:
                            assert mount and mount[0] == 1
                        choices.append((sum(abs(v) for s in samples for v in s[1:]), dict(
                            name=name,status=first[4:].hex(),pose=vectors[0][4:].hex(),twist=vectors[1][4:].hex(),
                            mount=mount.hex(),boot=moment[0],wall_us=moment[1],case=case)))
                    break
                kind, key_origin, key_seq = struct.unpack("<BII",tx[4:])
                if key_origin == origin and key_seq == seq:
                    vectors[kind] = frame
    if not choices or max(v for v,_ in choices) == 0:
        raise ValueError(f"No bracketed moving {name} snapshot")
    return max(choices,key=lambda x:x[0])[1]


def export(firmware, capture):
    """Write four SDK payload examples from a passing, unchanged native capture."""
    summary = json.loads((capture/"summary.json").read_bytes())
    assert not summary.get("changed_inputs") and all(summary["cases"][c]["result"] == "PASS" for c in ("encoder","imu","fused"))
    schema = Schema(firmware/"commands.json")
    for case in ("encoder", "imu", "fused"):
        raw = (capture/(case+".jsonl")).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == summary["cases"][case]["sha256"]
        boot = json.loads(raw.splitlines()[0])
        assert schema.sha256 == boot["build"]["schema_sha256"]
    records = [extract(schema,capture,*x) for x in (
        ("encoder",0,"encoder"),("imu",1,"imu"),("imu",1,"imu-expired",5),("fused",2,"fused"))]
    with (capture/"sdk-records.json").open("x",encoding="utf8") as stream:
        json.dump(records,stream,indent=2)
