"""Exercise the public selector on a clean native firmware checkout; retain wire evidence."""
import hashlib
import importlib
import json
import subprocess
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIM = HERE.parent.parent
from kinisi_virtual_board import FirmwareBoard
from kinisi_virtual_board.imu_regression_capture import Wire, CommandError

CASES = {
    "factory": ("persistence", "factory", "0.4.0", "factory"),
    "migration": ("persistence", "persistence", "0.4.0", "mecanum"),
    "v03": ("persistence", "unsupported", "0.3.0", "mecanum"),
    "nano": ("persistence", "unsupported", "0.3.1", "mecanum"),
    "encoder": ("wheel", "encoder", "0.4.0", "mecanum"),
    "switch_safety": ("wheel", "switch_safety", "0.4.0", "mecanum"),
    "imu": ("inertial", "imu", "0.4.0", "mecanum"),
    "fused": ("inertial", "fused", "0.4.0", "mecanum"),
}


def pin(repo, development=False, output=None):
    # Only a newly allocated output tree may be excluded, never other sources.
    paths = ["--", "."]
    if output is not None and output.is_relative_to(repo):
        paths.append(":(top,literal,exclude)" + output.relative_to(repo).as_posix())
    def git(*args):
        try:
            return subprocess.check_output(["git", "-C", str(repo), *args], text=True, stderr=subprocess.STDOUT).strip()
        except (OSError, subprocess.CalledProcessError) as exc:
            raise ValueError(f"Cannot identify Git source checkout {repo}: {exc}") from exc
    revision = git("rev-parse", "HEAD")
    if not git("status", "--porcelain", "--untracked-files=all", *paths):
        return revision
    if not development:
        raise ValueError(f"Dirty checkout: {repo}")
    patch = subprocess.check_output(["git","-C",str(repo),"diff","--binary","HEAD",*paths])
    digest = hashlib.sha256(patch)
    for path in sorted(git("ls-files","--others","--exclude-standard",*paths).splitlines()):
        digest.update(path.encode()+b"\0"+hashlib.sha256((repo/path).read_bytes()).digest())
    return revision+"+development."+digest.hexdigest()


class Session:
    def __init__(self, firmware, revision, stream, compiler=None):
        self.firmware, self.revision, self.stream = firmware, revision, stream
        self.board = None
        self.compiler = compiler
        self.boot = 0

    def close(self):
        if self.board is not None:
            self.board.close()
            self.board = None

    def mark(self, label, **fields):
        record = dict(event=label, boot=self.boot, time_us=self.board.now_us,
                      pwm=self.board.motor_pwm, counts=self.board.encoder_counts, **fields)
        self.stream.write(json.dumps(record, allow_nan=False) + "\n")
        self.stream.flush()

    # Wire owns protocol validation; these two methods retain the actual byte streams.
    def feed(self, frame):
        self.request = frame
        self.board.feed(frame)

    def drain(self):
        reply = self.board.drain()
        self.mark("wire", command=self.schema.by_code[self.request[1]]["command"],
                  tx=self.request.hex(), rx=reply.hex())
        return reply

    def ask(self, name, **values):
        raw = self.wire.ask(name, **values)
        fields = self.schema.response_fields(self.schema.commands[name])
        decoded = self.schema.codec(fields).unpack(raw)
        return dict(zip((f["name"] for f in fields), decoded, strict=True))

    def error(self, name, error, **values):
        try:
            self.ask(name, **values)
        except CommandError as exc:
            if exc.code == self.schema.errors[error]:
                return
            raise AssertionError(f"{name}: expected {error}, got {exc}") from exc
        raise AssertionError(f"{name}: expected {error}, got success")

    def status(self):
        return self.ask("GET_FEEDBACK_SOURCE_STATUS")

    def initialize(self):
        self.wire = Wire(self)
        return self.ask("INIT", sdk_type=1, protocol_major=self.schema.version[0],
                        protocol_minor=self.schema.version[1], client_capabilities=2)

    def reboot(self, image):
        self.close()
        self.boot += 1
        self.board = FirmwareBoard(firmware=self.firmware, revision=self.revision, fram_image=image, compiler=self.compiler)
        self.schema = self.board.schema
        self.mark("boot", fram_sha256=hashlib.sha256(image).hexdigest(), build=self.board.build,
                  ready=self.board.ready)
        assert not self.board.ready and not any(self.board.motor_pwm)
        result = self.initialize()
        assert tuple(result["board_"+key] for key in ("major","minor","patch")) == tuple(map(int,self.revision.split(".")))


def run(args):
    """Run selected scenarios and preserve raw packets, including failed cases."""
    if not __debug__:
        raise ValueError("Use Python with assertions enabled")
    firmware = args.firmware.resolve()
    args.out = args.out.resolve()
    if args.out.exists():
        raise FileExistsError(f"Output already exists: {args.out}")
    rev, sim = pin(firmware, args.development, args.out), pin(SIM, args.development, args.out)
    chosen = args.cases or list(CASES)
    if len(set(chosen)) != len(chosen):
        raise ValueError("Each case may be selected only once")
    args.out.mkdir(parents=True, exist_ok=False)
    files = sorted(HERE.glob("*.py")) + sorted((HERE/"fixtures").glob("*.fram"))
    def hashes():
        return {p.relative_to(HERE).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    result = dict(firmware=rev, simulator=sim, scripts=hashes(), cases={})
    def save():
        (args.out/"summary.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf8")
    save()
    for case in chosen:
        module, fn, revision, fixture = CASES[case]
        with (args.out/(case+".jsonl")).open("x", encoding="utf8") as stream:
            session = Session(firmware, revision, stream, args.compiler)
            try:
                img = (HERE/"fixtures"/(fixture+".fram")).read_bytes()
                session.reboot(img)
                getattr(importlib.import_module("."+module, __package__), fn)(session)
                outcome = dict(result="PASS")
            except Exception as exc:
                outcome = dict(result="FAIL_OR_INCOMPLETE", error=f"{type(exc).__name__}: {exc}",
                               trace=traceback.format_exc())
            finally:
                session.close()
        outcome["sha256"] = hashlib.sha256((args.out/(case+".jsonl")).read_bytes()).hexdigest()
        result["cases"][case] = outcome
        print(case, outcome["result"], outcome.get("error",""), flush=True)
        save()
    if pin(firmware, args.development, args.out) != rev or pin(SIM, args.development, args.out) != sim or hashes() != result["scripts"]:
        result["changed_inputs"] = True
        save()
        return 2
    needed = ("encoder", "imu", "fused")
    if all(result["cases"].get(c, {}).get("result") == "PASS" for c in needed):
        from .packets import export
        try:
            export(firmware, args.out)
            result["packet_export"] = dict(result="PASS", file="sdk-records.json",
                sha256=hashlib.sha256((args.out/"sdk-records.json").read_bytes()).hexdigest())
        except Exception as exc:
            result["packet_export"] = dict(result="FAIL_OR_INCOMPLETE", error=f"{type(exc).__name__}: {exc}")
    else:
        result["packet_export"] = dict(result="NOT_RUN", reason="Requires passing encoder, imu and fused scenarios")
    save()
    return int(any(v["result"] != "PASS" for v in result["cases"].values()) or
               result["packet_export"]["result"] == "FAIL_OR_INCOMPLETE")
