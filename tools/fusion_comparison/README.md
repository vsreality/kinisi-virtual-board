# Public fusion control comparison

Run default, encoder-feedback and fused-feedback control on both a reference
firmware revision and a candidate. The same native board, motor dynamics,
external chassis, delayed BMI088 samples, geometry, targets and seeds apply to
all six arms. Firmware owns the control loop and motor PWM. Truth and grip enter
the external plant and scorer, never the firmware controller.

From the virtual-board repository root:

```powershell
python -B tools/fusion_comparison run --reference C:/src/reference-firmware --candidate C:/src/candidate-firmware --spec tools/fusion_comparison/development.json --out C:/results/fusion-comparison-01
python -B -m unittest discover -s tools/fusion_comparison -p "test_*.py"
```

Use clean firmware Git checkouts with the public fusion experiment commands and
the native build prerequisites described in the repository README. The output
directory must not exist and must be outside the firmware checkouts. Previous
results are never replaced. Failed cases retain their error and any partial
traces. No qualification generator, entropy source, private fixture, sibling
release workspace or archived runner is imported. Supply an already materialized
public spec; geometry, units, seed ranges, ordered targets, disturbance intervals
and sample grids are validated. The bundled spec and verification runs use known
development inputs. This tool cannot generate, bind or authorize a new workload;
any future study must materialize and authorize its own public spec first.
`--primary-surface wheel1_slip` supports the separate three-seed public protocol.
There is no random draw or private-fixture option.

The result is `comparison.json`. It includes each primary metric, endpoint and
heading diagnostics, source health, raw-data parity, paired values, means,
favorable pairs and nominal allowances. `outcome` preserves independently
observed failures even when evidence is also incomplete. `tracking_gate` is
`UNASSESSABLE` when evidence is missing/invalid or the matrix lacks any of the
three presets, either course, nominal/declared disturbance, or three distinct
seed pairs. A proper subset of the bundled historic development workload is also
always incomplete, including omission of its supplementary common-slip surface.
A separately materialized three-seed public matrix may be assessed in full.
Exit codes are 0 PASS, 1 FAIL, 2 INCOMPLETE; non-comparison infrastructure errors
also exit nonzero and remain visible in the terminal. A development PASS is not
qualification or physical-hardware acceptance.

The official primary gates remain separate by preset/course: the declared disturbance
requires at least 10% lower mean tracking RMSE with a strict majority of favorable
seed pairs; nominal allows the larger of 5% of the matching encoder mean or twice
the original nominal per-run sample standard deviation. Common slip is reported
as supplementary stress. Unchanged default/encoder traces across revisions and
healthy encoder/fused estimates are also required. Additional release/default and
release/fused comparisons are reported independently. No bad interval is removed.

`nominal_dispersion.json` contains only six sample deviations and their derivation
from the original public 90-case baseline summary, whose SHA-256 is
`a3270391e63c4fc4e29c58deae08932da2260c23e091ac588a88decbc4a72ce3`.
The compact artifact is independently pinned and contains no workspace paths.
This preserves the original acceptance allowance without an ignored `build/`
directory dependency. `development.json` contains only the public development
part of the repository's encoder baseline case file.

To re-score retained development traces, pass all six arm folders explicitly:

```powershell
python -B tools/fusion_comparison score --spec tools/fusion_comparison/development.json --out C:/results/rescore-01 --arm release-default=C:/archive/release-release --arm release-encoder=C:/archive/release-encoder --arm release-fused=C:/archive/release-fused --arm candidate-default=C:/archive/candidate-release --arm candidate-encoder=C:/archive/candidate-encoder --arm candidate-fused=C:/archive/candidate-fused
```

This accepts the public evidence-v2 `run.json`/`results.json` layout as well as
new runs. It verifies original hashes, identity, complete sample grids, actual
targets, sensor clocks, PWM bounds and 100 ms dispatch cadence. It recomputes all
native scores and independently recomputes primary tracking RMSE from raw truth
at the registered timestamps. Changed or missing records remain invalid.

Callable interfaces (with this directory and its parent on Python's import path)
are `capture.collect(directory, firmware, spec, preset, course, surface, seed,
variant)`, `score.score_case(directory, spec, key, mode)`,
`score.compare(arms, spec, primary_surface="wheel0_slip")` and
`core.spec_at(path)`. Modes in arm names are
`default`, `encoder`, `fused`; the capture adapter uses `release` for default.
