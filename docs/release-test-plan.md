# Controller firmware release test plan

Use this plan for each controller firmware release. Run the common checks on each candidate, then the cases for its Agora stories. When a story becomes supported, keep its automated regression in subsequent releases. The current release is `r_e71aea9bb26d`. Use virtual v0.4.0 for the common suite and the original v0.3.0 and Nano v0.3.1 images for their profile-specific cases. Physical qualification is deferred for this run.

## Feedback source selector regression

From a clean simulator Git checkout, supply a clean firmware Git checkout with
the feedback selector commands and run:

```sh
python -m kinisi_virtual_board selector-regression --firmware /path/to/firmware --out build/selector-run
```

Use Python 3.10+, Git and a host GCC or Clang on Windows or Linux. Windows can use
MSYS2 UCRT64 GCC. Select another compiler with `--compiler PATH` or `CC`. The
native in-process regression requires no COM driver, SDK installation, MSVC,
administrator access or browser. Firmware and simulator repositories can live
anywhere; neither a shared workspace layout nor a mutable staging checkout is
required. The firmware must include its required source dependencies. The output directory can be inside the simulator checkout (for example
`--out results/selector`) or elsewhere. Only that newly created output tree is
excluded from the closing source-identity check; all other tracked and untracked
source changes remain guarded. Existing output is never overwritten.

The command runs these retained public cases without drawing new schedules:

| Case | Required observations |
| --- | --- |
| `factory` | Released factory v1 FRAM boots safely with encoder default. |
| `migration` | Released mecanum v1 geometry survives migration; selected source persists, transient tracking/mount confirmation does not; failed saves retain the committed record. |
| `v03`, `nano` | Unsupported source/persistence operations reject without storage changes or motion. |
| `encoder` | Selected pose/twist match native wheel odometry, have a fresh origin and coherent capture keys, and preserve reconnect behavior. |
| `switch_safety` | Same-source selection is a no-op. Armed, moving and coasting switches are refused; a real switch requires disarmed and quiet state, never arms, and rejects old capture keys. New encoder origin waits for a publication after the switch, including 1 Hz and 7 Hz odometry. |
| `imu` | Selected vectors match the native IMU producer; confirmation, expiry, frozen input, explicit restart and no silent encoder fallback are observable. |
| `fused` | Passive fusion requires mounting confirmation; vectors match an independent native release-producer instance under the same sensor input; loss and recovery remain visible. |

The two packaged v1 FRAM images are byte-identical to the documented
[public pre-selector fixtures](../tests/fixtures/configuration-v1/README.md).
The IMU cases retain public seed 131 and a 10 ms delivery delay. They drive
modeled sensor readings and external encoder counters, never estimator state.
The fused comparison uses a second native instance; it establishes protocol and
producer consistency, not independent localization accuracy. The IMU/fused
telemetry scenarios require zero motor PWM throughout. Encoder and switch
scenarios intentionally exercise motion and verify stopping separately.

`summary.json` records simulator and firmware revisions, harness/fixture hashes,
each outcome and the SHA-256 of its raw `<case>.jsonl`. Boot records include native
build manifests, board identity and FRAM hashes. Wire records preserve request
and reply bytes, virtual time, encoder counters and PWM. A full successful run
also writes `sdk-records.json`: coherent encoder, IMU and fused snapshots plus
an expired IMU status, in raw payload form for SDK tests. The exporter verifies
capture hashes and schema identities before extracting examples. These records
are examples to replay in SDK tests, not proof that a specific SDK was tested.

Exit 0 means every selected scenario and required export passed. Exit 1 retains
a failed or incomplete scenario/export. Exit 2 means setup or input-integrity
verification could not complete. Exceptions and partial raw logs remain in the
new output directory. `--case NAME` can be repeated for focused checks; a subset
is not evidence for all eight cases. Packet export requires passing `encoder`,
`imu` and `fused` cases. `--development` explicitly permits and fingerprints dirty
source checkouts; use clean pinned checkouts for release evidence. Python `-O`
is rejected because scenario assertions are part of the checks.

The separately verified optional browser path uses the production
`kinisi-serial-proxy` with its serial opening bound to the simulator's
`VirtualSerial`, entirely over loopback. It requires Python `pyserial` and
`websockets`, the matching built JavaScript SDK/web client (Node/npm for its
build), and Edge. Automated UI checks additionally require Python Playwright
and its Edge channel. This loopback path needs no Windows COM driver. It uses
the web client's **Remote proxy**, lists the simulated port and connects it.

That integration harness is currently the workspace-only
`releases/feedback-source-selector/board-demo/web_session.py`, alongside the
retained browser evidence; it is not shipped by this simulator repository and
is not run by `selector-regression`. In that workspace its entry points are
`python web_session.py --firmware PATH --site BUILT_SITE` and the same command
with `--check NEW_OUTPUT` for automated checks. The workspace must provide the
proxy checkout and the harness's documented layout. The portable native core
above has none of those dependencies. A separate direct Web Serial workflow
uses a PTY or the installed Windows COM driver and is not the verified loopback
browser path described here.

For browser acceptance record the exact served JS bundle and SDK/firmware pins,
UI actions, acknowledged wire writes, status readback and PWM. Exercise source
selections, IMU setup, fused mount confirmation and expiry. Keep browser
acceptance distinct from native and exported-packet checks, and keep large run
artifacts outside Git.

## Matched encoder/IMU fusion control

Use the same native simulator to run default, encoder-feedback and fused-feedback modes on the reference and candidate firmware. From the simulator checkout, supply both firmware checkouts and the public development workload:

```sh
python -B tools/fusion_comparison run --reference /path/to/reference-firmware --candidate /path/to/candidate-firmware --spec tools/fusion_comparison/development.json --out /path/to/new-comparison
```

This command replays historical tracking measurements and their original numerical verdicts. The current release criterion is communication of source health and disagreement. Use the native matrix below for that decision.

### Dual-source tracking health

From the simulator root, pass a clean v0.4 firmware checkout with the protocol 2.22 health commands and a new output directory. The regular user account needs Python and a host GCC or Clang compiler. The command works offline. If the compiler is outside PATH, append `--compiler /path/to/gcc`.

```sh
python -B -m kinisi_virtual_board tracking-health --firmware /path/to/firmware --out /path/to/results/health-01
```

The board runs native firmware. A public BMI088 model (seed 131) and externally imposed wheel counts exercise differential, omni and mecanum motion. `summary.json` identifies the two source checkouts, the workload, observed outcomes and failures. Each JSONL trace retains the wire bytes, board time, sensor inputs, shaft counts and motor outputs. Run the same command with a new output directory to repeat. Exit codes are 0 for pass, 1 for an observed failure and 2 for incomplete evidence or setup error.

Inspect encoder, IMU and fused status alongside the selected estimate. The cases exercise aligned straight travel and turns, an IMU I/O error, stale data, saturation, disabled encoder odometry, a brief yaw discrepancy, persistent conflicting counts, and recovery. Level constant-speed translation and rest can produce identical inertial signals. A stationary counter alone cannot identify a broken encoder. The motion interlock case drives an explicit target, verifies coasting before any diagnostic read, and requires a new target after recovery. A separate case checks physical IMU availability when its one-second tracking estimate expires. Older board profiles exercise the native unsupported responses.

For an integrated release, replay the captured health packets through the matching JavaScript, Python and Arduino SDKs and verify the operator's live browser readout. The native command requires only the simulator and firmware repositories. The browser path additionally requires a built matching web client, the local serial proxy, and a browser. Record the client revisions alongside the native run. Hardware measurements stay separate.


## Public IMU regression

For ordinary v0.4 IMU development and release checks, run one command from the
virtual-board checkout. Supply a firmware source checkout with the IMU tracking
commands, Python 3.10+ and host GCC or Clang. Use `CC` to select the C compiler.

```sh
python -m kinisi_virtual_board imu-regression --firmware ../kinisi-motor-controller-firmware --seed 8101
```

Keep the JSON output (redirect stdout to a file if desired), the simulator
revision, and the firmware revision or source changes. The committed motion
profiles and seed reproduce the sensor inputs. The default seed is 8101.
The command builds and runs the firmware locally in process, with no COM port,
SDK installation, account-specific fixtures, network service or approval step.

Six public cases exercise rest, a rapid forward start, braking, reversal, a
translating turn with yaw-rate reversal, and stale IMU samples. A seeded BMI088
model converts known motion into independently timestamped accelerometer and gyro
readings. The native firmware receives the sensor readings and normal operator
commands. A separate comparator checks its observable pose and twist throughout
the one-second tracking window against the closed-form reference at source time.
The cases also check expiry and the expected response to stale data.

| Peak absolute error | Limit |
|---|---|
| Planar position vector | 0.050 m |
| Body velocity vector | 0.020 m/s |
| Relative yaw | 2 degrees |
| Yaw rate | 0.020 rad/s |

Exit **0** means every planned case passed. Exit **1** means a measured product
check failed. Exit **2** means the check could not complete, for example because
the firmware could not build or expected observations were missing. The JSON
counts preserve incomplete cases even when a measured failure sets the exit code.

## Full public IMU release qualification

Use the six cases above for quick feedback. For release acceptance, select the
firmware checkout and run the 144-case chassis qualification from this repository:

```sh
python -m kinisi_virtual_board imu-qualify --firmware ../kinisi-motor-controller-firmware
```

The command builds the selected candidate, then draws three fresh seeds and
writes the complete public plan before simulation. Its default output directory
is `build/imu-qualification/<timestamp>`; `--out PATH` selects a fresh directory.
Each result is saved as it finishes. `report.json` counts pending, passed,
failed and incomplete cases. Exit codes are 0, 1 and 2 as above.

The 135 healthy attempts cross differential, omni and mecanum chassis, five
motions (stationary, accelerate, brake, reverse and translating turn), three
seeds and 10/13/17 ms delivery. Nine paired stationary cases exercise producer
freeze, accelerometer freeze and gyro reordering. A motion-producing native
board drives the chassis model; the separate tracking board sees its timestamped
sensor readings. `--motion-firmware PATH` selects an independent checkout for
the motion producer. The scorer checks actual chassis movement, all 100 expected
source-time estimates per healthy second, the four numerical limits above,
35 ms first-publication source age, 50 ms public latency and expiry allowance,
100 ms fault response, explicit stationary restart and zero diagnostic motor PWM.

Archive the run directory. To test the same plan and recorded motion inputs
against another checkout, copy the archive if needed and run:

```sh
python -m kinisi_virtual_board imu-qualify --firmware ../kinisi-motor-controller-firmware --replay path/to/archived-run
```

Replay uses the saved motion sources and writes a new result directory. An
unrealized motion or missing observation stays visible. The release procedure
runs entirely in the local workspace. The earlier confidential study retains
its historical verdict. MCU timing, physical accuracy, long-duration tracking
and motion-control feedback require separate evidence.

## Prepare the run

1. Record the release ID and included stories, date, host, toolchain versions, board profiles, protocol version, and revisions of firmware, virtual board, JavaScript/web client, Python and Arduino. Capture working-tree status for each. Keep a patch or equivalent source snapshot if testing uncommitted work.
2. Place those five checkouts alongside one another. Install their documented test dependencies. For Windows native builds use Python 3.10+, host GCC, MSVC and the Windows SDK. Set `KINISI_MSVC` to the installed MSVC version directory if using Visual Studio Build Tools. Build the embedded firmware with PlatformIO separately.
3. Start each independent case with fresh simulated storage. Retain the same simulated FRAM contents across cold boots *within* a persistence case. Record seed, simulated elapsed time, fixtures and fault injection points when used. Use only virtual endpoints for this run.

## Build and run the common suite

Run the following in PowerShell from the virtual-board repository. Keep the manifest and logs from the same candidate. Stop if a build fails so that an old executable cannot supply the result.

```powershell
$env:KINISI_FIRMWARE = (Resolve-Path ../kinisi-motor-controller-firmware).Path
$env:KINISI_COMMANDS = Join-Path $env:KINISI_FIRMWARE 'commands.json'
$env:KINISI_PYSDK = (Resolve-Path ../pykinisi).Path
$env:KINISI_JSSDK = (Resolve-Path ../jskinisi).Path
$env:KINISI_REQUIRE_NATIVE = '1'
$env:KINISI_REQUIRE_SDKS = '1'
$runDir = Join-Path (Get-Location) ('build/release-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
python tools/build_native.py --firmware $env:KINISI_FIRMWARE --revision 0.4.0 --out $runDir
if ($LASTEXITCODE) { throw 'Virtual board build failed' }
$env:KINISI_NATIVE_BINARY = (Resolve-Path (Join-Path $runDir 'kinisi-sim.exe')).Path
python -m unittest discover -s tests -v *> (Join-Path $runDir 'suite.log')
$suiteExit = $LASTEXITCODE
Get-Content (Join-Path $runDir 'suite.log') -Tail 12
if ($suiteExit) { throw 'Inspect suite.log for failures' }
```

On Linux use the same firmware checkout, schema and SDK variables, build with `python3 tools/build_native.py --revision 0.4.0 --out build/native-linux`, set `KINISI_NATIVE_BINARY` to the resulting `kinisi-sim`, and run the same unittest discovery. Linux supports a real virtual serial PTY. For Windows WebSerial use the simulator's printed COM port on a host with the virtual driver already configured. For direct firmware tests the in-process native harness works without a COM port. See [standalone setup](standalone.md) and [firmware harness and fault injection](native-firmware.md). A TCP reconnect can create a new board instance. Use a transport or harness with explicit board identity for reconnect and persistence checks.

| Check | Run | Pass evidence |
| --- | --- | --- |
| Firmware and schema | In the firmware checkout run `pio run -e v0_4` and the affected firmware host tests. For a protocol/profile change run the Kinisi sync audit, `python <kinisi-feature-sync>/scripts/check_sync.py --workspace <workspace>`. | Embedded build succeeds, generated commands and board profiles match across SDKs, and affected firmware tests pass. |
| Virtual-board regression | Run the common suite above against the release candidate. Inspect all skips and errors. | Production handler tests cover INIT/READY, commands, malformed input, telemetry, watchdog, safe stop, reconnect and profile differences. Identify which SDK tests exercised the native firmware backend. |
| SDKs and web client | In `pykinisi` run `python -m unittest discover -s tests`. In `jskinisi` run `npm test` and `npm run build`. In `ArduinoKinisi` run `python tests/run_tests.py`. | Record each exit status. Exercise every changed public operation through its normal interface. Distinguish Arduino host fixtures from a serial integration and browser component tests from an actual web-client session. |
| User workflow and compatibility | Connect to the candidate virtual v0.4, configure via the web client or SDK, read back and reconnect. Repeat the new operation against an unsupported profile and, when applicable, older firmware. | Supported operations return the expected values. Unsupported operations report unavailability and preserve existing configuration. Save a transcript or screenshot for changed UI behavior. |

## Current release cases

Execute each case when its story implementation is available. Record the concrete test name or manual steps, expected values and observed values in the run report. A missing simulator capability is a gap to implement and rerun.

For the v0.4 persistence cases, run `python -m unittest discover -s tests -p test_configuration.py -v` from this repository. Check the individual case names and count, including the separate-process cold boot. An empty discovery run gives no evidence. Complete the SDK and web-client workflows as well.

For tracking-source migration (`i_0c388c609d20`), run those configuration tests with the candidate firmware and the [retained v1 FRAM images](../tests/fixtures/configuration-v1/README.md). They cover erased storage, a released mecanum preset, default encoder odometry, reconnect during motion, and unsupported v0.3 and Nano responses.

| Case and Agora item | Procedure | Expected result |
| --- | --- | --- |
| Save and restore on v0.4 · `i_0c637931409e` | Start with blank FRAM. Configure setup A with known geometry, encoder resolution, direction flags and a supported nondefault PWM frequency. Save. Change settings to B without saving, disconnect and reconnect to the same running board, then cold-boot with A's FRAM. Repeat after explicitly saving B. Exercise Save from the web client and SDKs. | Blank boot allows manual setup. Reconnect shows B. First cold boot restores A, second restores B, without host reinitialization. Match fields through readback, including `GET_PWM_FREQUENCY`. Outputs remain stopped until a fresh motion command. |
| Persistence recovery · same story | During A-to-B replacement, interrupt writes before commit and around the commit boundary. Cold-boot after each cut. Also test unreadable storage, corrupt and incompatible records, then a good save after recovery. Attempt Save on v0.3. | Readback is the last complete committed record. No mixed record or false success. With no valid record the board starts safely unconfigured. v0.3 reports unsupported and its running configuration survives. |
| Sleep and wake · `i_8ab8525655b7` | Run `python -m unittest discover -s tests -p test_driver_sleep.py -v` on protocol 2.15. Confirm the five sleep cases execute and the native manifest lists `hw_motor.c` and `hw_drv8873.c`. Start stopped and moving in separate virtual cases. Request sleep and observe outputs, driver state, command responses, supported wake source and restored settings. Repeat cycles and inject a failed transition. | Transitions, supported commands and failure recovery match the approved story. Outputs stay stopped on wake until a new command. Report software-visible behavior here. Actual current draw needs a bench measurement. |
| DRV8870 sleep on original v0.3 and Nano, `i_3788e3c6057f` | Against protocol 2.17 run `python -m unittest discover -s tests -p test_drv8870_sleep.py -v`. Build each native image with `python tools/build_native.py --revision <revision> --out build/<revision>`, using `0.3.0` and `0.3.1`. | Verify the INIT identity, each regular timer channel and GPIO alternate function. Compare low/low timer coast with GPIO low/low on all eight inputs through sleep, reconnect and release. Confirm at least 2 ms of requested low dwell. On USB and I²C reject sleeping, queued and fragmented drive or brake with error 19. Fresh drive or short brake must affect only the addressed motor or platform wheels. Coast must return both inputs low. Record the simulator build IDs alongside results. Physical waveforms, electrical savings and first-drive latency need bench measurements. |
| Board idle (v0.4), `i_153acabd4598` | Run `python -m unittest discover -s tests -p test_board_idle.py -v` on protocol 2.16. Then run `python tests/test_board_idle.py --measure` and retain its JSON output with the build manifest. | Verify entry and exit with both prior driver policies, failed transitions and retry, reconnect, heartbeat expiry, time sync, queued commands, fresh telemetry, encoder continuity and v0.3 rejection. Compare one second with four initialized sleeping drivers, temperature and IMU subscriptions at 250 ms, and 20 PING plus 20 POLL commands in each mode. Require 40 ACKs in each, positive sensor reads and events at baseline, zero periodic reads and events in idle. Hardware power, interrupt behavior and wake latency need separate bench evidence. |
| Shallow board idle on original v0.3 and Nano, `i_df295fb22ad4` | On protocol 2.18 run `python -m unittest discover -s tests -p test_drv8870_idle.py -v`. In the same firmware checkout run `python test/test_usb_vbus/run_tests.py` and `python test/test_configuration/run_tests.py`. Retain both logs, manifests and firmware source revision or fingerprint. Use the 2.17 checkout to check the earlier unsupported contract. | Enter from active motor and platform control. Confirm idle on the same initialized USB session, exchange PING and clock status, and retain the configured clock interval after explicit exit. Check both prior driver policies, repeated commands, failed entry/exit and retry, and blocked USB and I2C queues with fragmented commands straddling wake. All eight modeled inputs stay low on successful exit until fresh actuation. Check Nano motor code never touches PA9 and that the firmware Nano fixture retains USB attachment through idle and handles a subsequent VBUS loss and renewed handshake. |
| IMU-only tracking · `i_03fc803954c7` | Run the [full public IMU release qualification](#full-public-imu-release-qualification) on the selected candidate. | Retain the public plan, all 144 outcomes, native build IDs, measured coverage, numerical peaks and fault results. |
| Encoder and fused tracking · `i_9d6028c11aa8` | Replay matched stationary, straight, turning and slip trajectories for encoder-only and fused modes. Inject IMU bias and sample loss. Compare estimates and closed-loop response against independent reference motion. | Report pose and velocity errors, stationary drift, latency and control tracking against each mode's story limits. Record fixture, units and observation window. |
| Tracking source selection · `i_0c388c609d20` | Select each supported mode. Switch with fresh and missing sensor data, then reconnect and reset. Exercise unsupported-board behavior. | Requested and active modes, transition behavior and fallback/error match the approved contract. Invalid or stale estimates are observable. |
| Custom platform geometry · `i_46691a2bffb0` | Compare editable presets with equivalent custom definitions. Supply an independent non-preset layout with a passive encoder wheel. Exercise unsupported motion and degenerate geometry. | Match wheel order, signs, motion constraints and expected targets computed independently from the input geometry. Passive wheels receive no drive target. |
| Platform diagrams · `i_f485c1a25fbb` | Using only the diagram and worked example, configure differential, three-wheel omni, four-wheel mecanum and custom layouts in the web client. Compare the values shown in documentation, entered in the client and read back from the virtual board. | Units, distance endpoints, angle references, handedness, motor/encoder indices and direction agree. Capture each diagram and its completed form. |

## Record results and decide

For each run create a dated report alongside the release evidence and link it from Agora. Record the candidate revisions, commands, board profile and transport, manifest, logs, per-case outcomes and reproduction steps. For example:

| Case | Candidate / profile | Expected | Observed | Outcome | Evidence / follow-up |
| --- | --- | --- | --- | --- | --- |
| Persistence after restart | firmware SHA, simulator SHA, v0.4 | Setup A restored, PWM stopped | Enter actual readback | Pass / Fail / Blocked / Deferred | Log path, test name and any remaining check |

An applicable case passes only when the expected behavior is observed on the candidate. Record skips individually with their reason. A required check skipped due to the host, missing tools or an unimplemented simulator hook stays outstanding. Preserve original failures, identify the fix and rerun affected cases and the common suite on the new candidate. The PO reviews required results and declared deferrals before accepting the software release. Carry physical FRAM, electrical power and real-sensor accuracy checks as separate hardware evidence wherever applicable.
