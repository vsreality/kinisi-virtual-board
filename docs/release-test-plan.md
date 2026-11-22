# Controller firmware release test plan

Use this plan for each controller firmware release. Run the common checks on each candidate, then the cases for its Agora stories. When a story becomes supported, keep its automated regression in subsequent releases. The current release is `r_e71aea9bb26d`. Use virtual v0.4.0 for the common suite and the original v0.3.0 and Nano v0.3.1 images for their profile-specific cases. Physical qualification is deferred for this run.

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

| Case and Agora item | Procedure | Expected result |
| --- | --- | --- |
| Save and restore on v0.4 · `i_0c637931409e` | Start with blank FRAM. Configure setup A with known geometry, encoder resolution, direction flags and a supported nondefault PWM frequency. Save. Change settings to B without saving, disconnect and reconnect to the same running board, then cold-boot with A's FRAM. Repeat after explicitly saving B. Exercise Save from the web client and SDKs. | Blank boot allows manual setup. Reconnect shows B. First cold boot restores A, second restores B, without host reinitialization. Match fields through readback, including `GET_PWM_FREQUENCY`. Outputs remain stopped until a fresh motion command. |
| Persistence recovery · same story | During A-to-B replacement, interrupt writes before commit and around the commit boundary. Cold-boot after each cut. Also test unreadable storage, corrupt and incompatible records, then a good save after recovery. Attempt Save on v0.3. | Readback is the last complete committed record. No mixed record or false success. With no valid record the board starts safely unconfigured. v0.3 reports unsupported and its running configuration survives. |
| Sleep and wake · `i_8ab8525655b7` | Run `python -m unittest discover -s tests -p test_driver_sleep.py -v` on protocol 2.15. Confirm the five sleep cases execute and the native manifest lists `hw_motor.c` and `hw_drv8873.c`. Start stopped and moving in separate virtual cases. Request sleep and observe outputs, driver state, command responses, supported wake source and restored settings. Repeat cycles and inject a failed transition. | Transitions, supported commands and failure recovery match the approved story. Outputs stay stopped on wake until a new command. Report software-visible behavior here. Actual current draw needs a bench measurement. |
| DRV8870 sleep on original v0.3 and Nano, `i_3788e3c6057f` | Against protocol 2.17 run `python -m unittest discover -s tests -p test_drv8870_sleep.py -v`. Build each native image with `python tools/build_native.py --revision <revision> --out build/<revision>`, using `0.3.0` and `0.3.1`. | Verify the INIT identity, each regular timer channel and GPIO alternate function. Compare low/low timer coast with GPIO low/low on all eight inputs through sleep, reconnect and release. Confirm at least 2 ms of requested low dwell. On USB and I²C reject sleeping, queued and fragmented drive or brake with error 19. Fresh drive or short brake must affect only the addressed motor or platform wheels. Coast must return both inputs low. Record the simulator build IDs alongside results. Physical waveforms, electrical savings and first-drive latency need bench measurements. |
| Board idle (v0.4), `i_153acabd4598` | Run `python -m unittest discover -s tests -p test_board_idle.py -v` on protocol 2.16. Then run `python tests/test_board_idle.py --measure` and retain its JSON output with the build manifest. | Verify entry and exit with both prior driver policies, failed transitions and retry, reconnect, heartbeat expiry, time sync, queued commands, fresh telemetry, encoder continuity and v0.3 rejection. Compare one second with four initialized sleeping drivers, temperature and IMU subscriptions at 250 ms, and 20 PING plus 20 POLL commands in each mode. Require 40 ACKs in each, positive sensor reads and events at baseline, zero periodic reads and events in idle. Hardware power, interrupt behavior and wake latency need separate bench evidence. |
| Shallow board idle on original v0.3 and Nano, `i_df295fb22ad4` | On protocol 2.18 run `python -m unittest discover -s tests -p test_drv8870_idle.py -v`. In the same firmware checkout run `python test/test_usb_vbus/run_tests.py` and `python test/test_configuration/run_tests.py`. Retain both logs, manifests and firmware source revision or fingerprint. Use the 2.17 checkout to check the earlier unsupported contract. | Enter from active motor and platform control. Confirm idle on the same initialized USB session, exchange PING and clock status, and retain the configured clock interval after explicit exit. Check both prior driver policies, repeated commands, failed entry/exit and retry, and blocked USB and I2C queues with fragmented commands straddling wake. All eight modeled inputs stay low on successful exit until fresh actuation. Check Nano motor code never touches PA9 and that the firmware Nano fixture retains USB attachment through idle and handles a subsequent VBUS loss and renewed handshake. |
| Fused and IMU-only tracking · `i_9d6028c11aa8`, `i_03fc803954c7` | Replay identical timestamped stationary, straight, turning and slip trajectories for encoder-only, IMU-only and fused modes. Inject IMU bias and sample loss. Compare estimates and closed-loop response against an independent reference trajectory available only to the scorer. | Report pose and velocity errors, stationary drift, latency and control tracking against the agreed story limits for each mode. Record fixture, units and observation window. If limits are open, retain measurements and leave acceptance pending. |
| Tracking source selection · `i_0c388c609d20` | Select each supported mode. Switch with fresh and missing sensor data, then reconnect and reset. Exercise unsupported-board behavior. | Requested and active modes, transition behavior and fallback/error match the approved contract. Invalid or stale estimates are observable. |
| Custom platform geometry · `i_46691a2bffb0` | Compare editable presets with equivalent custom definitions. Supply an independent non-preset layout with a passive encoder wheel. Exercise unsupported motion and degenerate geometry. | Match wheel order, signs, motion constraints and expected targets computed independently from the input geometry. Passive wheels receive no drive target. |
| Platform diagrams · `i_f485c1a25fbb` | Using only the diagram and worked example, configure differential, three-wheel omni, four-wheel mecanum and custom layouts in the web client. Compare the values shown in documentation, entered in the client and read back from the virtual board. | Units, distance endpoints, angle references, handedness, motor/encoder indices and direction agree. Capture each diagram and its completed form. |

## Record results and decide

For each run create a dated report alongside the release evidence and link it from Agora. Record the candidate revisions, commands, board profile and transport, manifest, logs, per-case outcomes and reproduction steps. For example:

| Case | Candidate / profile | Expected | Observed | Outcome | Evidence / follow-up |
| --- | --- | --- | --- | --- | --- |
| Persistence after restart | firmware SHA, simulator SHA, v0.4 | Setup A restored, PWM stopped | Enter actual readback | Pass / Fail / Blocked / Deferred | Log path, test name and any remaining check |

An applicable case passes only when the expected behavior is observed on the candidate. Record skips individually with their reason. A required check skipped due to the host, missing tools or an unimplemented simulator hook stays outstanding. Preserve original failures, identify the fix and rerun affected cases and the common suite on the new candidate. The PO reviews required results and declared deferrals before accepting the software release. Carry physical FRAM, electrical power and real-sensor accuracy checks as separate hardware evidence wherever applicable.
