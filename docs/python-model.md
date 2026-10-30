# Legacy Python model

Select `--backend python` or construct `VirtualBoard(commands_path)` to use the original behavioral model without a C compiler. It reads the schema, but implements behavior separately from firmware. `--backend python --check` fails if the schema includes commands that this model does not implement.

## What is modeled

| Area | Behavior |
| --- | --- |
| Wire contract | Schema-driven IDs, directions, little-endian field types, response objects, ranges, finite doubles, boolean values, topic enums, correlated ACK/ERROR |
| Session | INIT identity/version checks, READY, uptime/wall-clock handshake, periodic sync requests, PING, heartbeat expiry and reconnect gating |
| Motors | Initialization, PWM, velocity controller state, stop/brake, encoder mapping, position origin/targets and lifecycle |
| Encoders | Counts, odometry start/reset/stop, initial sample availability, radians and timestamps |
| Platforms | Differential, omni, mecanum ownership, ideal wheel kinematics, body velocity/world pose, position targets, controller cancellation |
| I/O and sensors | GPIO state, status LED, board-specific sensor presence, startup settling, configurable temperature/IMU samples |
| Telemetry | Typed topics plus legacy odometry events, subscription prerequisites/rates, unsubscribe, message-ID-zero events, polling |
| Failures | Invalid frames/arguments, unavailable resources, injected controller errors, missing replies, watchdog loss |

The behavioral model originally covered protocol 2.6.0. With the current schema, it reports seven unsupported commands: motor current, motor faults/clear, current-limit get/set, SUBSCRIBE_WITH_MODE, and SUBSCRIBE_CONFIGURED. Use the native firmware backend for these features. Handler coverage does not establish firmware equivalence.

## Deterministic tests and injected failures

`VirtualBoard.feed(bytes)` accepts arbitrary fragments or multiple frames.
`drain()` returns outgoing bytes. `advance(seconds)` drives all model time.
`VirtualSerial` wraps this with the pyserial methods used by the SDK, a lock, read
timeouts, cancellation, and configurable read fragmentation. It advances real
time by default; pass `realtime=False` for deterministic tests. Use its `advance`
method while a reader thread is active, rather than calling the board directly.

```python
board = VirtualBoard(commands_path)
port = VirtualSerial(board, realtime=False, fragment_size=1)

# After connecting the SDK through this port:
board.fail_next("SET_MOTOR_SPEED", "INTERNAL_ERROR")  # No state change
board.drop_next_reply("SET_MOTOR_SPEED")             # Executes, ACK is lost
board.sensor_errors["temperature", 0] = "INTERNAL_ERROR"
board.temperature = 32.5
port.advance(0.6)  # Can expire a 500 ms watchdog immediately
```

Fault configuration should be set while requests are idle. The `VirtualBoard`
core itself is intentionally single-threaded. Disable heartbeat explicitly when
advancing several simulated seconds to test motion; retain it when testing loss.
The [SDK example](../examples/python_sdk.py) now uses native firmware. The legacy model remains covered by `tests/test_board.py` and parts of `tests/test_sdks.py`.

## Fidelity limits

This is an SDK contract and lifecycle simulator, not an execution of compiled C
firmware. Continue running firmware host tests and hardware tests alongside it.

- Velocity follows targets instantly. Open-loop PWM maps linearly to a configurable
  maximum of 10 rad/s. Position uses a bounded proportional model; PID commands
  validate/store I/D settings, but I/D dynamics, noise, inertia, load, electrical
  behavior, and real controller tuning are not simulated.
- Motion uses logical configured directions, assuming motor/encoder reversal
  flags correctly compensate wiring. It cannot diagnose reversed wiring. Platform
  wheel geometry is modeled, but traction, collisions, slip, and load are not.
- Integration uses at most 10 ms substeps. Frequencies are accepted/quantized and
  constrain telemetry, but do not reproduce RTOS scheduling or PID sampling.
  Samples use current model time; sensor/odometry cache cadence is simplified.
- Wall-clock synchronization uses one ideal exchange with no drift. Firmware's
  multi-sample RTT filter, retries, stale-clock transitions, and sync failure
  timing are not modeled. Use firmware session tests for those algorithms.
- Telemetry emits at most one latest sample per subscription per `advance`, with
  no historical catch-up. Slow-consumer backpressure and firmware queue limits
  are not reproduced. Drain output regularly in raw tests.
- Each connection owns a whole simulated board. Shared physical-board state
  across multiple transports, I2C arbitration, USB enumeration, and serial line
  timing are outside this model. Native COM/TTY modes keep one board per process,
  retaining it across SDK reconnects. `POLL_TELEMETRY` tests framing, not the I2C driver.

## Extending the contract

Change the firmware schema and regenerate the SDKs normally. The emulator reads
that schema at startup; no generation step is needed here. Add behavior to
`board.py` for new commands/topics, including explicit membership in `SUPPORTED`,
then add an SDK scenario and error/lifecycle tests. The schema describes wire
structure; behavioral semantics still require implementation and review against
firmware. Keep firmware reference tests as the independent source of truth to
avoid a simulator and SDK agreeing on the same mistake.
