# Legacy v0.4 configuration images

These two 8,192-byte FRAM dumps are inputs for testing a newer firmware reader. Both came from clean Kinisi firmware `34909771ad4d440da474d8f6ed49e90f3d8bf943`, protocol 2.20.0, running through the virtual board's native C backend at `79879935dcf593443281593890d3763fce858285`. The backend ran the firmware's configuration capture, journal and restoration code.

| Image | Released state | SHA-256 |
|---|---|---|
| [factory.fram](factory.fram) | All bytes erased to 0xff. No platform is initialized. Saving at factory defaults returns `PLATFORM_NOT_INITIALIZED`. | `7d2c7ac4888bfd75cd5f56e8d61f69595121183afc81556c876732fd3782c62f` |
| [mecanum.fram](mecanum.fram) | Public `SAVE_CONFIGURATION` committed slot 0, generation 1, format 1, 72-byte payload for model 1 revision 0.4.0. Slot 1 is erased. | `d492e8f032c427407fb232d459b65d0fa23d13ffebef2c7026764bc4de66231e` |

## Use with a firmware checkout

From the virtual-board repository, point `KINISI_FIRMWARE` at the checkout under test. The tests load the saved bytes into fresh native runtimes. Keep both images fixed when testing a post-change reader.

```powershell
$env:KINISI_FIRMWARE = (Resolve-Path ../kinisi-motor-controller-firmware).Path
python -B tests/test_configuration.py -v
```

```sh
KINISI_FIRMWARE=../kinisi-motor-controller-firmware python3 -B tests/test_configuration.py -v
```

Append `-k legacy` for just the three fixture checks. The saved preset can also be restored in its own process with `python tests/test_configuration.py --cold-worker boot tests/fixtures/configuration-v1/mecanum.fram`. To reproduce the preset bytes, use the released checkout and run the same worker with `save` and a **new scratch path**. The factory image is `board.fram_image` on a newly created `FirmwareBoard` with the released checkout and no image supplied.

## Observed release behavior

* **Factory boot.** Public configuration type was 0 and PWM frequency was 100 kHz. Geometry reads returned `PLATFORM_NOT_INITIALIZED`.
* **Preset boot.** The public readback was mecanum, wheel diameter 0.12 m, length 0.48 m, width 0.31 m and PWM frequency 25 kHz. Encoder resolutions were 4096/2000/2000/2000 counts per revolution, encoder reversals 1/1/1/0 and motor reversals 1/0/1/0. The new process restored the preset at time zero after the saving process had driven the motors. All four outputs were zero.
* **Boot and reconnect.** Both idle images kept zero PWM. Odometry required an explicit start. Volatile fusion mode read back as 0 (legacy encoders) and IMU-only state as 0 (stopped). Factory boot used two FRAM reads and preset boot used three. Neither wrote bytes. A fresh handshake on each live runtime added zero FRAM transactions.
* **Active encoder tracking.** With the saved preset, the test explicitly started ordinary odometry and control, supplied an unavailable IMU, and drove the virtual chassis. At 240 ms, public heading was −0.0433076563 rad, agreeing with heading reconstructed from the four signed raw encoder counters and saved scales. Disconnect during motion immediately zeroed all four PWM outputs. The new session could read the same timestamped pose and PWM stayed zero for the next 200 ms. Reconnect preserves live odometry across the session boundary.
* **Unsupported boards.** v0.3.0 and v0.3.1 Nano both returned `SENSOR_NOT_AVAILABLE` for configuration read/save, fusion configuration and IMU-only status. Neither accessed FRAM or changed the supplied v0.4 image. The firmware configuration suite also exercised Nano VBUS loss and renewed handshake using its native USB harness.

The native build IDs below were observed on Windows with MSYS2 GCC 16.1.0. Other host toolchains can produce different build IDs. This baseline covers simulated firmware behavior ahead of source-selector migration. When the selector lands, extend the fixture checks with its new public requested-source, active-source and validity readbacks.

| Profile | Native build ID |
|---|---|
| v0.4.0 | `cf94d0baa1190ea4cdab2a24d95b418c7069533902d40a8ccc47ef0a6a4d67b9` |
| v0.3.0 | `c29859c5b6a19fba3197e41a4dbb4abe17d4932dc2743bef891ac43af976860e` |
| v0.3.1 Nano | `312e085a8eae3d33292117b56e7cabb54674a930267ffb34d875ef07fcccea5d` |
