# Released Mini chip-reference mount

`chip-mount.fram` is an 8192-byte image saved by native firmware `0c7d42baf540f5fa3fcf1abd014fc48aa7a36874`, protocol 2.23.0. Starting with erased v0.4 storage, the operator sent INIT, SET_IMU_MOUNT with angles (9000, 0, 0) centidegrees and chip coordinates (42000, −51000, 17000) µm, then SAVE_CONFIGURATION. GET_IMU_MOUNT returned those six fields with configured=1. The image is the resulting virtual FRAM, including the release's original journal and checksum.

On protocol 2.24 the same installation reads back underside center (57373, −48925, −9953) µm. The expected center uses the Mini's physical dimensions and +90° roll. Run `python -m unittest discover -s tests -p test_configuration.py -v` against a firmware checkout with PCB-center support to exercise a cold boot and another boot after saving.
