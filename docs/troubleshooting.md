## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Kinect #0 not found (0 connected ...)` and `lsusb` shows only `02b0` | The camera and audio are powered by the **12 V adapter**: with USB power alone only the motor enumerates. Reseat the orange Kinect connector and check the adapter. Camera and audio appear in `journalctl -k -f` as soon as 12 V arrives. |
| `freenect_open_device failed` | Another process already owns the camera (another driver instance, a `learn/` example, `freenect-glview`, ...), or the user is not in `plugdev` / the udev rule is missing. |
| Topic listed but `echo` or another tool receives nothing | QoS mismatch: the subscriber is RELIABLE and the publisher is BEST_EFFORT. Use `--qos-reliability best_effort` or a sensor-data QoS. |
| `parameter 'depth_mode' has invalid type ... bool` | Launch files parse arguments as YAML, where `off` means `false`. Fixed in `camera.launch.py` (`value_type=str`). Keep this in mind when you pass `on`, `off`, `yes` or `no` as strings. |
| Zero-copy check reports `FAIL` | Check the `Intra-process communication: enabled` log line, and that consumers are in the **same container**, use `ConstSharedPtr` and compatible QoS (`KEEP_LAST`, `VOLATILE`). |
