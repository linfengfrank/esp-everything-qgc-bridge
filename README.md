After cloning the repository, make sure to clone the MAVLink C library into the project by:  

```bash
cd components/mavlink/include  
git clone --depth 1 https://github.com/mavlink/c_library_v2.git .
```

## QGroundControl MAVLink WiFi Bridge

This version includes a MAVLink UDP bridge for QGroundControl:

```text
QGroundControl ↔ UDP 14550/8888 ↔ ESP32-S3 ↔ UART MAVLink ↔ PX4
```

See `QGC_MAVLINK_BRIDGE.md` for setup and debugging notes.
