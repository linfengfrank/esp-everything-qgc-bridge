# esp-everything-qgc-bridge

ESP32-S3 companion-computer firmware for CDE1302.
It consists of ESP32 firmware in `main/` and a laptop-side Python scripts in `laptop/`.

## Setup

Everything this firmware needs is committed in-tree (MAVLink `c_library_v2`,
esp-apriltag, the VL53L5CX driver and `managed_components/`). The only external
dependency is ESP-IDF itself:

```bash
git clone https://github.com/espressif/esp-idf.git ../esp-idf
cd ../esp-idf && git checkout "$(sed -n 's/^IDF_CHECKOUT_REF=//p' ../esp-everything-qgc-bridge/tools/idf-pin.env)"
git submodule update --init --recursive
./install.sh esp32s3          # ~2 GB of toolchain into ~/.espressif
cd -
```

See `tools/idf-pin.env` for the pinned version and why it matters.

Laptop side (Python 3.10+):

```bash
python3 -m pip install -r laptop/requirements.txt
```

## Build and flash a drone

For example, let drone ID = 22 and the ESP32-S3 is on `/dev/tty.usbmodem2101` (default MacBook Pro USB-C port):

`flash_drone.sh` pins `CONFIG_DRONE_ID` and `CONFIG_HOST_IPV4_ADDR`
(host IP = `192.168.1.<100 + drone id>`) into `sdkconfig`, builds, verifies the
generated header really is that drone, flashes, then restores `sdkconfig`:

```bash
./flash_drone.sh                           # prompts for drone ID and port
./flash_drone.sh 22                        # prompts for the port
./flash_drone.sh 22 /dev/tty.usbmodem2101  # non-interactive
```

It finds ESP-IDF automatically (`$IDF_PATH`, `./esp-idf`, `../esp-idf`,
`~/esp/esp-idf`), sourcing `export.sh` only if needed.

### Deterministic drone addresses

`flash_drone.sh` asks for the drone ID when no ID argument is supplied and
builds the firmware with this mapping:

```text
drone IPv4 address = 192.168.1.(200 + drone ID)
```

For example, drone 9 is `192.168.1.209` and drone 22 is
`192.168.1.222`. This is separate from `CONFIG_HOST_IPV4_ADDR`: the flash
script retains the existing QGC/laptop mapping `192.168.1.(100 + drone ID)`.
After flashing, it prints the exact `camera_stream.py --esp-ip ...` command.
The WiFi router must use gateway `192.168.1.1` with netmask
`255.255.255.0`.

## QGroundControl MAVLink WiFi Bridge

This version includes a MAVLink UDP bridge for QGroundControl:

```text
QGroundControl ↔ UDP 14550/8888 ↔ ESP32-S3 ↔ UART MAVLink ↔ PX4
```

See `QGC_MAVLINK_BRIDGE.md` for setup and debugging notes.
## ESP32 camera preview

View a drone's camera over WiFi (drone IP = `192.168.1.(200 + drone ID)`;
the laptop must be `CONFIG_HOST_IPV4_ADDR`):

```bash
python3 laptop/camera_stream.py --esp-ip 192.168.1.222 [--fps 10] [--quality 60]
```

`s` saves a frame.

With ApriTags drawn on the frame:
```bash
python3 laptop/tag_stream.py --esp-ip 192.168.1.222 [--detail]
```

`--detail`, or `d` while it runs, adds the tuning layers.

For both commands, `q`/Esc quits. The drone streams only while the viewer
runs (~9 fps, ~130 ms from capture to the laptop on the OV3660). The overlay
and the console show the drone-side frame age and dropped frames.

## Fly a CSV trajectory

```bash
python3 laptop/send_trajectory.py --drone-id 22 --takeoff --trajectory trajectory/circle_traj.csv
```

See `trajectory/README.md` to generate the CSV and for the space it needs.
