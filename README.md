# esp-everything-qgc-bridge

ESP32-S3 companion-computer firmware for the SAFMC 2026 Cat Swarm Challenge,
plus the laptop-side fleet coordinator in `laptop/`.

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

`flash_drone.sh` pins `CONFIG_DRONE_ID` and `CONFIG_HOST_IPV4_ADDR`
(host IP = `192.168.1.<100 + drone id>`) into `sdkconfig`, builds, verifies the
generated header really is that drone, flashes, then restores `sdkconfig`:

```bash
./flash_drone.sh 22                        # prompts for the port
./flash_drone.sh 22 /dev/tty.usbmodem2101  # non-interactive
```

It finds ESP-IDF automatically (`$IDF_PATH`, `./esp-idf`, `../esp-idf`,
`~/esp/esp-idf`), sourcing `export.sh` only if needed.

## QGroundControl MAVLink WiFi Bridge

This version includes a MAVLink UDP bridge for QGroundControl:

```text
QGroundControl ↔ UDP 14550/8888 ↔ ESP32-S3 ↔ UART MAVLink ↔ PX4
```

See `QGC_MAVLINK_BRIDGE.md` for setup and debugging notes.
