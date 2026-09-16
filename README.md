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
./flash_drone.sh                           # prompts for drone ID and port
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
## ESP32 camera preview

The onboard QVGA grayscale camera can be viewed over WiFi without changing
the AprilTag detector's camera format. The viewer requests the stream on
demand; firmware JPEG-compresses completed detector frames and sends them as
MTU-safe UDP chunks on port 5009.

Install the laptop dependencies, reflash this firmware, and run:

```bash
python3 -m pip install -r laptop/requirements.txt
python3 laptop/camera_stream.py --esp-ip 192.168.1.222
```

This example targets drone 22; replace the final octet using the mapping
below. The laptop must also be the address configured by
`CONFIG_HOST_IPV4_ADDR`. Press `q` or Escape to exit and `s` to save a frame.
The stream is capped at 2 fps to limit its effect on navigation and stops
automatically when viewer keepalives cease.

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
`255.255.255.0`, and its DHCP pool must exclude `192.168.1.200` through
`192.168.1.230` (or reserve those addresses for the drones).
