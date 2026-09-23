# esp-everything-qgc-bridge

ESP32-S3 companion-computer firmware for CDE1302.
It consists of ESP32 firmware in `main/` and a laptop-side Python scripts in `laptop/`.

Note that the example drone ID in this README is 22. You need to change it to your drone's ID when running the scripts.

## 1. Setup ESP32 (Optional)

You can skip this if you only want to run the laptop scripts. The ESP32 firmware is already built and flashed into the drone's ESP32-S3.

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

### Build and flash a drone

For example, let drone ID = 22 and the ESP32-S3 is on `/dev/tty.usbmodem2101` (default MacBook Pro USB-C port):

`flash_drone.sh` pins `CONFIG_DRONE_ID` and `CONFIG_HOST_IPV4_ADDR`
(host IP = `192.168.1.<100 + drone id>`) into `sdkconfig`, builds, verifies the
generated header really is that drone, flashes, then restores `sdkconfig`:

```bash
./flash_drone.sh                           # prompts for drone ID and port
./flash_drone.sh 22                        # prompts for the port
./flash_drone.sh 22 /dev/tty.usbmodem2101  # non-interactive
```

Note that this requires internet to build this.

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

## 2. Setup (Laptop)

You can choose one of these following two options to install the laptop-side Python scripts and dependencies:

### Pip install (Python 3.10+)

Laptop side (Python 3.10+), with pip:

```bash
python3 -m pip install -r laptop/requirements.txt
```

or in a conda environment (below), which brings its own Python, so it also
works on a machine whose Python is too old or shared with other projects.

### Conda environment

`laptop/environment.yml` creates an environment named `cde1302-laptop` with
Python 3.12 and the packages from `laptop/requirements.txt`. Every package
installs prebuilt on 64-bit Windows, Linux with glibc 2.27+ (Ubuntu 18.04+),
and macOS 13+ (14+ on an Intel Mac); older systems fall back to compiling
OpenCV or SciPy from source.

1. Install conda once per machine. An existing Anaconda or Miniconda works;
   on a new machine install Miniforge (on Windows, see [Windows](#windows)):

   ```bash
   curl -LO "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
   bash "Miniforge3-$(uname)-$(uname -m).sh"   # answer yes when it offers to initialize conda
   ```

   Then open a new terminal. If `conda activate` later says to run
   `conda init`, run `conda init zsh` (or `bash`) and open a new terminal.

2. Create the environment, once, from the repo root:

   ```bash
   conda env create -f laptop/environment.yml
   ```

3. Activate it in every new terminal before running the scripts:

   ```bash
   conda activate cde1302-laptop
   python laptop/tag_stream.py --esp-ip 192.168.1.222   # for example
   conda deactivate                          # when done
   ```

After `laptop/requirements.txt` changes, update the environment with
`conda env update -f laptop/environment.yml`. The laptop tests run with
`cd laptop && python -m pytest tests`.

### C compiler

`tag_stream.py` compiles the drone's AprilTag detector on its first run, so
it needs a C compiler: `xcode-select --install` on macOS,
`sudo apt install build-essential` on Ubuntu. For Windows, see step 3 of
[Windows](#windows).

### Windows (Havn't tested yet)

Follow the conda steps above, with these changes:

1. **Folders:** put the repo and conda in paths with only English letters
   and no spaces, e.g. `C:\cde1302` and `C:\miniforge3`. The compiler in
   step 3 can't handle other paths.
2. **Installing conda:** run `Miniforge3-Windows-x86_64.exe` from
   <https://github.com/conda-forge/miniforge/releases/latest> and type every
   command in "Miniforge Prompt" from the Start menu ("Anaconda Prompt" with
   Anaconda). Don't use WSL: its default networking never passes the drones'
   packets on.
3. **C compiler:** after creating the environment, install MinGW-w64 gcc into
   it (about 110 MB; Visual Studio's compiler won't work) and check the build:

   ```bat
   conda activate cde1302-laptop
   conda install -c conda-forge gcc
   python laptop/apriltag_host.py
   ```

   It should print `tag16h5 ncodes = 22 (expected 22)`.
4. **`python`, not `python3`:** conda's Python on Windows has no `python3`
   command, so type `python` wherever this README says `python3`.
5. **Firewall:** the drones send telemetry and video to the laptop unprompted,
   so allow Python through Windows Defender Firewall when it asks, and set
   the drone Wi-Fi to a Private network. It asks once per environment; if you
   clicked Cancel, remove the block rule under "Allow an app through
   firewall".

### Laptop IP address

The drones send MAVLink, telemetry and ToF debug data to
`192.168.1.(100 + drone ID)` (set when flashing), so the QGroundControl or
mission laptop must hold that address on the drone Wi-Fi. Camera video follows
the source IP of the latest `camera_stream.py` or `tag_stream.py` keepalive and
can therefore be viewed from another laptop on the same Wi-Fi. The small
AprilTag debug stream is also copied to that viewer when `tag_stream.py` uses
`--detail`.

## 3. ESP32 camera preview

View a drone's camera over WiFi (drone IP = `192.168.1.(200 + drone ID)`):

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

Only one camera destination is active at a time: if viewers on two laptops
send keepalives, the most recent keepalive wins. The Wi-Fi access point must
allow client-to-client traffic, and the viewer firewall must allow Python to
receive UDP 5008 and 5009. MAVLink remains on `CONFIG_HOST_IPV4_ADDR` while a
different laptop is viewing the camera.

## 4. Simple arming and takeoff (Lab 5)

Check the communication without sending flight commands.
```bash
python3 laptop/send_trajectory.py --drone-id [DRONE_ID] --takeoff --monitor-only
```

Takeoff -> hover at 0.5 m altitude -> land. The altitude is 0.5 m by default, but you better check the parameter `CRUISE_ALT_M` in the flight controller.
```bash
python3 laptop/send_trajectory.py --drone-id [DRONE_ID] --takeoff
```
Type `ARM-[drone ID]` when prompted.

## 5. Send a waypoint mission (Lab 5)

```bash
python3 laptop/simple_waypoint_mission.py \
  --drone-id [DRONE_ID] \
  --waypoints-file waypoints/waypoints_example.txt
```

## 4. Fly from a CSV trajectory

```bash
python3 laptop/send_trajectory.py --drone-id [DRONE_ID] --takeoff --trajectory trajectory/circle_traj.csv
```

See `trajectory/README.md` to generate the CSV and for the space it needs.
