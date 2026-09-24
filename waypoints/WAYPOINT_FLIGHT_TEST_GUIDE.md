# Waypoint Flight Test Guide

This guide describes how to conduct a supervised waypoint flight test using
`simple_waypoint_mission.py`.  
The script commands the drone to arm, take off, fly through a sequence of
map-frame NED waypoints, and land — using the custom UDP protocol already
implemented in this project.

> **Prerequisite:** Complete the arm / hover test documented in
> `TEST_GUIDE.md` (sections 1–7) before attempting a waypoint flight.
> The drone must have demonstrated stable hover and clean landing on its own
> before adding waypoints.

---

## Protocol summary

| Command | Value | Action |
|---------|-------|--------|
| `CMD_START` | `0x05` | Arm + take off to 0.5 m AGL |
| `CMD_GOTO`  | `0x01` | Fly to NED map-frame (x, y) |
| `CMD_LAND`  | `0x02` | Land in place |

Waypoints use the **NED map frame**: x = North, y = East, in metres relative
to the PX4 odometry origin (takeoff point).

---

## Current network configuration

```text
Wi-Fi SSID : Mate20
Laptop IP  : 192.168.43.106
Drone ID   : 2
Telemetry  : ESP32 → laptop  UDP 5005
Commands   : laptop → ESP32  UDP 5006
```

Verify the laptop IP before every session — a phone hotspot may re-assign the
address after reconnecting.

```powershell
ipconfig
```

The active Wi-Fi address must match `CONFIG_HOST_IPV4_ADDR` in `sdkconfig`.
If it does not match, update via `idf.py menuconfig`, rebuild, and reflash.

---

## Step 1 — Prepare the Python environment

Open a terminal inside `laptop/`:

```bash
cd esp-everything-qgc-bridge/laptop
python -m pip install pyyaml
```

Confirm the required files are present:

```text
laptop/
├── comms.py
├── protocol.py
├── setup.yaml
├── simple_waypoint_mission.py
└── waypoints_example.txt        ← edit this or create your own
```

---

## Step 2 — Define waypoints

Waypoints are NED map-frame positions in metres (x = North, y = East).  
The origin is the PX4 odometry origin at the moment of takeoff.

### Option A — inline on the command line

```bash
python simple_waypoint_mission.py \
  --drone-id 2 \
  --waypoint 0.5,0.0 \
  --waypoint 0.5,0.5 \
  --waypoint 0.0,0.0
```

### Option B — waypoint file

Edit `waypoints_example.txt` (or create a new file).  
Each line is `x,y`. Blank lines and lines starting with `#` are ignored.

```text
# Example — small square route, NED frame (x=north, y=east)
0.5,0.0
0.5,0.5
0.0,0.5
0.0,0.0
```

Run with:

```bash
python simple_waypoint_mission.py \
  --drone-id 2 \
  --waypoints-file waypoints_example.txt
```

---

## Step 3 — Verify the ESP32 firmware

The ESP32 must run the standard firmware and must **not** have the
`send_setpoint()` call disabled.  
The 20 Hz Offboard setpoint stream in `main/mavlink_task.c` must remain active
— the Python GCS relies on the ESP32 to maintain Offboard mode with PX4.

---

## Step 4 — Start hardware and monitoring

Recommended power-on order:

1. Remove propellers for a desk/bench test; refit them only in a clear test area.
2. Power PX4 and ESP32.
3. Connect the laptop and the ESP32 to `Mate20`.
4. Start the ESP32 serial monitor (separate terminal):

```bash
idf.py monitor
```

5. Open QGroundControl for **observation only** — do not arm or change flight
   modes through QGC during the Python test.

Expected ESP32 startup messages:

```text
Telemetry valid
Pre-streaming hold setpoint for 2 s...
Waiting for CMD_START from laptop...
Telemetry -> 192.168.43.106:5005 | Commands <- port 5006
```

---

## Step 5 — Communication-only check

Run the monitor to confirm telemetry is flowing before sending any flight
commands:

```bash
python simple_waypoint_mission.py \
  --drone-id 2 \
  --waypoint 0.5,0.0 \
  --waypoint 0.5,0.5 \
  --waypoint 0.0,0.0
```

> The script waits up to 30 seconds for the first telemetry packet.  
> If no packet arrives, it exits with a `TimeoutError`.

Do not proceed if:

- no telemetry is received within 30 seconds;
- packet age repeatedly exceeds 1 second;
- the drone ID in the packet is not `2`;
- the ESP32 serial log shows Wi-Fi or PX4 errors.

---

## Step 6 — Desk test (propellers removed)

With propellers removed, run the mission with `--confirm` to require an
explicit go-ahead:

```bash
python simple_waypoint_mission.py \
  --drone-id 2 \
  --waypoint 0.5,0.0 \
  --waypoint 0.5,0.5 \
  --waypoint 0.0,0.0 \
  --confirm
```

The script prompts:

```text
Type MISSION-2 to start the mission:
```

Type `MISSION-2` and press Enter.

Expected ESP32 log sequence:

```text
CMD_START received — checking ToF sensors...
All ... ToF sensors OK — proceeding to arm
Requesting OFFBOARD mode...
OFFBOARD mode confirmed
Arming...
Armed confirmed
Taking off to 0.5 m AGL...
```

Because the propellers are removed, this verifies command routing and state
machine flow only.  
Disarm the vehicle via the RC or PX4 safety switch before handling it.

---

## Step 7 — Supervised waypoint flight test

Conduct this only in a clear, approved test area with a human safety pilot
holding a manual recovery method (RC transmitter in Stabilise/Altitude mode).

### Recommended first-flight parameters

```bash
python simple_waypoint_mission.py \
  --drone-id 2 \
  --waypoints-file waypoints_example.txt \
  --takeoff-wait 5.0 \
  --arrival-timeout 30.0 \
  --confirm
```

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `--takeoff-wait` | 3.0 s | Stabilisation time after arming before the first `CMD_GOTO` is sent |
| `--arrival-timeout` | 25.0 s | Maximum time to wait for `NAV_ARRIVED` per waypoint before advancing |
| `--confirm` | off | Require `MISSION-<id>` confirmation before arming |

### Full flight sequence

1. Script waits up to 30 s for telemetry; prints `Telemetry link established.`
2. Prompt appears: type `MISSION-2` to continue.
3. Script sends `CMD_START` → ESP32 arms and climbs to 0.5 m AGL.
4. After `--takeoff-wait` seconds, script sends `CMD_GOTO` for waypoint 1/N.
5. Script polls telemetry for `NAV_ARRIVED`; logs position when reached.
6. Repeats steps 4–5 for every remaining waypoint.
7. Script sends `CMD_LAND`; drone descends and disarms.

### What to observe in QGroundControl

- Flight mode switches to **Offboard**.
- Vehicle **arms** and climbs to ~0.5 m.
- Drone tracks each waypoint in sequence; horizontal drift should be < 0.3 m.
- After the last waypoint, drone **descends** and **disarms**.

### Expected Python log

```text
10:05:01 | INFO | Waiting for telemetry from drone 2...
10:05:02 | INFO | Telemetry link established.
Type MISSION-2 to start the mission: MISSION-2
10:05:05 | INFO | Sending CMD_START (arm + takeoff)
10:05:10 | INFO | Sending waypoint 1/3 to (0.50, 0.00)
10:05:17 | INFO | Waypoint reached at (0.50, 0.01)
10:05:17 | INFO | Sending waypoint 2/3 to (0.50, 0.50)
10:05:24 | INFO | Waypoint reached at (0.49, 0.50)
10:05:24 | INFO | Sending waypoint 3/3 to (0.00, 0.00)
10:05:31 | INFO | Waypoint reached at (0.01, 0.01)
10:05:31 | INFO | Sending CMD_LAND
```

### Expected final ESP32 messages

```text
CMD_LAND
CMD_LAND received from laptop
LAND command sent
Disarmed — mission complete
```

---

## Step 8 — Waypoint arrival timeout

If a waypoint is not reached within `--arrival-timeout` seconds, the script
logs a warning and advances to the next waypoint:

```text
WARNING | Waypoint arrival timeout; last nav_state=NAVIGATING at (0.31, 0.12)
```

The drone continues navigating but the GCS moves on.  
If timeouts occur repeatedly, investigate:

- VFH blocking (obstacles or sensor miscalibration);
- `NAV_CRUISE_SPEED_MS` too low for the waypoint spacing;
- `NAV_ARRIVE_RADIUS_M` too tight (default 0.25 m).

---

## Step 9 — Emergency stop

Press **Ctrl+C** at any time after `CMD_START`.  
The script catches the interrupt and attempts to send `CMD_LAND` before
closing.

```text
^C
10:05:20 | INFO | Interrupted — sending CMD_LAND
10:05:20 | INFO | CMD_LAND sent to drone 2.
```

This is a software fallback only.  
The safety pilot must be ready to take manual control at any moment.

---

## Step 10 — Repeating the test

After landing, `mission_task` on the ESP32 deletes itself and cannot restart
without a power cycle.

Before running a second mission:

1. Confirm the drone has touched down and disarmed.
2. Stop the Python script (`Ctrl+C`).
3. Power-cycle or reset the ESP32.
4. Wait for `Waiting for CMD_START from laptop...` in the serial monitor.
5. Restart the Python script.

---

## Telemetry limitations

The custom telemetry packet contains:

- horizontal NED position (x, y)
- heading
- navigation state (`NAV_ARRIVED`, `NAVIGATING`, `IDLE`, …)
- VFH blocked bins
- AprilTag sightings
- breadcrumb batch

It does **not** contain altitude, armed state, PX4 flight mode, battery
status, or command acknowledgements.  
Use the ESP32 serial monitor and QGroundControl to observe those states
throughout the test.
