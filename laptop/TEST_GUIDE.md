# Arm, takeoff, hover and landing test

This test is based on the attached `esp-everything-qgc-bridge(1)` project.
It uses the existing custom Python GCS protocol, not direct Pymavlink.

## Exact behavior in this project

The command definitions are in `laptop/protocol.py` and `main/wifi_task.h`:

- `CMD_START = 0x05`
- `CMD_HOLD = 0x03`
- `CMD_LAND = 0x02`

The flight sequence is implemented in `main/main.c`:

1. Wait for valid PX4 telemetry.
2. Pre-stream a hold setpoint for two seconds.
3. Wait for `CMD_START` from the laptop.
4. Require all ToF sensors to be initialized.
5. Request PX4 Offboard mode.
6. Arm PX4.
7. Take off to `CRUISE_ALT_M = 0.5 m`.
8. Wait for laptop navigation or landing commands.
9. On `CMD_LAND`, send `MAV_CMD_NAV_LAND`.

Therefore, the existing protocol does not provide separate ARM and TAKEOFF
commands. `CMD_START` performs both.

## Current network configuration

The attached `sdkconfig` contains:

```text
Wi-Fi SSID: Mate20
Laptop IP: 192.168.43.106
Drone ID: 2
Custom telemetry: ESP32 -> laptop UDP 5005
Custom commands: laptop -> ESP32 UDP 5006
QGroundControl telemetry: ESP32 -> laptop UDP 14550
QGroundControl commands: laptop -> ESP32 UDP 8888
```

Verify the laptop IP before testing because a phone hotspot may assign a new
address after reconnecting.

## 1. Copy the script

Copy:

```text
simple_arm_takeoff_land.py
```

into the project directory:

```text
esp-everything-qgc-bridge/laptop/
```

The result should be:

```text
laptop/
├── comms.py
├── protocol.py
├── setup.yaml
└── simple_arm_takeoff_land.py
```

## 2. Prepare Python

Open a terminal in `laptop/`:

```bash
cd esp-everything-qgc-bridge/laptop
python -m pip install pyyaml
```

On Windows, `py` may be used instead of `python`.

## 3. Verify the ESP32 firmware mode

Use the normal firmware from the attached project. The ESP32 must continue to
send the 20 Hz Offboard setpoint stream in `main/mavlink_task.c`.

Do not enable a modified direct-laptop-MAVLink mode that disables
`send_setpoint()`, because the existing custom Python GCS depends on the ESP32
for Offboard setpoint streaming.

## 4. Start the hardware and monitoring tools

Recommended order:

1. Remove propellers for the first bench test.
2. Power PX4 and ESP32.
3. Connect the laptop and ESP32 to `Mate20`.
4. Start the ESP32 serial monitor:

```bash
idf.py monitor
```

5. Open QGroundControl for observation.
6. Do not use QGroundControl to arm or change modes during the Python test.

Expected ESP32 startup messages include:

```text
Telemetry valid
Pre-streaming hold setpoint for 2 s...
Waiting for CMD_START from laptop...
Telemetry -> 192.168.43.106:5005 | Commands <- port 5006
```

## 5. Quick PX4-ESP32 link check

Before running the mission script, run this connectivity check from the project
root:

```bash
python laptop/check_px4_esp32_link.py
```

Or, if your terminal is already in `laptop/`:

```bash
python check_px4_esp32_link.py
```

## 6. Communication-only test

Run:

```bash
python simple_arm_takeoff_land.py --drone-id 2 --monitor-only
```

Expected Python output:

```text
Waiting for drone 2 telemetry on UDP 5005...
Connected: drone=2, ESP32_IP=..., map=(...), nav=IDLE
MONITOR | remaining ... | map=(...) | nav=IDLE | age=...
```

Do not proceed if:

- no telemetry is received;
- the packet age repeatedly exceeds one second;
- the drone ID is not 2;
- the ESP32 reports Wi-Fi or PX4 telemetry problems.

### If telemetry is not received

Check:

```powershell
ipconfig
```

The active laptop Wi-Fi address must match:

```text
CONFIG_HOST_IPV4_ADDR="192.168.43.106"
```

If it does not match, update it through `idf.py menuconfig`, rebuild, and flash.
Also allow Python through Windows Defender Firewall on private networks.

## 7. Propellers-removed command test

With propellers removed, run:

```bash
python simple_arm_takeoff_land.py --drone-id 2
```

Alternative command path using the simple goal script:

```bash
python simple_arm_set_goal.py --drone-id 2 --goal-x -2.0 --goal-y 5.0 --confirm
```

The program waits for telemetry and asks:

```text
Type ARM-2 to start the test:
```

Enter:

```text
ARM-2
```

The Python program sends:

```text
CMD_START
```

The expected ESP32 log sequence is:

```text
CMD_START
CMD_START received — checking ToF sensors...
All ... ToF sensors OK — proceeding to arm
Requesting OFFBOARD mode...
OFFBOARD mode confirmed
Arming...
Armed confirmed
Taking off to 0.5 m AGL...
```

Because the propellers are removed, this is only a communication and command
acceptance test. Use the remote controller or PX4 safety procedures as needed
to ensure the vehicle is disarmed before touching it.

## 8. Supervised low-altitude flight test

Conduct this only in an approved clear test area with appropriate supervision
and a manual recovery method.

Run:

```bash
python simple_arm_takeoff_land.py \
  --drone-id 2 \
  --takeoff-wait 12 \
  --hover-time 5
```

Simple waypoint flight command:

```bash
python simple_waypoint_mission.py \
  --drone-id 2 \
  --waypoints-file waypoints_example.txt \
  --takeoff-wait 5.0 \
  --arrival-timeout 30.0 \
  --confirm
```

Sequence:

1. Type `ARM-2`.
2. Python sends `CMD_START`.
3. ESP32 requests Offboard, arms, and commands NED `z = -0.5 m`.
4. Python waits 12 seconds and reports horizontal telemetry.
5. Python sends `CMD_HOLD`.
6. The UAV holds for five seconds.
7. Python sends `CMD_LAND`.

During the test, observe in QGroundControl:

- flight mode becomes Offboard;
- vehicle becomes armed;
- altitude rises to about 0.5 m;
- horizontal position remains stable;
- vehicle descends after `CMD_LAND`;
- vehicle disarms after touchdown.

Expected final ESP32 messages:

```text
CMD_LAND
CMD_LAND received from laptop
LAND command sent
Disarmed — mission complete
```

## 9. Emergency interruption

Pressing `Ctrl+C` after `CMD_START` makes the Python program attempt to send
`CMD_LAND` before closing.

This is only a software fallback. Keep the supervised manual recovery method
ready throughout the test.

## 10. Repeating the test

After landing, `mission_task` ends with:

```c
vTaskDelete(NULL);
```

The ESP32 continues sending telemetry, but it will no longer process another
full start sequence in `mission_task`. Before a second flight:

1. Confirm touchdown and disarming.
2. Stop the Python program.
3. Reset or power-cycle the ESP32.
4. Wait until the log again shows `Waiting for CMD_START from laptop...`.
5. Restart the Python program.

## 10. Important telemetry limitation

The existing custom telemetry contains horizontal position, heading,
navigation state, AprilTag information, VFH state, and relocation age.
It does not contain:

- altitude;
- armed state;
- PX4 flight mode;
- battery status;
- command acknowledgements.

Consequently, this simple Python script cannot independently verify takeoff or
landing completion. QGroundControl and the ESP32 serial log are used only to
observe those states during the initial tests.
