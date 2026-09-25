# Arm, takeoff, hover and land test

`laptop/simple_arm_takeoff_land.py`: CMD_START → hover → CMD_HOLD → CMD_LAND.
Run from the repo root. Examples use drone **22**; use your flashed ID.

## Setup

- Flash with `./flash_drone.sh <id> <port>`. The ESP32 gets `192.168.1.(200+id)`
  and sends telemetry only to `192.168.1.(100+id)`, so the laptop must hold that
  address on the drone Wi-Fi (README, "Laptop IP address").
- Python: README section 2.
- Serial monitor: `idf.py -p <port> monitor`. Open it before arming (it resets
  the ESP32). Wait for `Waiting for CMD_START from laptop...`.
- QGroundControl: observe only.
- Optional link check (close QGC first): `python3 laptop/check_px4_esp32_link.py --drone-id 22` → `RESULT: PASS ...`.
- `camera_stream.py` / `tag_stream.py` can run alongside (use `--fps 5` in
  flight). Tags never affect the flight. Start `tag_debug.py` only after the
  flight script, because it takes UDP 5005.

## Commands

| Command | ESP32 behaviour |
|---------|-----------------|
| `CMD_START` | Accepted once per flight, from `Pre-streaming hold setpoint...` on; otherwise ignored (`CMD_START ignored — not waiting for it`). OFFBOARD → arm (gives up after 10 s) → climb to 0.5 m. |
| `CMD_HOLD` | Stops an active goal/trajectory; no effect otherwise. |
| `CMD_LAND` | Any time after START: aborts OFFBOARD/arming, or lands (also mid-climb). Ignored before START. |
| `CMD_GOTO`, trajectory start | Only after takeoff completes; otherwise ignored (`... ignored — not flying`). |

## 1. Communication check

```bash
python3 laptop/simple_arm_takeoff_land.py --drone-id 22 --monitor-only
```

Expect `Connected: drone=22, ESP32_IP=192.168.1.222, ...`, then 15 s of
`MONITOR | ... | nav=IDLE | age=...` with `age` well below 1 s. No flight commands are sent.

If it fails with `No telemetry from drone 22 on UDP 5005 (heard drones: [...])`:
another ID listed → wrong `--drone-id`; `none` → check the laptop IP and firewall.

## 2. Props-off test, then flight

```bash
python3 laptop/simple_arm_takeoff_land.py --drone-id 22
```

Type `ARM-22`. The script sends CMD_START, waits `--takeoff-wait` (12 s), sends
CMD_HOLD, waits `--hover-time` (5 s), sends CMD_LAND and exits.

Key ESP32 lines (others interleave):

```text
mission: CMD_START received
mission: ToF disabled (TOF_ENABLED=0) — skipping pre-arm sensor check
mission: OFFBOARD mode confirmed
mission: Armed confirmed
mission: Taking off to 0.5 m AGL (NED z=-0.50)...
mission: Altitude reached: NED z=-0.3x (target=-0.50)
mission: Exploration mode — waiting for laptop goals...
wifi: CMD_HOLD
wifi: CMD_LAND
mission: CMD_LAND received from laptop
mission: Land command sent
mission: Disarmed — mission complete
mission: Mission loop complete — waiting for next CMD_START
```

- Props off: `Takeoff timeout: NED z=0.0x ...` replaces `Altitude reached` (expected).
- `Takeoff aborted: OFFBOARD refused (check QGC)` / `arming refused (check QGC)`:
  PX4 rejected it. Fix the cause in QGC and run again.
- `Still armed after 20 s — land with RC`: take over and land with the RC.

In flight, watch QGC: Offboard → armed → ~0.5 m → steady hover → Land → disarmed.

## 3. Emergency

- **RC pilot first:** switch out of Offboard or use the kill switch. The ESP32
  never overrides this. After an RC landing it logs `Disarmed without CMD_LAND`
  and is ready for the next START.
- **Ctrl+C** after START sends CMD_LAND (3×): it aborts arming, or lands the
  drone, even mid-climb. Stale telemetry (3 s) does the same.
- If the script or laptop dies without sending LAND, the drone keeps hovering: use the RC.
- **Wi-Fi killswitch:** if the ESP32 loses the access point it holds position,
  and after 3 s sends a non-forced disarm. PX4 normally refuses this in the air
  (`Disarming denied: not landed`), so take over with the RC.

## 4. Repeat

No reboot needed. Wait for `Waiting for CMD_START from laptop...` and run again.

## Limitations

- Telemetry has no altitude, armed state or flight mode: use QGC and the serial monitor.
- Commands are not acknowledged: check `wifi: CMD_...` in the serial monitor.
- A START sent too early is ignored. The script can't tell, so check the serial monitor.
- `tag_id` / `tag_dist_m` in telemetry are info only, latched until the ESP32 reboots.
