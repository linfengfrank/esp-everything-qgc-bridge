# Waypoint flight test

`laptop/simple_waypoint_mission.py`: CMD_START → CMD_GOTO per waypoint → CMD_LAND.
Do the arm/takeoff test in [`laptop/TEST_GUIDE.md`](../laptop/TEST_GUIDE.md)
first; setup (network, serial monitor, QGC, camera viewers) and emergency steps
are the same. Run from the repo root. Examples use drone **22**.

## Waypoints

- `x,y` in metres, NED: x = north, y = east. They are PX4 local coordinates
  (origin normally where the drone was powered on) plus the drone's
  `start_x`/`start_y` in `laptop/setup.yaml`, which is (0,0) if the drone isn't
  listed (22 isn't). Power on at the takeoff spot.
- The drone flies straight lines at 0.5 m, at about 0.2 m/s (PX4 `MPC_XY_VEL_MAX`).
  There is no obstacle avoidance. Tags never shift waypoints.
- Inline: `--waypoint 0.5,0 --waypoint -0.5,0.5`.
- File: `--waypoints-file <file>`, one `x,y` or `x y` per line; whole-line `#`
  comments.

| File | Route |
|------|-------|
| `waypoints/waypoints_example.txt` | (0,0) → (0.8,0) → (0.8,0.8). First flight |
| `laptop/my_path.txt` | 2 m zig-zag north |
| `laptop/navtag_test_path.txt` | 1 m west, back, then the zig-zag |

## Run

Communication check (this script has no monitor mode):
`python3 laptop/simple_arm_takeoff_land.py --drone-id 22 --monitor-only`

```bash
python3 laptop/simple_waypoint_mission.py --drone-id 22 --confirm \
  --waypoints-file waypoints/waypoints_example.txt
```

Type `MISSION-22`. **Always use `--confirm`**: without it, the drone takes off
as soon as telemetry arrives.

| Option | Default | Notes |
|--------|---------|-------|
| `--takeoff-wait` | 10 s | Before the first GOTO. A GOTO is ignored until takeoff completes, and the script resends it every 2 s until the drone moves. |
| `--arrival-timeout` | 25 s | Per waypoint (~4 m at 0.2 m/s). Raise it for longer legs. |
| `--arrival-radius` | 0.25 m | Horizontal. |
| `--finish-action` | `land` | `hold` leaves it hovering: land with the RC. |
| `--stale-timeout` | 3 s | Lands if telemetry stops. |

Exit code: `0` all waypoints reached, `1` error or missed waypoint, `130` Ctrl+C.

Props-off desk test: `--waypoint 0.3,0 --arrival-timeout 5`. Expect a
`Waypoint arrival timeout` warning, then CMD_LAND and exit code 1.

Key log lines:

```text
laptop: Sending waypoint 1/3 to (0.00, 0.00)
laptop: Waypoint reached at (0.03, -0.02), goal=(0.00, 0.00), dist=0.04 m
ESP32:  nav: Goal set: ... → wifi: CMD_GOTO map(0.80,0.00) → nav: Goal reached (dist=0.24 m)
```

The takeoff and landing lines are the same as in the arm test.

## Emergency

- RC pilot first.
- Ctrl+C or any script error after START sends CMD_LAND (3×), including when
  telemetry is stale for more than 3 s. Ctrl+C before START sends nothing.
- If the laptop dies without LAND, the drone hovers at its current goal: use the RC.

## Repeat

No reboot needed. Wait for `Waiting for CMD_START from laptop...`.

## Limitations

- Telemetry has no altitude, armed state or flight mode.
- Commands are not acknowledged. GOTO is resent and LAND is sent 3×; confirm in the serial monitor.
