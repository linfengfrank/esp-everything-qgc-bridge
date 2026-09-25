# Trajectories

CSV files use `t,x,y,z` (seconds and NED metres). The uploader makes each path
relative to the hover position and holds the drone's starting heading.

## Minimum-snap ellipse

- 2.00 m North-South major axis × 1.00 m East-West minor axis
- 36 s clockwise lap; approximately 0.283 m/s peak speed
- Starts and finishes at rest at the +North major-axis tip

Place the drone at that tip with its nose outward in **+North / +x**. The centre
is 1.00 m behind it in **-North / -x**, and the path extends 0.50 m to either
side. Confirm the axes in QGC and allow extra safety clearance.

Run `ellipse_min_snap_demo.m` in MATLAB to regenerate the CSV and placement
guide. Set `run_simulation = true` for the optional Simulink test.

```bash
python3 laptop/send_trajectory.py --drone-id 22 --takeoff --takeoff-wait 12 --confirm \
    --trajectory trajectory/ellipse_min_snap_traj.csv
```

## Circle

Run `circle_traj_demo.m` in MATLAB to regenerate the default clockwise circle:
2 m diameter, approximately 21 s, starting toward `+y`. From takeoff it spans
`x = 0..-2 m` and `y = -1..+1 m`.

```bash
python3 laptop/send_trajectory.py --drone-id 22 --takeoff --takeoff-wait 12 --confirm \
    --trajectory trajectory/circle_traj.csv
```

Keep generated paths at or below 0.30 m/s. CSV generation needs MATLAB; the
optional simulations also require Simulink and the model's toolboxes.
