# Trajectories

Check this repo
[CDE1302_UAV](https://github.com/linfengfrank/CDE1302_UAV).

## Generate

Open `circle_traj_demo.m` in MATLAB and Run.
It writes `circle_traj.csv` (`t,x,y,z`: s, NED m, 100 Hz) next to itself, then
simulates tracking it. The CSV needs only MATLAB. The simulation also needs
Simulink, Aerospace Blockset, Aerospace Toolbox, and Robotics System Toolbox or
UAV Toolbox.

Keep `R*omega` at or below 0.3 m/s, since `send_trajectory.py` refuses anything
faster. The circle always runs clockwise seen from above; a negative `omega`
gives an empty CSV.

## Fly

```bash
python3 laptop/send_trajectory.py --drone-id 22 --takeoff --takeoff-wait 12 \
    --trajectory trajectory/circle_traj.csv
```

- The path starts wherever the drone is hovering, and the drone holds its heading.
- The drone starts the path once it is armed in OFFBOARD, even if it is still
  climbing, so leave enough `--takeoff-wait` to reach 0.5 m.
- From the start point, the default circle covers x 0 to -2 m and y -1 to +1 m
  (R = 1 m, 0.3 m/s, about 21 s). It heads toward +y first.
- x/y are PX4's local axes, not the drone's nose. Check which way +x points
  (QGC, `LOCAL_POSITION_NED`), then clear about 3 m x 3 m on the drone's -x side.
