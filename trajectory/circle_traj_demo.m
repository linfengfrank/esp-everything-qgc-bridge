%% circle_traj_demo
% Constant-speed circle in NED. Writes circle_traj.csv (t,x,y,z) next to this
% file for laptop/send_trajectory.py, then simulates tracking it in Simulink.

clc; close all;
traj_dir = fileparts(mfilename('fullpath'));   % resolve files from here, not the current folder

%% User params
dt        = 0.01;          % sample time (s)
R         = 1.0;           % radius (m)
omega     = 0.3;           % angular rate (rad/s); keep R*omega <= 0.3 m/s
T_end     = 2*pi/omega;    % one lap (s)
theta0    = 0;             % start angle (rad)
center_N  = -2.0;          % centre (m, NED); only the simulation uses it
center_E  = 2.0;
center_D  = 0.0;
yaw_mode  = "tangent";     % "tangent" or "fixed"; only the simulation uses it
yaw_fixed = deg2rad(0);    % used when yaw_mode = "fixed"

%% Reference: position, velocity, acceleration, yaw (NED)
t     = (0:dt:T_end).';
theta = theta0 + omega .* t;
cosT  = cos(theta);  sinT = sin(theta);

X_n  = center_N + R * cosT;
Y_n  = center_E + R * sinT;
Z_n  = center_D + zeros(size(t));
Vx_n = -R * sinT * omega;
Vy_n =  R * cosT * omega;
Vz_n =  zeros(size(t));
Ax_n = -R * cosT * (omega^2);
Ay_n = -R * sinT * (omega^2);
Az_n =  zeros(size(t));

switch yaw_mode
    case "tangent", Yaw = atan2(Vy_n, Vx_n);   % face along velocity
    case "fixed",   Yaw = yaw_fixed * ones(size(t));
    otherwise,      error('Unknown yaw_mode.');
end
Yawrate = [0; diff(Yaw)] ./ [1; diff(t)];

%% CSV for the drone (position only)
writetable(table(t, X_n, Y_n, Z_n, 'VariableNames', {'t','x','y','z'}), fullfile(traj_dir, 'circle_traj.csv'));

figure('Name','Circle (NED)','Color','w');
plot3(X_n, Y_n, Z_n, 'LineWidth',2); grid on; axis equal; hold on;
plot3(center_N, center_E, center_D, 'ko', 'MarkerFaceColor','k');
xlabel('North (m)'); ylabel('East (m)'); zlabel('Down (m)');
title(sprintf('R=%.1f m, T=%.1f s, |v|=R|ω|=%.2f m/s', R, T_end, abs(R*omega)));

%% Simulink: fly the reference with the flight controller
addpath(fullfile(traj_dir, 'position_control'), fullfile(traj_dir, 'position_control', 'icon'));

ref.time = t;
ref.signals.values     = [X_n Y_n Z_n Vx_n Vy_n Vz_n Ax_n Ay_n Az_n Yaw Yawrate];
ref.signals.dimensions = 11;

model = 'PosControl_Sim_RPT';
open_system(model);
set_param(model, 'StopTime', num2str(t(end)));
sim(model);

sim_pose.t = PosE.time;
sim_pose.x = PosE.signals.values(:,1);
sim_pose.y = PosE.signals.values(:,2);
sim_pose.z = PosE.signals.values(:,3);

figure;
subplot(2,2,1); plot(sim_pose.t, sim_pose.x); grid on; xlabel('time (s)'); ylabel('x (m)');
subplot(2,2,2); plot(sim_pose.t, sim_pose.y); grid on; xlabel('time (s)'); ylabel('y (m)');
subplot(2,2,3); plot(sim_pose.t, sim_pose.z); grid on; xlabel('time (s)'); ylabel('z (m)');
subplot(2,2,4); plot3(sim_pose.x, sim_pose.y, sim_pose.z, 'LineWidth',2); grid on; axis equal;
xlabel('North (m)'); ylabel('East (m)'); zlabel('Down (m)');
