%% ellipse_min_snap_demo
% Compact, closed minimum-snap ellipse in NED coordinates.
%
% The trajectory starts and ends at rest at the +North tip of the major
% axis. Put the drone at that tip with its nose pointing +North, away from
% the ellipse centre. The onboard player holds that takeoff heading for the
% whole lap.
%
% Outputs beside this file:
%   ellipse_min_snap_traj.csv       t,x,y,z at 100 Hz
%   ellipse_start_placement.png     top-down takeoff placement guide

clc; close all;
traj_dir = fileparts(mfilename('fullpath'));

%% User parameters
dt             = 0.01;  % output sample time (s); uploader resamples at 20 Hz
semi_major_N   = 1.00;  % half of the 2.0 m major axis, along North
semi_minor_E   = 0.50;  % half of the 1.0 m minor axis, along East
lap_time       = 36.0;  % one lap including smooth start/stop (s)
n_segments     = 24;    % ellipse waypoints; tangent constraints preserve shape
max_speed      = 0.30;  % must match send_trajectory.py's default limit (m/s)
clockwise      = true;  % viewed from above with North up and East right
run_simulation = false; % requires Simulink and the position-control toolboxes

assert(semi_major_N > semi_minor_E && semi_minor_E > 0, ...
    'Use positive axes with semi_major_N > semi_minor_E.');
assert(lap_time > 0 && n_segments >= 8, ...
    'lap_time must be positive and n_segments must be at least 8.');

%% Ellipse waypoints
direction = 1;
if ~clockwise
    direction = -1;
end

theta = direction * linspace(0, 2*pi, n_segments + 1).';
centre_N = -semi_major_N; % makes the first waypoint [0, 0, 0]
centre_E = 0;
waypoints = [centre_N + semi_major_N*cos(theta), ...
             centre_E + semi_minor_E*sin(theta), ...
             zeros(size(theta))];

% Allocate time in proportion to chord length. The QP is solved with the
% same relative durations scaled to mean 1 s, which improves conditioning;
% a uniform time scale does not change the minimum-snap spatial solution.
chord = vecnorm(diff(waypoints, 1, 1), 2, 2);
segment_time = lap_time * chord / sum(chord);
shape_time = n_segments * chord / sum(chord);

%% Seventh-order, C3-continuous minimum-snap solve
% Position is fixed at every ellipse waypoint. Velocity is constrained to
% the ellipse tangent at each internal waypoint. Velocity, acceleration and
% jerk are continuous at joins and zero at the beginning and end.
coeff = solveMinimumSnapEllipse(waypoints, shape_time, ...
    centre_N, centre_E, semi_major_N, semi_minor_E);

%% Sample position and derivatives using the real lap timing
t = (0:dt:lap_time).';
[position, velocity, acceleration] = sampleTrajectory(coeff, segment_time, t);

X_n = position(:,1);  Y_n = position(:,2);  Z_n = position(:,3);
Vx_n = velocity(:,1); Vy_n = velocity(:,2); Vz_n = velocity(:,3);
Ax_n = acceleration(:,1); Ay_n = acceleration(:,2); Az_n = acceleration(:,3);

% This heading is for the optional simulation. The ESP trajectory player
% instead captures and holds the real drone heading when playback starts.
Yaw = zeros(size(t));
Yawrate = zeros(size(t));

speed = vecnorm(velocity, 2, 2);
assert(max(speed) <= max_speed + 1e-9, ...
    'Peak speed %.3f m/s exceeds %.3f m/s; increase lap_time.', ...
    max(speed), max_speed);

csv_path = fullfile(traj_dir, 'ellipse_min_snap_traj.csv');
writetable(table(t, X_n, Y_n, Z_n, ...
    'VariableNames', {'t','x','y','z'}), csv_path);

fprintf('Wrote %s\n', csv_path);
fprintf('Footprint: %.2f m North-South x %.2f m East-West\n', ...
    max(X_n)-min(X_n), max(Y_n)-min(Y_n));
fprintf('Duration: %.1f s; peak speed: %.3f m/s\n', t(end), max(speed));

%% Top-down placement guide: East right, North up
guide = figure('Name', 'Ellipse takeoff placement', 'Color', 'w', ...
    'Position', [100 100 820 720]);
plot(Y_n, X_n, 'Color', [0.08 0.45 0.78], 'LineWidth', 3); hold on;
plot(centre_E, centre_N, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
plot(Y_n(1), X_n(1), 'o', 'Color', [0.86 0.20 0.16], ...
    'MarkerFaceColor', [0.86 0.20 0.16], 'MarkerSize', 12);

% Nose/outward direction at the starting tip.
quiver(Y_n(1), X_n(1), 0, 0.48, 0, 'Color', [0.86 0.20 0.16], ...
    'LineWidth', 3, 'MaxHeadSize', 0.65);
text(Y_n(1)+0.05, X_n(1)+0.30, ...
    {'DRONE HERE', 'nose / +North', '(outward)'}, ...
    'Color', [0.70 0.10 0.08], 'FontWeight', 'bold', 'FontSize', 11);
text(centre_E+0.04, centre_N, 'ellipse centre', ...
    'Color', [0.12 0.12 0.12], 'FontSize', 10);

% Direction arrow after the vehicle has begun moving clockwise.
[~, arrow_i] = min(abs(t - 0.18*lap_time));
arrow_j = min(arrow_i + round(0.55/dt), numel(t));
quiver(Y_n(arrow_i), X_n(arrow_i), ...
    Y_n(arrow_j)-Y_n(arrow_i), X_n(arrow_j)-X_n(arrow_i), 0, ...
    'Color', [0.08 0.45 0.78], 'LineWidth', 2, 'MaxHeadSize', 1.1);
text(0.70, -0.82, 'clockwise', 'Color', [0.08 0.35 0.65], ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right');

axis equal; grid on; box on;
xlim(1.55*[-semi_minor_E semi_minor_E]);
ylim([centre_N-semi_major_N-0.18, 0.50]);
xlabel('East / +y (m)'); ylabel('North / +x (m)');
title({'Minimum-snap ellipse: takeoff placement (top view)', ...
       sprintf('Path stays behind the nose; axes %.2f m x %.2f m', ...
       2*semi_major_N, 2*semi_minor_E)});
subtitle('Red dot = major-axis tip and takeoff point');

guide_path = fullfile(traj_dir, 'ellipse_start_placement.png');
exportgraphics(guide, guide_path, 'Resolution', 180);
fprintf('Wrote %s\n', guide_path);

%% Optional Simulink tracking test
if run_simulation
    addpath(fullfile(traj_dir, 'position_control'), ...
            fullfile(traj_dir, 'position_control', 'icon'));

    ref.time = t;
    ref.signals.values = [X_n Y_n Z_n Vx_n Vy_n Vz_n ...
                          Ax_n Ay_n Az_n Yaw Yawrate];
    ref.signals.dimensions = 11;

    model = 'PosControl_Sim_RPT';
    open_system(model);
    set_param(model, 'StopTime', num2str(t(end)));
    sim(model);

    figure('Name', 'Ellipse tracking');
    plot3(X_n, Y_n, Z_n, '--', 'LineWidth', 1.5); hold on;
    plot3(PosE.signals.values(:,1), PosE.signals.values(:,2), ...
          PosE.signals.values(:,3), 'LineWidth', 2);
    grid on; axis equal;
    xlabel('North (m)'); ylabel('East (m)'); zlabel('Down (m)');
    legend('reference', 'simulated');
end

%% Local functions
function coeff = solveMinimumSnapEllipse(waypoints, segment_time, ...
        centre_N, centre_E, semi_major_N, semi_minor_E)
    n_order = 7;
    n_coef = n_order + 1;
    n_seg = size(waypoints, 1) - 1;
    n_vars = n_seg * n_coef;

    % Integral of squared fourth derivative on every segment.
    Q = zeros(n_vars);
    for s = 1:n_seg
        block = zeros(n_coef);
        T = segment_time(s);
        for i = 4:n_order
            for j = 4:n_order
                block(i+1,j+1) = factorial(i)/factorial(i-4) ...
                    * factorial(j)/factorial(j-4) / (i+j-7) / T^7;
            end
        end
        cols = (s-1)*n_coef + (1:n_coef);
        Q(cols,cols) = block;
    end

    % Eight endpoint rows plus five rows per internal join.
    n_constraints = 8 + 5*(n_seg-1);
    A = zeros(n_constraints, n_vars);
    B = zeros(n_constraints, 2);
    row_i = 0;

    for derivative = 0:3
        row_i = row_i + 1;
        A(row_i,1:n_coef) = basisRow(0, derivative, segment_time(1));
        if derivative == 0
            B(row_i,:) = waypoints(1,1:2);
        end
    end

    last_cols = (n_seg-1)*n_coef + (1:n_coef);
    for derivative = 0:3
        row_i = row_i + 1;
        A(row_i,last_cols) = basisRow(1, derivative, segment_time(end));
        if derivative == 0
            B(row_i,:) = waypoints(end,1:2);
        end
    end

    for s = 1:n_seg-1
        left = (s-1)*n_coef + (1:n_coef);
        right = s*n_coef + (1:n_coef);

        row_i = row_i + 1;
        A(row_i,left) = basisRow(1, 0, segment_time(s));
        B(row_i,:) = waypoints(s+1,1:2);

        row_i = row_i + 1;
        A(row_i,right) = basisRow(0, 0, segment_time(s+1));
        B(row_i,:) = waypoints(s+1,1:2);

        for derivative = 1:3
            row_i = row_i + 1;
            A(row_i,left) = basisRow(1, derivative, segment_time(s));
            A(row_i,right) = -basisRow(0, derivative, segment_time(s+1));
        end
    end

    % Couple North and East with one tangent constraint at each internal
    % waypoint. The ellipse normal is [(N-Nc)/a^2, (E-Ec)/b^2], and its dot
    % product with velocity must be zero. This removes corner-cutting and
    % scalloping while leaving the internal tangent speed free for the QP.
    Axy = [A zeros(size(A)); zeros(size(A)) A];
    bxy = [B(:,1); B(:,2)];
    tangent_A = zeros(n_seg-1, 2*n_vars);
    for s = 1:n_seg-1
        velocity_row = zeros(1, n_vars);
        cols = (s-1)*n_coef + (1:n_coef);
        velocity_row(cols) = basisRow(1, 1, segment_time(s));
        normal_N = (waypoints(s+1,1) - centre_N) / semi_major_N^2;
        normal_E = (waypoints(s+1,2) - centre_E) / semi_minor_E^2;
        tangent_A(s,:) = [normal_N*velocity_row, normal_E*velocity_row];
    end
    Axy = [Axy; tangent_A];
    bxy = [bxy; zeros(n_seg-1,1)];

    % Equality-constrained QP KKT system. This avoids an Optimization
    % Toolbox dependency. A common factor of 2 in Q does not affect coeff.
    Qxy = [Q zeros(size(Q)); zeros(size(Q)) Q];
    KKT = [Qxy Axy'; Axy zeros(size(Axy,1))];
    solution = KKT \ [zeros(2*n_vars,1); bxy];
    coeff = zeros(n_vars,3);
    coeff(:,1) = solution(1:n_vars);
    coeff(:,2) = solution(n_vars+1:2*n_vars);
end

function [position, velocity, acceleration] = sampleTrajectory(coeff, segment_time, t)
    n_coef = 8;
    n_seg = numel(segment_time);
    edges = [0; cumsum(segment_time(:))];
    position = zeros(numel(t), 3);
    velocity = zeros(numel(t), 3);
    acceleration = zeros(numel(t), 3);

    for s = 1:n_seg
        if s < n_seg
            sample = t >= edges(s) & t < edges(s+1);
        else
            sample = t >= edges(s) & t <= edges(s+1) + eps(edges(end));
        end
        tau = (t(sample) - edges(s)) / segment_time(s);
        cols = (s-1)*n_coef + (1:n_coef);
        C = coeff(cols,:);
        position(sample,:) = basisMatrix(tau, 0, segment_time(s)) * C;
        velocity(sample,:) = basisMatrix(tau, 1, segment_time(s)) * C;
        acceleration(sample,:) = basisMatrix(tau, 2, segment_time(s)) * C;
    end
end

function row = basisRow(tau, derivative, duration)
    row = basisMatrix(tau, derivative, duration);
end

function M = basisMatrix(tau, derivative, duration)
    n_order = 7;
    tau = tau(:);
    M = zeros(numel(tau), n_order+1);
    for power = derivative:n_order
        M(:,power+1) = factorial(power)/factorial(power-derivative) ...
            * tau.^(power-derivative) / duration^derivative;
    end
end
