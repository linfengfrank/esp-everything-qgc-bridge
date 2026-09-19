% Quadrotor model parameters (NUSWARM); Init_control.m runs this.

% Initial condition
ModelInit_PosE = [0, 0, 0];
ModelInit_VelB = [0, 0, 0];
ModelInit_AngEuler = [0, 0, 0];
ModelInit_RateB = [0, 0, 0];
ModelInit_Rads = 1500;   % motor speed (rad/s)

% Airframe
ModelParam_uavMass = 0.302;                                % kg
ModelParam_uavJ = diag([3.9195e-4, 4.0515e-4, 6.3890e-4]);  % inertia (kg.m^2)
ModelParam_uavR = 0.0755;                                  % body radius (m)
ModelParam_uavCd = 0.0;                                    % damping coefficient (N/(m/s)^2)
ModelParam_uavCCm = [0.00 0.00 0.00];                      % damping moment coefficients

% Motors and rotors
ModelParam_motorCr = 3480.7;                   % throttle-speed slope (rad/s)
ModelParam_motorWb = 445.7799;                 % throttle-speed constant (rad/s)
ModelParam_motorCp = [-4160  7902.2  -510.3];  % normalized PWM -> speed, 2nd-order polynomial
ModelParam_motorT = 0.02;                      % time constant (s)
ModelParam_motorJm = 3.1771e-6;                % rotor + propeller inertia (kg.m^2)
ModelParam_rotorCm = 4.7345e-9;                % torque M = Cm*w^2
ModelParam_rotorCt = 2.9625e-7;                % thrust T = Ct*w^2

ModelParam_envGravityAcc = 9.8;                % m/s^2
