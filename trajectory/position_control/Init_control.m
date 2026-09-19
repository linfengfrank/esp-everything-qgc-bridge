% Controller parameters for PosControl_Sim_RPT (its InitFcn runs this).
Init;

DEG2RAD = 0.0174533;
CONSTANTS_ONE_G = 9.8;
FLT_EPSILON = 1e-5;
THR_HOVER = 0.331;   % hover throttle

% Attitude control (nuswarm param, min_snap_30_11_23)
MC_PITCH_P = 6.5;
MC_PITCHRATE_P = 0.06;
MC_PITCHRATE_I = 0.2;
MC_PITCHRATE_D = 0.0005;

MC_ROLL_P = 11.5;
MC_ROLLRATE_P = 0.055;
MC_ROLLRATE_I = 0.2;
MC_ROLLRATE_D = 0.0005;

MC_YAW_P = 2.8;
MC_YAWRATE_P = 0.2;
MC_YAWRATE_I = 0.1;
MC_YAWRATE_D = 0.00;
MC_YAW_WEIGHT = 0.4;

% Integral saturation
Saturation_I_RP_Max = 0.3;
Saturation_I_RP_Min = -0.3;
Saturation_I_Y_Max = 0.2;
Saturation_I_Y_Min = -0.2;
Saturation_I_az = 5;

% Limits
MAX_CONTROL_ANGLE_RATE_PITCH = 220;
MAX_CONTROL_ANGLE_RATE_ROLL = 220;
MAX_CONTROL_ANGLE_RATE_Y = 200;
MAX_CONTROL_VELOCITY_XY = 5*2;   % m/s
MAX_CONTROL_VELOCITY_Z = 3*2;
MPC_TILTMAX_AIR = 90;            % deg
MPC_THR_MAX = 0.9;
MPC_THR_MIN = 0.06;
MPC_THR_XY_MARG = 0.3;

% RPT outer loops (z reuses ki from xy)
wn    = 0.4;
sigma = 1.1*1.5;
ki    = 0.8*1.5;
eps   = 1*0.4;
F_xy  = [(wn^2+2*sigma*wn*ki)/eps^2, (2*sigma*wn+ki)/eps, ki*wn^2/eps^3, -(wn^2+2*sigma*wn*ki)/eps^2, -(2*sigma*wn+ki)/eps];

wn    = 0.5;
sigma = 1.1*1.5;
eps   = 1*0.5;
F_z   = [(wn^2+2*sigma*wn*ki)/eps^2, (2*sigma*wn+ki)/eps, ki*wn^2/eps^3, -(wn^2+2*sigma*wn*ki)/eps^2, -(2*sigma*wn+ki)/eps];
