function par = rocket_params()
%ROCKET_PARAMS  Single source of truth for all rocket parameters.
%  [PLACEHOLDER] values should be updated after CFD data.

par = struct();

%% Rocket specifications [PLACEHOLDER]
par.m0       = 2.5;           % initial mass (kg)
par.m_prop   = 0.4;           % propellant mass (kg)
par.L        = 0.8;           % rocket length (m)
par.d        = 0.10;          % rocket diameter (m)
par.Sref     = pi/4*par.d^2;  % reference area (m^2)
par.g        = 9.81;          % gravitational acceleration (m/s^2)

%% Moment of inertia [PLACEHOLDER]
par.Ixx = 0.0026;    % roll (kg*m^2)
par.Iyy = 0.1106;    % pitch (kg*m^2)
par.Izz = 0.1106;    % yaw (kg*m^2)

%% Thrust profile
par.motor_select = 'short';

switch lower(par.motor_select)
    case 'short', motor_csv = 'motor_1_6s.csv';
    case 'long',  motor_csv = 'motor_3_6s.csv';
    case 'const', motor_csv = '';
    otherwise, error('Unknown motor_select: %s', par.motor_select);
end

if ~isempty(motor_csv)
    par = load_motor_csv(par, motor_csv);
else
    par.T_avg    = 200;
    par.t_burn   = 3.5;
    par.m_prop   = 0.4;
    par.mdot     = par.m_prop / par.t_burn;
    par.thrust_profile = [];
end

%% Aerodynamic coefficients
par.CNa =  8.8998;   par.CNd =  1.5;
par.Cma = -4.4;      par.Cmd =  2.5;
par.Cmq = -50.0;

% Yaw (pitch/yaw symmetry)
par.CYb  = -par.CNa;  par.CYdr =  par.CNd;
par.Cnb  = -par.Cma;  par.Cndr = -par.Cmd;
par.Cnr  =  par.Cmq;

% Roll [PLACEHOLDER]
par.Clp  = -0.5;    par.Clda = 0.12;

% Drag [PLACEHOLDER]
par.CA0  = 0.42;

%% Actuator [PLACEHOLDER]
par.wn_act         = 60;           % rad/s
par.zeta_act       = 0.7;
par.delta_max      = deg2rad(15);
par.delta_rate_max = deg2rad(360);
par.roll_alloc_max = deg2rad(10);

%% Guidance
par.guid_law      = 'augmented pp';
par.t_guide_on    = 1.0;
par.K_guid        = 3.0;
par.qbar_min_ctrl = 50;

par.K_guid_far       = 1.5;
par.K_guid_near      = 3.0;
par.R_gain_sched     = 150;
par.R_terminal_slant =  50;

par.N_PN          = 5.0;
par.pn_filter_tau = 0.05;

par.K_guid_vert = 1.5;

par.wind_est = [0; 0; 0];

%% Target
par.target_NED = [100; 100; -500];

%% Launch condition (automatic LOS aiming)
los = par.target_NED;
par.pitch0 = atan2(-los(3), norm(los(1:2)));
par.yaw0   = atan2(los(2), los(1));
par.V0     = 15.0;

%% Gain design parameters
par.design_V = 80;
par.design_h = 300;
par.design_m = par.m0 - par.m_prop;

par.sched_V       = [40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180];

par.gain_method = 'bode';

par.wc_rate       = 25;
par.wc_accel      = 10;
par.wI_accel      = 1.0;
par.wc_roll_inner = 25;
par.roll_pi_zero  = 2.0;
par.wc_roll_outer = 10.0;

par.pole_zeta     = 0.7;
par.pole_wn_scale = 1.0;
par.pole_pi_ratio = 0.3;
par.pole_PM_min   = 35;
par.pole_max_iter = 15;

par.disable_pitch_yaw = false;
par.disable_angle_feedback = false;


%% Atmosphere model
par.rho0    = 1.225;
par.h_scale = 8500;

%% Simulation parameters
par.t_sim_max   = 25;
par.ode_RelTol  = 1e-6;
par.ode_AbsTol  = 1e-8;
par.ode_MaxStep = 0.01;

end

%% Thrust profile loading
function par = load_motor_csv(par, csv_file)
    data = readmatrix(csv_file);
    t_raw = data(:,1);  T_raw = data(:,2);
    mp_g1 = data(:,3);  mp_g2 = data(:,4);
    mdot_g1 = data(:,5); mdot_g2 = data(:,6);

    m_prop_remaining = (mp_g1 + mp_g2) / 1000;
    burn_end_idx = find(T_raw > 0, 1, 'last');
    if isempty(burn_end_idx), burn_end_idx = length(t_raw); end
    if burn_end_idx < length(t_raw)
        idx = 1:(burn_end_idx+1);
    else
        idx = 1:burn_end_idx;
    end

    tp.t    = t_raw(idx);
    tp.T    = T_raw(idx);
    tp.mdot = mdot_g1(idx) + mdot_g2(idx);
    tp.m_prop_remaining = m_prop_remaining(idx);
    par.thrust_profile = tp;

    par.t_burn = tp.t(end);
    par.m_prop = m_prop_remaining(1);
    par.T_avg  = trapz(tp.t, tp.T) / par.t_burn;
    par.mdot   = par.m_prop / par.t_burn;

    fprintf('Motor loaded: %s (%.2fs, %.1fN avg, %.1f N*s)\n', ...
        csv_file, par.t_burn, par.T_avg, trapz(tp.t, tp.T));
end
