function xdot = eom_6dof(t, x, par, ground_vel_in, wind_est_in)
%EOM_6DOF  6-DOF nonlinear equations of motion (27 states).
%
%  State vector (27):
%   x(1:3)   pos NED          x(4:6)   vel NED
%   x(7:10)  quat [q0..q3]    x(11:13) body rates [p q r]
%   x(14)    roll PI integrator
%   x(15:22) fin actuators (4x[d, d_dot])
%   x(23:25) guidance states (PN LOS rate filter)
%   x(26:27) pitch/yaw accel error integrator

xdot = zeros(27, 1);

% Unpack
pos_NED = x(1:3);  vel_NED = x(4:6);
quat    = x(7:10); omega_b = x(11:13);
int_ep  = x(14);
fin     = [x(15); x(17); x(19); x(21)];
fin_dot = [x(16); x(18); x(20); x(22)];
x_guid  = x(23:25);

p = omega_b(1); q_rate = omega_b(2); r = omega_b(3);

% Quaternion normalize
qn = norm(quat);
if qn < 1e-10, quat = [1;0;0;0]; else, quat = quat/qn; end

R_BN = gnc_utils.quat2dcm_bn(quat);
R_NB = R_BN';

% Body velocity
vel_b = R_BN * vel_NED;
u = vel_b(1); v_body = vel_b(2); w = vel_b(3);
V = max(norm(vel_b), 1.0);

alpha = atan2(w, max(u, 0.1));
beta  = asin(max(-1, min(1, v_body/V)));

% Atmosphere, mass, thrust
alt  = max(-pos_NED(3), 0);
rho  = par.rho0 * exp(-alt / par.h_scale);
qbar = 0.5 * rho * V^2;

if isfield(par, 'thrust_profile') && ~isempty(par.thrust_profile)
    tp = par.thrust_profile;
    if t < tp.t(end)
        T = interp1(tp.t, tp.T, t, 'linear', 0);
        m = (par.m0 - par.m_prop) + interp1(tp.t, tp.m_prop_remaining, t, 'linear', 0);
    else
        T = 0;  m = par.m0 - par.m_prop;
    end
else
    if t < par.t_burn, m = par.m0 - par.mdot*t; T = par.T_avg;
    else,              m = par.m0 - par.m_prop;  T = 0; end
    m = max(m, par.m0 - par.m_prop);
end

%% Guidance
if nargin >= 4 && ~isempty(ground_vel_in)
    nav_vel = ground_vel_in;
else
    nav_vel = vel_NED;
end

if nargin >= 5 && ~isempty(wind_est_in)
    [nz_cmd, ny_cmd, xdot_guid] = guidance_module(t, pos_NED, nav_vel, R_BN, qbar, m, par, x_guid, wind_est_in);
else
    [nz_cmd, ny_cmd, xdot_guid] = guidance_module(t, pos_NED, nav_vel, R_BN, qbar, m, par, x_guid);
end
xdot(23:25) = xdot_guid;

persistent dbg_last_t
dbg_on = ~(isfield(par, 'suppress_debug') && par.suppress_debug);
if dbg_on && (isempty(dbg_last_t) || t - dbg_last_t >= 0.5)
    dbg_last_t = t;
    fprintf('[t=%5.3f] qbar=%6.1f  nz_cmd=%+7.4f  ny_cmd=%+7.4f  ', ...
        t, qbar, nz_cmd, ny_cmd);
end

%% Autopilot -- 3-loop cascade
if isfield(par,'V_ctrl_on') && par.V_ctrl_on > 0
    ctrl_active = (V > par.V_ctrl_on);
else
    ctrl_active = (qbar > par.qbar_min_ctrl);
end

% Gain lookup
if isfield(par, 'gain_table')
    V_bp = par.gain_table.V_bp;
    Vc   = max(V_bp(1), min(V_bp(end), V));
    KR_g  = interp1(V_bp, par.gain_table.KR,  Vc);
    KA_g  = interp1(V_bp, par.gain_table.KA,  Vc);
    KDC_g = interp1(V_bp, par.gain_table.KDC, Vc);
    K_phi_g   = interp1(V_bp, par.gain_table.K_phi,   Vc);
    Kp_roll_g = interp1(V_bp, par.gain_table.Kp_roll, Vc);
    Ki_roll_g = interp1(V_bp, par.gain_table.Ki_roll, Vc);
else
    KR_g = par.KR; KA_g = par.KA; KDC_g = par.KDC;
    K_phi_g = par.K_phi; Kp_roll_g = par.Kp_roll; Ki_roll_g = par.Ki_roll;
end

if isfield(par, 'gain_table') && isfield(par.gain_table, 'wI')
    wI_g = interp1(V_bp, par.gain_table.wI, Vc);
elseif isfield(par, 'wI')
    wI_g = par.wI;
else
    wI_g = 0;
end

% Pitch/Yaw
int_nz = x(26);  int_ny = x(27);

if ctrl_active
    nz_fb = (q_rate * u) / par.g;
    ny_fb = - (r * u) / par.g;

    nz_err = KDC_g * nz_cmd - nz_fb;
    ny_err = KDC_g * ny_cmd - ny_fb;

    de_v = KR_g * (KA_g * nz_err + wI_g * int_nz - q_rate);
    dr_v = KR_g * (KA_g * ny_err + wI_g * int_ny - r);

    % Integrator + anti-windup
    if wI_g > 0
        xdot(26) = nz_err; xdot(27) = ny_err;
        if abs(de_v) > par.delta_max && sign(nz_err) == sign(de_v), xdot(26) = 0; end
        if abs(dr_v) > par.delta_max && sign(ny_err) == sign(dr_v), xdot(27) = 0; end
    end
else
    de_v = -KR_g * q_rate;
    dr_v = -KR_g * r;
end

if isfield(par, 'disable_pitch_yaw') && par.disable_pitch_yaw
    de_v = 0;
    dr_v = 0;
    xdot(26) = 0;
    xdot(27) = 0;
end

if dbg_on && ~isempty(dbg_last_t) && t == dbg_last_t
    dpy = isfield(par,'disable_pitch_yaw') && par.disable_pitch_yaw;
    fprintf('de_v=%+7.4f  dr_v=%+7.4f  KR=%6.4f  KA=%6.4f  disable_py=%d  ctrl=%d\n', ...
        de_v, dr_v, KR_g, KA_g, dpy, ctrl_active);
end

% Roll
if isfield(par, 'disable_angle_feedback') && par.disable_angle_feedback
    p_cmd = 0;
else
    eul = gnc_utils.quat2eul_zyx(quat);
    phi = eul(1);
    theta = eul(2);
    abs_theta = abs(theta);
    if abs_theta < deg2rad(70)
        e_phi = mod(-phi + pi, 2*pi) - pi;
        p_cmd = K_phi_g * e_phi;
    elseif abs_theta > deg2rad(80)
        p_cmd = 0;
    else
        e_phi = mod(-phi + pi, 2*pi) - pi;
        blend = (abs_theta - deg2rad(70)) / deg2rad(10);
        p_cmd = K_phi_g * e_phi * (1 - blend);
    end
end

e_p = p_cmd - p;
da_v_raw = Kp_roll_g * e_p + Ki_roll_g * int_ep;
da_v = max(-par.delta_max, min(par.delta_max, da_v_raw));

if abs(da_v_raw) > par.delta_max && sign(e_p) == sign(da_v_raw)
    xdot(14) = 0;
else
    xdot(14) = e_p;
end

%% Full disable (ballistic)
if isfield(par, 'disable_all_control') && par.disable_all_control
    de_v = 0;  dr_v = 0;  da_v = 0;
    xdot(14) = 0;
    xdot(26) = 0;
    xdot(27) = 0;
end

%% Fin mixing (roll priority allocation)
if isfield(par, 'roll_alloc_max') && par.roll_alloc_max > 0
    roll_lim = par.roll_alloc_max;
else
    roll_lim = par.delta_max;
end
da_alloc = max(-roll_lim, min(roll_lim, da_v));
remaining = par.delta_max - abs(da_alloc);
de_alloc = max(-remaining, min(remaining, de_v));
dr_alloc = max(-remaining, min(remaining, dr_v));

fin_cmd = [ de_alloc+da_alloc;  dr_alloc+da_alloc;
           -de_alloc+da_alloc; -dr_alloc+da_alloc];
fin_cmd = max(-par.delta_max, min(par.delta_max, fin_cmd));

if dbg_on && ~isempty(dbg_last_t) && t == dbg_last_t
    fprintf('  de_alloc=%+.4f dr_alloc=%+.4f da_alloc=%+.4f  fin_cmd=[%+.2f %+.2f %+.2f %+.2f] deg\n', ...
        de_alloc, dr_alloc, da_alloc, rad2deg(fin_cmd(1)), rad2deg(fin_cmd(2)), ...
        rad2deg(fin_cmd(3)), rad2deg(fin_cmd(4)));
end

%% Fin actuator dynamics (4 x 2nd order)
for i = 1:4
    idx = 13 + 2*i;
    [xdot(idx), xdot(idx+1)] = act_dyn(fin(i), fin_dot(i), fin_cmd(i), par);
end

%% Aero forces & moments
de_eff = (fin(1) - fin(3)) / 2;
dr_eff = (fin(2) - fin(4)) / 2;
da_eff = (fin(1) + fin(2) + fin(3) + fin(4)) / 4;

Fx = -qbar * par.Sref * par.CA0;
Fy =  qbar * par.Sref * (par.CYb*beta  + par.CYdr*dr_eff);
Fz = -qbar * par.Sref * (par.CNa*alpha + par.CNd*de_eff);

Ml = qbar*par.Sref*par.d * (par.Clp*p*par.d/(2*V) + par.Clda*da_eff);
Mm = qbar*par.Sref*par.d * (par.Cma*alpha + par.Cmd*de_eff + par.Cmq*q_rate*par.d/(2*V));
Mn = qbar*par.Sref*par.d * (par.Cnb*beta  + par.Cndr*dr_eff + par.Cnr*r*par.d/(2*V));

F_b = [Fx;Fy;Fz] + [T;0;0] + R_BN*[0;0;m*par.g];
M_b = [Ml;Mm;Mn];

%% Equations of motion
xdot(1:3) = vel_NED;
xdot(4:6) = R_NB * (F_b / m);

I_vec = [par.Ixx; par.Iyy; par.Izz];
xdot(11:13) = (M_b - cross(omega_b, I_vec.*omega_b)) ./ I_vec;

Omega = [0,-p,-q_rate,-r; p,0,r,-q_rate; q_rate,-r,0,p; r,q_rate,-p,0];
xdot(7:10) = 0.5 * Omega * quat;

end

function [dx1,dx2] = act_dyn(x1, x2, cmd, par)
    wn = par.wn_act;  z = par.zeta_act;
    cs = max(-par.delta_max, min(par.delta_max, cmd));
    dx1 = x2;
    dx2 = wn^2*(cs - x1) - 2*z*wn*x2;
    if abs(dx1) > par.delta_rate_max, dx1 = sign(dx1)*par.delta_rate_max; dx2 = 0; end
    if (x1 >= par.delta_max && dx1 > 0) || (x1 <= -par.delta_max && dx1 < 0), dx1 = 0; dx2 = 0; end
end
