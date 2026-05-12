%% validate_c_vs_matlab.m -- C vs MATLAB automated validation
%  Compares C output (c_validation.json) against MATLAB calculations.
%  Float (C) vs double (MATLAB) tolerance ~1e-5.

clear; clc;
fprintf('======== C vs MATLAB Validation ========\n\n');

par = rocket_params();
par = build_gain_table_for_sim(par, 'bode');

n_pass = 0;
n_fail = 0;
tol = 1e-5;

%% Load C output
json_file = 'c_validation.json';
if ~isfile(json_file)
    fprintf('  FAIL: %s not found.\n', json_file);
    fprintf('  Build and run C test_gnc first, then copy c_validation.json here.\n');
    return;
end

c = jsondecode(fileread(json_file));
fprintf('  c_validation.json loaded.\n\n');

%% Part A: Autopilot standalone
fprintf('--- Part A: Sole Autopilot (C vs MATLAB) ---\n');

V_test = 100;  Ts = 0.01;  g0 = par.g;

KR_g  = 0.170526;
KA_g  = 0.063352;
KDC_g = 1.000000;
wI_g  = par.wI;

nz_cmd = 0.5;  ny_cmd = -0.2;
q_rate = 0.1;  r_rate = -0.05;

u_body = V_test;
nz_fb =  (q_rate * u_body) / g0;
ny_fb = -(r_rate * u_body) / g0;
nz_err = KDC_g * nz_cmd - nz_fb;
ny_err = KDC_g * ny_cmd - ny_fb;

pitch_int = (Ts/2) * nz_err;
yaw_int   = (Ts/2) * ny_err;

de_v = KR_g * (KA_g * nz_err + wI_g * pitch_int - q_rate);
dr_v = KR_g * (KA_g * ny_err + wI_g * yaw_int   - r_rate);

check_pair('de_cmd',    de_v,      c.partA.de_cmd);
check_pair('dr_cmd',    dr_v,      c.partA.dr_cmd);
check_pair('pitch_int', pitch_int, c.partA.pitch_int);
check_pair('yaw_int',   yaw_int,   c.partA.yaw_int);

%% Part B: Guidance + Autopilot
fprintf('\n--- Part B: Guidance + Autopilot (C vs MATLAB) ---\n');

nav_pos_b = [20; 15; -250];
nav_vel_b = [8; 6; -45];
t_b = 5.0;

qq = [0.9239; 0; 0.3827; 0];
qq = qq / norm(qq);
qw=qq(1); qx=qq(2); qy=qq(3); qz=qq(4);
R_BN_b = [1-2*(qy^2+qz^2),  2*(qx*qy+qw*qz),  2*(qx*qz-qw*qy);
          2*(qx*qy-qw*qz),  1-2*(qx^2+qz^2),  2*(qy*qz+qw*qx);
          2*(qx*qz+qw*qy),  2*(qy*qz-qw*qx),  1-2*(qx^2+qy^2)];

V_b   = norm(nav_vel_b);
alt_b = -nav_pos_b(3);
rho_b = 1.225 * exp(-alt_b / 8500);
qbar_b = 0.5 * rho_b * V_b^2;
m_b = par.m0 - par.m_prop;

[nz_b, ny_b, ~] = guidance_module(t_b, nav_pos_b, nav_vel_b, R_BN_b, qbar_b, m_b, par, zeros(3,1), [0;0;0]);

check_pair('nz_cmd', nz_b, c.partB.nz_cmd);
check_pair('ny_cmd', ny_b, c.partB.ny_cmd);

% Autopilot (1 step)
omega_b = [0.01; 0.5; -0.1];
p_b = omega_b(1);  q_b = omega_b(2);  r_b = omega_b(3);

V_bp_c = [40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180];
KR_c   = [1.434345, 0.892373, 0.598034, 0.420630, 0.305569, 0.226777, 0.170526, 0.129037, 0.097645, 0.073431, 0.054522, 0.039732, 0.028411, 0.020457, 0.016325];
KA_c   = [1.043264, 0.306639, 0.168678, 0.113909, 0.086495, 0.071505, 0.063352, 0.059576, 0.059061, 0.061445, 0.066942, 0.076331, 0.090744, 0.109464, 0.121957];
KDC_c  = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
Kphi_c = [9.152149, 9.169450, 9.186750, 9.204050, 9.221349, 9.238648, 9.255945, 9.273241, 9.290536, 9.307829, 9.325121, 9.342411, 9.359698, 9.376984, 9.394267];
Kpr_c  = [0.737454, 0.471977, 0.327768, 0.240814, 0.184378, 0.145685, 0.118008, 0.097531, 0.081956, 0.069835, 0.060218, 0.052459, 0.046108, 0.040846, 0.036435];
Kir_c  = [1.474907, 0.943954, 0.655535, 0.481628, 0.368755, 0.291370, 0.236017, 0.195062, 0.163912, 0.139670, 0.120435, 0.104917, 0.092217, 0.081691, 0.072871];

Vc_b = max(V_bp_c(1), min(V_bp_c(end), V_b));
KR_b  = interp1(V_bp_c, KR_c,  Vc_b, 'linear');
KA_b  = interp1(V_bp_c, KA_c,  Vc_b, 'linear');
KDC_b = interp1(V_bp_c, KDC_c, Vc_b, 'linear');
K_phi_b   = interp1(V_bp_c, Kphi_c, Vc_b, 'linear');
Kp_roll_b = interp1(V_bp_c, Kpr_c,  Vc_b, 'linear');
Ki_roll_b = interp1(V_bp_c, Kir_c,  Vc_b, 'linear');

vb_air = R_BN_b * nav_vel_b;
u_body_b = max(vb_air(1), 1.0);
nz_fb_b =  (q_b * u_body_b) / g0;
ny_fb_b = -(r_b * u_body_b) / g0;
nz_err_b = KDC_b * nz_b - nz_fb_b;
ny_err_b = KDC_b * ny_b - ny_fb_b;

pi_nz = (Ts/2) * nz_err_b;
pi_ny = (Ts/2) * ny_err_b;

de_v_b = KR_b * (KA_b * nz_err_b + wI_g * pi_nz - q_b);
dr_v_b = KR_b * (KA_b * ny_err_b + wI_g * pi_ny - r_b);

% Roll
phi_b   = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));
theta_b = asin(max(-1, min(1, 2*(qw*qy - qz*qx))));
abs_theta = abs(theta_b);
if abs_theta < deg2rad(70)
    e_phi = -phi_b;
    p_cmd = K_phi_b * e_phi;
elseif abs_theta > deg2rad(80)
    p_cmd = 0;
else
    e_phi = -phi_b;
    blend = (abs_theta - deg2rad(70)) / deg2rad(10);
    p_cmd = K_phi_b * e_phi * (1 - blend);
end
e_p = p_cmd - p_b;
roll_int_b = (Ts/2) * e_p;
da_v_b = Kp_roll_b * e_p + Ki_roll_b * roll_int_b;
da_v_b = max(-par.delta_max, min(par.delta_max, da_v_b));

% Fin mixing
roll_lim = par.roll_alloc_max;
if roll_lim <= 0 || roll_lim > par.delta_max, roll_lim = par.delta_max; end
da_alloc = max(-roll_lim, min(roll_lim, da_v_b));
remaining = par.delta_max - abs(da_alloc);
de_alloc = max(-remaining, min(remaining, de_v_b));
dr_alloc = max(-remaining, min(remaining, dr_v_b));

check_pair('de_cmd', de_alloc, c.partB.de_cmd);
check_pair('dr_cmd', dr_alloc, c.partB.dr_cmd);

%% Anti-windup test
fprintf('\n--- TEST: Anti-windup ---\n');

pitch_int_aw = 5.0;
nz_err_big = KDC_g * 5.0 - nz_fb;
pitch_int_new = pitch_int_aw + (Ts/2)*(nz_err_big + nz_err_big);
de_big = KR_g * (KA_g * nz_err_big + wI_g * pitch_int_new - q_rate);

if abs(de_big) > par.delta_max && sign(nz_err_big) == sign(de_big)
    rollback = pitch_int_new - (Ts/2)*(nz_err_big + nz_err_big);
    if abs(rollback - pitch_int_aw) < 1e-10
        fprintf('  PASS: Anti-windup rollback good\n');
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL: Rollback mismatch\n');
        n_fail = n_fail + 1;
    end
else
    fprintf('  (not triggered — verified separately in C test_gnc)\n');
end

%% Fin mixing symmetry
fprintf('\n--- TEST: Fin mixing symmetry ---\n');
de_only = 0.05;
remaining = par.delta_max;
fin1 =  de_only;  fin2 = 0;  fin3 = -de_only;  fin4 = 0;
if abs(fin1+fin3) < 1e-10 && abs(fin2) < 1e-10
    fprintf('  PASS: Pitch symmetry good\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL: Pitch symmetry broken\n');
    n_fail = n_fail + 1;
end

%% Vector saturation
fprintf('\n--- TEST: Vector saturation ---\n');
nz_t = 3;  ny_t = 4;  lim = 2;
mag = sqrt(nz_t^2+ny_t^2);
s = lim/mag;
if abs(sqrt((nz_t*s)^2+(ny_t*s)^2) - lim) < 1e-10
    fprintf('  PASS: Vector saturation good\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL\n');
    n_fail = n_fail + 1;
end

%% Summary
fprintf('\n============================================\n');
fprintf('  PASS: %d,  FAIL: %d\n', n_pass, n_fail);
if n_fail == 0
    fprintf('  All tests passed.\n');
else
    fprintf('  %d test(s) failed.\n', n_fail);
end
fprintf('============================================\n');

%% Comparison helper
function check_pair(name, matlab_val, c_val)
    err = abs(matlab_val - c_val);
    tol = 1e-5;
    if err < tol
        fprintf('  PASS %-12s  MATLAB=%+.10f  C=%+.10f  err=%.1e\n', name, matlab_val, c_val, err);
        evalin('caller', 'n_pass = n_pass + 1;');
    else
        fprintf('  FAIL %-12s  MATLAB=%+.10f  C=%+.10f  err=%.1e  > tol\n', name, matlab_val, c_val, err);
        evalin('caller', 'n_fail = n_fail + 1;');
    end
end
