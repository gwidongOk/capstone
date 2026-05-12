%% run_6dof_discrete.m
%  Discrete-time 6-DOF simulation with 5-phase state machine.
%  Output format matches run_6dof.m (continuous version).
%
%  Architecture:
%    Plant (RK4 @ dt_plant) -> Sensor (ZOH @ Ts) -> Controller (Ts)
%    -> Actuator cmd -> Plant ...
%
%  Phases:
%    RAIL    -> on launch rail, fins locked to zero
%    BOOST   -> off rail, motor burning, rate damp only
%    GUIDE   -> guidance + full control active
%    CPA     -> closest point of approach passed, ballistic coast
%    DESCENT -> apogee passed, parachute region

clear; clc; close all; drawnow;

%% ========== OPTIONS ==========
Ts_ctrl  = 0.0025;      % 400 Hz
dt_plant = 0.0005;      % 2 kHz RK4

%% ========== 1. PARAMETERS ==========
par = rocket_params();

% --- Wind ---
par.wind_model = 'powerlaw';
par.wind_NED   = [3; 0; 0];
par.wind_est   = [0; 0; 0];

% --- Power-law parameters (used if wind_model = 'powerlaw' or 'powerlaw_dryden') ---
par.wind_ref_speed  = 2.5;
par.wind_ref_height = 10.0;
par.wind_direction  = deg2rad(315);
par.wind_alpha      = 1/7;
par.wind_h_min      = 1.0;

if norm(par.wind_est) > 0.01
    mode_str = 'Wind Drift Lead';
else
    mode_str = 'no wind compensation';
end

%% ========== 2. GAINS ==========
par = build_gain_table_for_sim(par, 'bode');
disc = discretize_controller(par, Ts_ctrl);

fprintf('\n====== DISCRETE 6-DOF SIMULATION ======\n');
fprintf('Control: %.0f Hz, Plant: %.0f Hz\n', 1/Ts_ctrl, 1/dt_plant);
fprintf('Target: [%.0f, %.0f, %.0f] m\n', par.target_NED);
fprintf('Mode: %s\n\n', mode_str);

%% ========== 3. INITIAL CONDITIONS ==========
q0_init = gnc_utils.eul2quat_zyx(0, par.pitch0, par.yaw0);
R_BN0   = gnc_utils.quat2dcm_bn(q0_init);
v_NED0  = R_BN0' * [par.V0; 0; 0];

x_plant = zeros(13,1);
x_plant(4:6)  = v_NED0;
x_plant(7:10) = q0_init(:);

ctrl_state = discrete_sim_utils.init_ctrl_state(disc);
act_state  = zeros(8,1);
x_guid     = zeros(3,1);

%% ========== 4. SIMULATE ==========
N_ppc = round(Ts_ctrl / dt_plant);
N_ctrl = ceil(par.t_sim_max / Ts_ctrl);

% Preallocate
hist_t     = zeros(N_ctrl+1,1);
hist_pos   = zeros(N_ctrl+1,3);
hist_vel   = zeros(N_ctrl+1,3);
hist_quat  = zeros(N_ctrl+1,4);
hist_omega = zeros(N_ctrl+1,3);
hist_fin   = zeros(N_ctrl+1,4);
hist_nzcmd = zeros(N_ctrl+1,1);
hist_nycmd = zeros(N_ctrl+1,1);

hist_pos(1,:)   = x_plant(1:3)';
hist_vel(1,:)   = x_plant(4:6)';
hist_quat(1,:)  = x_plant(7:10)';
hist_omega(1,:) = x_plant(11:13)';

fprintf('------ 5-Phase State Machine ------\n');
tic;

% Phase enum
PHASE_RAIL = 0; PHASE_BOOST = 1; PHASE_GUIDE = 2;
PHASE_CPA = 3; PHASE_DESCENT = 4;
phase_names = {'RAIL','BOOST','GUIDE','CPA','DESCENT'};

phase = PHASE_RAIL;
t_phase_enter = 0;
prev_vclose = 0;
prev_vD = -1;
k_cpa = 0;
hist_phase = zeros(N_ctrl+1,1);

for k = 1:N_ctrl
    tn = (k-1)*Ts_ctrl;

    % Sensor read
    pos = x_plant(1:3); vel = x_plant(4:6);
    qt = x_plant(7:10); wb = x_plant(11:13);
    qn = norm(qt); if qn>1e-10, qt=qt/qn; end
    RB = gnc_utils.quat2dcm_bn(qt);

    % Airspeed
    alt = max(-pos(3),0);
    w_now = discrete_sim_utils.get_wind(tn, alt, par);
    vel_air_NED = vel - w_now;
    vb_air = RB*vel_air_NED;
    V_air = max(norm(vb_air), 1.0);
    rho = par.rho0*exp(-alt/par.h_scale);
    qbar = 0.5*rho*V_air^2;

    % Mass/thrust
    if tn < par.t_burn
        m = par.m0 - par.mdot*tn; T = par.T_avg;
    else
        m = par.m0 - par.m_prop; T = 0;
    end

    % ===== Phase transition (one-directional) =====
    old_phase = phase;
    switch phase
        case PHASE_RAIL
            if tn > 0.01
                phase = PHASE_BOOST;
                t_phase_enter = tn;
            end
        case PHASE_BOOST
            aero_ok = (qbar > par.qbar_min_ctrl);
            if isfield(par,'V_ctrl_on') && par.V_ctrl_on > 0
                aero_ok = (V_air > par.V_ctrl_on);
            end
            if aero_ok && tn > par.t_guide_on
                phase = PHASE_GUIDE;
                t_phase_enter = tn;
            end
        case PHASE_GUIDE
            rv = pos - par.target_NED;
            Rt = norm(rv);
            if Rt > 0.1
                vc = dot(rv, vel)/Rt;
                if vc > 0 && prev_vclose < 0 && tn > par.t_guide_on + 1.0
                    phase = PHASE_CPA;
                    t_phase_enter = tn;
                    k_cpa = k+1;
                end
                prev_vclose = vc;
            end
        case PHASE_CPA
            vD = vel(3);
            if prev_vD < 0 && vD >= 0 && tn > t_phase_enter + 0.5
                phase = PHASE_DESCENT;
                t_phase_enter = tn;
            end
    end

    if phase ~= old_phase
        fprintf('  t=%.3f s: %s -> %s\n', tn, phase_names{old_phase+1}, phase_names{phase+1});
    end

    % ===== Phase-dependent GNC =====
    nzc = 0; nyc = 0;
    switch phase
        case PHASE_RAIL
            fc = [0;0;0;0];

        case PHASE_BOOST
            % Rate damping only (nz_cmd = ny_cmd = 0)
            [fc, ctrl_state] = discrete_sim_utils.autopilot(tn, 0, 0, ...
                wb, qt, V_air, qbar, par, disc, ctrl_state, vel_air_NED);

        case PHASE_GUIDE
            % Full guidance + control
            [nzc, nyc, xdg] = guidance_module(tn, pos, vel, RB, qbar, m, par, x_guid);
            x_guid = x_guid + Ts_ctrl*xdg;
            [fc, ctrl_state] = discrete_sim_utils.autopilot(tn, nzc, nyc, ...
                wb, qt, V_air, qbar, par, disc, ctrl_state, vel_air_NED);

        case PHASE_CPA
            % Ballistic coast — fins locked to zero (matches C gnc_main.c)
            fc = [0;0;0;0];

        case PHASE_DESCENT
            fc = [0;0;0;0];
    end

    % Plant integration (w_now already computed above)
    for j = 1:N_ppc
        act_state = discrete_sim_utils.actuator_step(act_state, fc, par, dt_plant);
        fa = act_state([1,3,5,7]);
        x_plant = discrete_sim_utils.rk4_plant_step(tn+(j-1)*dt_plant, x_plant, fa, T, m, par, dt_plant, w_now);
        qn = norm(x_plant(7:10)); if qn>1e-10, x_plant(7:10)=x_plant(7:10)/qn; end
    end

    % Store
    hist_t(k+1)       = tn + Ts_ctrl;
    hist_pos(k+1,:)   = x_plant(1:3)';
    hist_vel(k+1,:)   = x_plant(4:6)';
    hist_quat(k+1,:)  = x_plant(7:10)';
    hist_omega(k+1,:) = x_plant(11:13)';
    hist_fin(k+1,:)   = act_state([1,3,5,7])';
    hist_nzcmd(k+1)   = nzc;
    hist_nycmd(k+1)   = nyc;
    hist_phase(k+1)   = phase;

    % v_D tracking for apogee detection
    prev_vD = vel(3);

    % Termination conditions
    if phase == PHASE_DESCENT
        fprintf('  DESCENT phase reached, stopping.\n');
        break;
    end
    if tn > 1.0 && -x_plant(3) < 0
        fprintf('  Ground impact at t=%.3f s\n', tn+Ts_ctrl);
        break;
    end
end

wall_time = toc;
k_end = k+1;

% Trim
hist_t     = hist_t(1:k_end);
hist_pos   = hist_pos(1:k_end,:);
hist_vel   = hist_vel(1:k_end,:);
hist_quat  = hist_quat(1:k_end,:);
hist_omega = hist_omega(1:k_end,:);
hist_fin   = hist_fin(1:k_end,:);
hist_nzcmd = hist_nzcmd(1:k_end);
hist_nycmd = hist_nycmd(1:k_end);

fprintf('Done: %.2fs wall, %.1fs sim, %d steps\n\n', wall_time, hist_t(end), k_end);

%% ========== 5. POST-PROCESS ==========
t = hist_t;
pos = hist_pos;  vel = hist_vel;
omg = hist_omega; fin = hist_fin;
alt = -pos(:,3);
V_h = vecnorm(vel,2,2);
N_t = length(t);

% Euler angles, alpha, beta
eul_h   = zeros(N_t,3);
alpha_h = zeros(N_t,1);
beta_h  = zeros(N_t,1);
nz_fb_h = zeros(N_t,1);
ny_fb_h = zeros(N_t,1);

for k = 1:N_t
    qq = hist_quat(k,:)'/norm(hist_quat(k,:));
    eul_h(k,:) = gnc_utils.quat2eul_zyx(qq);
    R = gnc_utils.quat2dcm_bn(qq);
    vb = R * vel(k,:)';
    uu = max(vb(1), 0.1);
    alpha_h(k) = atan2(vb(3), uu);
    beta_h(k)  = asin(max(-1,min(1, vb(2)/max(norm(vb),1))));
    nz_fb_h(k) = omg(k,2)*uu/par.g;
    ny_fb_h(k) = -omg(k,3)*uu/par.g;
end

slant = vecnorm(pos - par.target_NED', 2, 2);
[min_miss, i_min] = min(slant);
[max_alt, i_apo]  = max(alt);

%% ========== 6. TEXT OUTPUT (matches run_6dof.m) ==========
label = sprintf('Discrete (Ts=%.0fms) -- %s', Ts_ctrl*1000, mode_str);

fprintf('============ %s PERFORMANCE ============\n', upper(label));
fprintf('--- Trajectory ---\n');
fprintf('  Apogee:       %.1f m at t=%.1f s\n', max_alt, t(i_apo));
fprintf('  Max speed:    %.1f m/s\n', max(V_h));
fprintf('  Min miss:     %.1f m at t=%.1f s\n', min_miss, t(i_min));
fprintf('  Guid law:     %s\n', par.guid_law);
fprintf('\n--- Stability ---\n');
fprintf('  Max |alpha|: %.1f deg\n', max(abs(rad2deg(alpha_h))));
fprintf('  Max |beta|:  %.2f deg\n', max(abs(rad2deg(beta_h))));
fprintf('  Max |q|:     %.1f deg/s\n', max(abs(rad2deg(omg(:,2)))));
fprintf('\n--- Actuator ---\n');
for i = 1:4
    fprintf('  Fin%d: %.1f deg (%.0f%%)\n', i, ...
        max(abs(rad2deg(fin(:,i)))), max(abs(rad2deg(fin(:,i))))/15*100);
end
fprintf('\n--- PASS/FAIL ---\n');
check_pass('Miss < 10m',       min_miss < 10);
check_pass('|alpha| < 15 deg', max(abs(rad2deg(alpha_h))) < 15);
check_pass('|q| < 50 deg/s',   max(abs(rad2deg(omg(:,2)))) < 50);
check_pass('No fin saturation', all(max(abs(fin)) < par.delta_max*0.95));
fprintf('================================================\n\n');

%% ========== 7. FIGURE 1 -- Flight Results ==========
figure('Name', label, 'Position', [40 40 1500 900], 'Renderer','painters');

subplot(2,3,1);
plot3(pos(:,1), pos(:,2), alt, 'b-','LineWidth',1.5); hold on;
plot3(par.target_NED(1), par.target_NED(2), -par.target_NED(3), ...
    'rp','MarkerSize',15,'MarkerFaceColor','r');
plot3(0,0,0,'go','MarkerSize',10,'MarkerFaceColor','g');
plot3(pos(i_apo,1), pos(i_apo,2), alt(i_apo), 'c^','MarkerSize',8,'MarkerFaceColor','c');
plot3(pos(i_min,1), pos(i_min,2), alt(i_min), 'mv','MarkerSize',8,'MarkerFaceColor','m');
xlabel('North (m)'); ylabel('East (m)'); zlabel('Alt (m)');
title('3D Trajectory'); grid on; axis equal; view(-45,30);
legend('Traj','Target','Launch','Apogee','CPA','Location','best');

subplot(2,3,2);
plot(t, slant, 'b-','LineWidth',1.5); hold on;
plot(t(i_min), min_miss, 'rv','MarkerSize',10,'MarkerFaceColor','r');
xline(par.t_burn,'r--','Burnout');
xline(par.t_guide_on,'g--','Guid ON');
if k_cpa > 0, xline(t(k_cpa),'m--','CPA'); end
xlabel('Time (s)'); ylabel('Range (m)');
title(sprintf('Slant Range (min=%.1f m)', min_miss)); grid on;

subplot(2,3,3);
yyaxis left; plot(t, V_h, 'b-','LineWidth',1.5); ylabel('Speed (m/s)');
yyaxis right; plot(t, alt, 'r-','LineWidth',1.2); ylabel('Altitude (m)');
xline(par.t_burn,'k--','Burnout');
xlabel('Time (s)'); title('Speed & Altitude'); grid on;

subplot(2,3,4);
plot(t, rad2deg(eul_h(:,1)),'r-', t, rad2deg(eul_h(:,2)),'b-', ...
    t, rad2deg(eul_h(:,3)),'g-','LineWidth',1.2);
xlabel('Time (s)'); ylabel('deg'); title('Euler Angles'); grid on;
legend('\phi','\theta','\psi');

subplot(2,3,5);
plot(t, rad2deg(omg(:,1)),'r-', t, rad2deg(omg(:,2)),'b-', ...
    t, rad2deg(omg(:,3)),'g-','LineWidth',1.2);
xlabel('Time (s)'); ylabel('deg/s'); title('Body Rates'); grid on;
legend('p','q','r');

subplot(2,3,6);
plot(t, rad2deg(fin(:,1)),'b-', t, rad2deg(fin(:,2)),'r-', ...
    t, rad2deg(fin(:,3)),'g-', t, rad2deg(fin(:,4)),'m-','LineWidth',1.0);
yline(15,'k--'); yline(-15,'k--');
xlabel('Time (s)'); ylabel('deg'); title('Fin Deflections'); grid on;
legend('Fin1','Fin2','Fin3','Fin4');

sgtitle(sprintf('6-DOF -- %s | miss = %.1f m', label, min_miss));
drawnow;

%% ========== 8. FIGURE 2 -- GNC Diagnostics ==========
figure('Name', sprintf('GNC Diag [%s]', label), ...
    'Position', [80 40 1500 900], 'Renderer','painters');

subplot(2,3,1);
plot(t, hist_nzcmd,'b-','LineWidth',1.3); hold on;
plot(t, nz_fb_h,'b:','LineWidth',1.3);
plot(t, hist_nycmd,'r-','LineWidth',1.3);
plot(t, ny_fb_h,'r:','LineWidth',1.3);
yline(0,'k-','LineWidth',0.5);
xline(par.t_guide_on,'g--','Guid ON');
if k_cpa > 0, xline(t(k_cpa),'m--','CPA'); end
xlabel('Time (s)'); ylabel('g');
title('Accel: cmd vs fb'); grid on;
legend('nz_{cmd}','nz_{fb}','ny_{cmd}','ny_{fb}','Location','best');

subplot(2,3,2);
plot(t, rad2deg(alpha_h),'b-', t, rad2deg(beta_h),'r-','LineWidth',1.2);
xlabel('Time (s)'); ylabel('deg');
title('\alpha / \beta'); grid on; legend('\alpha','\beta');

subplot(2,3,3);
rho_h = par.rho0*exp(-max(alt,0)/par.h_scale);
qbar_h = 0.5*rho_h.*V_h.^2;
plot(t, qbar_h, 'b-','LineWidth',1.5);
yline(par.qbar_min_ctrl,'r--','Ctrl ON');
xlabel('Time (s)'); ylabel('Pa'); title('Dynamic Pressure'); grid on;

subplot(2,3,4);
v_close = zeros(N_t,1);
for k = 1:N_t
    rv = pos(k,:)'-par.target_NED;
    Rk = norm(rv);
    if Rk>0.1, v_close(k) = dot(rv, vel(k,:)')/Rk; end
end
plot(t, v_close,'b-','LineWidth',1.5); hold on;
yline(0,'k--');
if k_cpa > 0, plot(t(k_cpa), v_close(k_cpa),'rv','MarkerSize',10,'MarkerFaceColor','r'); end
xlabel('Time (s)'); ylabel('m/s');
title('Closing Velocity'); grid on;

subplot(2,3,5);
de_eff = (fin(:,1) - fin(:,3))/2;
dr_eff = (fin(:,2) - fin(:,4))/2;
da_eff = mean(fin, 2);
plot(t, rad2deg(de_eff),'b-', t, rad2deg(dr_eff),'r-', ...
    t, rad2deg(da_eff),'g-','LineWidth',1.2);
xlabel('Time (s)'); ylabel('deg');
title('\delta_e / \delta_r / \delta_a'); grid on;
legend('\delta_e','\delta_r','\delta_a');

subplot(2,3,6);
% Guidance command magnitude
cmd_mag = sqrt(hist_nzcmd.^2 + hist_nycmd.^2);
plot(t, cmd_mag, 'b-','LineWidth',1.2);
xlabel('Time (s)'); ylabel('|cmd| (g)');
title('Guidance Command Magnitude'); grid on;

sgtitle(sprintf('GNC Diagnostics -- %s', label));
drawnow;

%% ========== HELPER ==========
function check_pass(name, passed)
    if passed, fprintf('  [PASS] %s\n', name);
    else,      fprintf('  [FAIL] %s\n', name); end
end