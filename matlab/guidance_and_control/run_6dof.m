%% run_6dof.m -- Continuous 6-DOF simulation with 2-phase state machine.

clear; clc; close all; drawnow;

%% 1. Load parameters
par = rocket_params();
par.wind_NED = [3; 0; 0];
par.wind_model      = 'powerlaw';
par.wind_ref_speed  = 3.0;
par.wind_ref_height = 10.0;
par.wind_direction  = 0;
par.wind_alpha      = 1/7;
par.wind_h_min      = 1.0;
par.dryden_W20      = 3.0;
par.dryden_h_ref    = 500;
par.dryden_V_ref    = 100;

[par.gust_t, par.gust_NED, gust_interps] = generate_dryden_gust(par, par.t_sim_max, 0.001, 42);
par.gust_interp_N = gust_interps.N;
par.gust_interp_E = gust_interps.E;
par.gust_interp_D = gust_interps.D;

par.wind_est = [0; 0; 0];

if norm(par.wind_est) > 0.01
    mode_str = 'Wind Drift Lead';
else
    mode_str = 'no wind compensation';
end

fprintf('Parameters loaded from rocket_params.m\n');
fprintf('  Target: [%.0f, %.0f, %.0f] m (NED)\n', par.target_NED);
fprintf('  Launch: pitch=%.1f deg, yaw=%.1f deg\n', rad2deg(par.pitch0), rad2deg(par.yaw0));
fprintf('  Guidance: %s\n', par.guid_law);
fprintf('  Wind: [%.1f, %.1f, %.1f] m/s\n', par.wind_NED);
fprintf('  Mode: %s\n\n', mode_str);

%% 2. Build gain table
methods = {'bode'};
labels  = {'Frequency Response Design'};
N_methods = length(methods);
par_set = cell(1, N_methods);

for m = 1:N_methods
    par_set{m} = build_gain_table_for_sim(par, methods{m});
    par_set{m}.wind_est = par.wind_est;
    par_set{m}.wind_NED = par.wind_NED;
    fprintf('  %s gain table ready (V = %s m/s)\n', labels{m}, mat2str(par.sched_V));
end
fprintf('\n');

%% 3. Initial conditions
q0_init = gnc_utils.eul2quat_zyx(0, par.pitch0, par.yaw0);
R_BN0   = gnc_utils.quat2dcm_bn(q0_init);
v_NED0  = R_BN0' * [par.V0; 0; 0];

x0 = zeros(27, 1);
x0(4:6)  = v_NED0;
x0(7:10) = q0_init(:);

%% 4. Simulate & plot
results = cell(1, N_methods);

for m = 1:N_methods
    par_m = par_set{m};
    par_m.phase = 1;

    TIMEOUT_SEC = 30;
    run_timer = tic;

    opts1 = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, ...
        'Events', @(t,x) gnc_utils.flight_events(t, x, par_m), ...
        'OutputFcn', @(t,y,flag) timeout_check(t, y, flag, run_timer, TIMEOUT_SEC));

    fprintf('========= %s Simulation [%s] =========\n', labels{m}, mode_str);
    fprintf('------ Phase 1: Full Control ------\n');
    tic;
    [t1, X1, te1, ~, ie1] = ode45(@(t,x) eom_6dof_wind(t, x, par_m), ...
        [0 par_m.t_sim_max], x0, opts1);
    wall_time_1 = toc;

    event_names = {'CPA', 'Apogee', 'Ground'};
    fprintf('Phase 1 events detected: %d\n', length(ie1));
    for k = 1:length(ie1)
        fprintf('  t=%.3f s, event #%d (%s)\n', te1(k), ie1(k), event_names{ie1(k)});
    end

    % Phase 2: Post-CPA coast
    run_phase2 = ~isempty(ie1) && ie1(end) == 1;

    if run_phase2
        fprintf('\nCPA at t=%.3f s -> Phase 2 (Pitch/Yaw OFF, Roll active)\n', te1(end));

        par_m2 = par_m;
        par_m2.phase = 2;
        par_m2.disable_pitch_yaw      = true;
        par_m2.disable_angle_feedback = false;

        x_phase2_init = X1(end,:)';
        x_phase2_init(26) = 0;
        x_phase2_init(27) = 0;

        TIMEOUT_SEC = 3;
        run_timer = tic;

        opts2 = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, ...
        'Events', @(t,x) gnc_utils.flight_events(t, x, par_m2), ...
        'OutputFcn', @(t,y,flag) timeout_check(t, y, flag, run_timer, TIMEOUT_SEC));

        tic;
        [t2, X2, te2, ~, ie2] = ode45(@(t,x) eom_6dof_wind(t, x, par_m2), ...
            [te1(end) par_m.t_sim_max], x_phase2_init, opts2);
        wall_time_2 = toc;

        fprintf('Phase 2 events detected: %d\n', length(ie2));
        for k = 1:length(ie2)
            fprintf('  t=%.3f s, event #%d (%s)\n', te2(k), ie2(k), event_names{ie2(k)});
        end

        t = [t1; t2(2:end)];
        X = [X1; X2(2:end,:)];

        if ~isempty(ie2)
            fprintf('Terminated by: %s at t=%.2f s\n', event_names{ie2(end)}, te2(end));
        end
        fprintf('Done: %.2fs wall (P1=%.2fs, P2=%.2fs), %.1fs sim, %d+%d=%d steps\n\n', ...
            wall_time_1 + wall_time_2, wall_time_1, wall_time_2, ...
            t(end), length(t1), length(t2)-1, length(t));
    else
        t = t1; X = X1;
        if ~isempty(ie1)
            fprintf('Terminated by: %s at t=%.2f s (no CPA, Phase 2 skipped)\n', ...
                event_names{ie1(end)}, te1(end));
        end
        fprintf('Done: %.2fs wall, %.1fs sim, %d steps\n\n', ...
            wall_time_1, t(end), length(t));
    end

    sim_postprocess(t, X, par_m, [labels{m} ' — ' mode_str], m);
    results{m} = struct('t', t, 'X', X, 'par', par_m);
end

t_sim = results{1}.t;  X_sim = results{1}.X;  par_sim = results{1}.par;

drawnow;
fprintf('\n====== COMPLETE ======\n');
fprintf('Mode: %s\n', mode_str);
fprintf('Workspace: (t_sim, X_sim, par_sim)\n');
fprintf('Animation: animate_trajectory(t_sim, X_sim, par_sim)\n');

%% Local functions

function sim_postprocess(t, X, par, label, fig_offset)

    pos = X(:,1:3);  vel = X(:,4:6);
    quat_h = X(:,7:10);  omg = X(:,11:13);
    V_h = vecnorm(vel,2,2);  alt = -pos(:,3);
    N_t = length(t);

    eul_h    = zeros(N_t,3);
    alpha_h  = zeros(N_t,1);
    beta_h   = zeros(N_t,1);
    nz_cmd_h = zeros(N_t,1);
    ny_cmd_h = zeros(N_t,1);
    nz_fb_h  = zeros(N_t,1);
    ny_fb_h  = zeros(N_t,1);
    nz_err_h = zeros(N_t,1);
    ny_err_h = zeros(N_t,1);

    has_gain_table = isfield(par, 'gain_table') && isfield(par.gain_table, 'KDC');
    if has_gain_table
        V_bp = par.sched_V;
    end

    for k = 1:N_t
        qq = quat_h(k,:)'/norm(quat_h(k,:));
        eul_h(k,:) = gnc_utils.quat2eul_zyx(qq);
        R = gnc_utils.quat2dcm_bn(qq);
        vb = R * vel(k,:)';
        uu = max(vb(1), 0.1);
        alpha_h(k) = atan2(vb(3), uu);
        beta_h(k)  = asin(max(-1,min(1, vb(2)/max(norm(vb),1))));
        nz_fb_h(k) = omg(k,2)*uu/par.g;
        ny_fb_h(k) = -omg(k,3)*uu/par.g;

        if t(k) > par.t_guide_on
            rho_k = par.rho0 * exp(-max(alt(k),0)/par.h_scale);
            qbar_k = 0.5 * rho_k * max(norm(vel(k,:)),1)^2;
            m_k = max(par.m0-par.mdot*t(k), par.m0-par.m_prop);
            [nz_k, ny_k, ~] = guidance_module(t(k), pos(k,:)', vel(k,:)', ...
                R, qbar_k, m_k, par, X(k,23:25)');
            nz_cmd_h(k) = nz_k;
            ny_cmd_h(k) = ny_k;

            if has_gain_table
                Vc = min(max(V_h(k), V_bp(1)), V_bp(end));
                KDC_k = interp1(V_bp, par.gain_table.KDC, Vc);
            else
                KDC_k = par.KDC;
            end
            nz_err_h(k) = KDC_k * nz_k - nz_fb_h(k);
            ny_err_h(k) = KDC_k * ny_k - ny_fb_h(k);
        end
    end

    slant = vecnorm(pos - par.target_NED', 2, 2);
    [min_slant, i_min] = min(slant);
    [~, i_apo] = max(alt);

    v_close = zeros(N_t, 1);
    for k = 1:N_t
        r_vec = pos(k,:)' - par.target_NED;
        R_k = norm(r_vec);
        if R_k > 0.1, v_close(k) = dot(r_vec, vel(k,:)') / R_k; end
    end

    guid_name = 'Pure Pursuit';
    if isfield(par, 'guid_law'), guid_name = par.guid_law; end

    fprintf('============ %s PERFORMANCE ============\n', upper(label));
    fprintf('--- Trajectory ---\n');
    fprintf('  Apogee:       %.1f m at t=%.1f s\n', alt(i_apo), t(i_apo));
    fprintf('  Max speed:    %.1f m/s\n', max(V_h));
    fprintf('  Min miss:     %.1f m at t=%.1f s\n', min_slant, t(i_min));
    fprintf('  Guid law:     %s\n', guid_name);
    fprintf('\n--- Stability ---\n');
    fprintf('  Max |alpha|: %.1f deg\n', max(abs(rad2deg(alpha_h))));
    fprintf('  Max |beta|:  %.2f deg\n', max(abs(rad2deg(beta_h))));
    fprintf('  Max |q|:     %.1f deg/s\n', max(abs(rad2deg(omg(:,2)))));
    fprintf('\n--- Actuator ---\n');
    for i = 1:4
        idx = 15 + (i-1)*2;
        fprintf('  Fin%d: %.1f deg (%.0f%%)\n', i, ...
                max(abs(rad2deg(X(:,idx)))), ...
                max(abs(rad2deg(X(:,idx))))/15*100);
    end

    fprintf('\n--- PASS/FAIL ---\n');
    check_pass('Miss < 10m',         min_slant < 10);
    check_pass('|alpha| < 15 deg',   max(abs(rad2deg(alpha_h))) < 15);
    check_pass('|q| < 50 deg/s',     max(abs(rad2deg(omg(:,2)))) < 50);
    check_pass('No fin saturation',  all(max(abs(X(:,15:2:21))) < par.delta_max*0.95));
    fprintf('================================================\n\n');

    % CPA diagnostic window
    t_cpa = t(i_min);
    win = (t >= par.t_guide_on) & (t <= t_cpa - 0.1);
    if sum(win) >= 5
        de_eff_h = (X(:,15) - X(:,19))/2;
        dr_eff_h = (X(:,17) - X(:,21))/2;

        fprintf('--- CPA Diagnostic Window ---\n');
        fprintf('  Window: t = [%.2f, %.2f] s  (t_cpa = %.2f s, N = %d samples)\n\n', ...
                par.t_guide_on, t_cpa - 0.1, t_cpa, sum(win));

        fprintf('  [Guidance]   cmd = signed mean |  |cmd| mean / max\n');
        fprintf('    nz_cmd:  %+7.3f g     | %6.3f g / %6.3f g\n', ...
                mean(nz_cmd_h(win)), mean(abs(nz_cmd_h(win))), max(abs(nz_cmd_h(win))));
        fprintf('    ny_cmd:  %+7.3f g     | %6.3f g / %6.3f g\n\n', ...
                mean(ny_cmd_h(win)), mean(abs(ny_cmd_h(win))), max(abs(ny_cmd_h(win))));

        fprintf('  [Achieved]   fb  = signed mean |  |fb|  mean / max\n');
        fprintf('    nz_fb:   %+7.3f g     | %6.3f g / %6.3f g\n', ...
                mean(nz_fb_h(win)), mean(abs(nz_fb_h(win))), max(abs(nz_fb_h(win))));
        fprintf('    ny_fb:   %+7.3f g     | %6.3f g / %6.3f g\n\n', ...
                mean(ny_fb_h(win)), mean(abs(ny_fb_h(win))), max(abs(ny_fb_h(win))));

        fprintf('  [Error = KDC*cmd - fb]  signed mean |  |err| mean\n');
        fprintf('    nz_err:  %+7.3f g     | %6.3f g\n', ...
                mean(nz_err_h(win)), mean(abs(nz_err_h(win))));
        fprintf('    ny_err:  %+7.3f g     | %6.3f g\n\n', ...
                mean(ny_err_h(win)), mean(abs(ny_err_h(win))));

        fprintf('  [Fin]        signed mean | |.| mean / max\n');
        fprintf('    de_eff:  %+6.2f deg    | %5.2f / %5.2f deg\n', ...
                rad2deg(mean(de_eff_h(win))), rad2deg(mean(abs(de_eff_h(win)))), ...
                rad2deg(max(abs(de_eff_h(win)))));
        fprintf('    dr_eff:  %+6.2f deg    | %5.2f / %5.2f deg\n\n', ...
                rad2deg(mean(dr_eff_h(win))), rad2deg(mean(abs(dr_eff_h(win)))), ...
                rad2deg(max(abs(dr_eff_h(win)))));

        i_start = find(win, 1, 'first');
        i_end   = find(win, 1, 'last');
        fprintf('  [wI integrators]  start -> end  (delta)\n');
        fprintf('    int_nz:  %+.4f -> %+.4f rad  (d = %+.4f)\n', ...
                X(i_start,26), X(i_end,26), X(i_end,26) - X(i_start,26));
        fprintf('    int_ny:  %+.4f -> %+.4f rad  (d = %+.4f)\n', ...
                X(i_start,27), X(i_end,27), X(i_end,27) - X(i_start,27));
        fprintf('================================================\n\n');
    else
        fprintf('--- CPA Diagnostic Window too short (N<5), skipping ---\n\n');
    end

    % Figure 1: Flight Results
    title_str = sprintf('6-DOF — %s | miss = %.1f m', label, min_slant);

    figure('Name', sprintf('Flight [%s]', label), ...
           'Position', [40+fig_offset*60, 40+fig_offset*40, 1500, 900], ...
           'Renderer', 'painters');

    subplot(2,3,1);
    plot3(pos(:,1), pos(:,2), alt, 'b-','LineWidth',1.5); hold on;
    plot3(par.target_NED(1), par.target_NED(2), -par.target_NED(3), ...
          'rp','MarkerSize',15,'MarkerFaceColor','r');
    plot3(0,0,0,'go','MarkerSize',10,'MarkerFaceColor','g');
    plot3(pos(i_apo,1), pos(i_apo,2), alt(i_apo), 'c^','MarkerSize',8,'MarkerFaceColor','c');
    plot3(pos(i_min,1), pos(i_min,2), alt(i_min), 'mv','MarkerSize',8,'MarkerFaceColor','m');
    xlabel('North (m)'); ylabel('East (m)'); zlabel('Alt (m)');
    title('3D Trajectory'); grid on; axis equal; view(-45, 30);
    legend('Traj','Target','Launch','Apogee','CPA','Location','best');

    subplot(2,3,2);
    plot(t, slant, 'b-', 'LineWidth', 1.5); hold on;
    plot(t(i_min), min_slant, 'rv', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xline(par.t_burn, 'r--', 'Burnout');
    xline(par.t_guide_on, 'g--', 'Guid ON');
    xlabel('Time (s)'); ylabel('Range (m)');
    title(sprintf('Slant Range (min=%.1f m)', min_slant)); grid on;

    subplot(2,3,3);
    yyaxis left; plot(t, V_h, 'b-', 'LineWidth', 1.5); ylabel('Speed (m/s)');
    yyaxis right; plot(t, alt, 'r-', 'LineWidth', 1.2); ylabel('Altitude (m)');
    xline(par.t_burn, 'k--', 'Burnout');
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
    plot(t, rad2deg(X(:,15)),'b-', t, rad2deg(X(:,17)),'r-', ...
         t, rad2deg(X(:,19)),'g-', t, rad2deg(X(:,21)),'m-','LineWidth',1.0);
    yline(15,'k--'); yline(-15,'k--');
    xlabel('Time (s)'); ylabel('deg'); title('Fin Deflections'); grid on;
    legend('Fin1','Fin2','Fin3','Fin4');

    sgtitle(title_str);
    drawnow;

    % Figure 2: GNC Diagnostics
    rho_h = par.rho0*exp(-max(alt,0)/par.h_scale);
    qbar_h = 0.5*rho_h.*V_h.^2;

    figure('Name', sprintf('GNC Diagnostics [%s]', label), ...
           'Position', [80+fig_offset*60, 80+fig_offset*40, 1500, 900], ...
           'Renderer', 'painters');

    subplot(2,3,1);
    plot(t, nz_cmd_h, 'b-',  'LineWidth', 1.3); hold on;
    plot(t, nz_fb_h,  'b:',  'LineWidth', 1.3);
    plot(t, ny_cmd_h, 'r-',  'LineWidth', 1.3);
    plot(t, ny_fb_h,  'r:',  'LineWidth', 1.3);
    yline(0, 'k-', 'LineWidth', 0.5);
    xline(par.t_guide_on, 'g--', 'Guid ON');
    if i_min > 1 && i_min < N_t
        xline(t(i_min), 'm--', 'CPA');
    end
    xlabel('Time (s)'); ylabel('g');
    title('Accel: cmd vs fb (pitch=blue, yaw=red)'); grid on;
    legend('nz_{cmd}','nz_{fb}','ny_{cmd}','ny_{fb}','Location','best');

    subplot(2,3,2);
    plot(t, rad2deg(alpha_h),'b-', t, rad2deg(beta_h),'r-','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('deg');
    title('\alpha / \beta'); grid on; legend('\alpha','\beta');

    subplot(2,3,3);
    plot(t, qbar_h, 'b-','LineWidth',1.5); hold on;
    yline(par.qbar_min_ctrl, 'r--', 'Ctrl ON');
    xlabel('Time (s)'); ylabel('Pa'); title('Dynamic Pressure'); grid on;

    subplot(2,3,4);
    plot(t, v_close, 'b-', 'LineWidth', 1.5); hold on;
    yline(0, 'k--'); xline(par.t_burn, 'r--', 'Burnout');
    if i_min > 1 && i_min < N_t
        plot(t(i_min), v_close(i_min), 'rv', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    end
    xlabel('Time (s)'); ylabel('m/s');
    title('Closing Velocity (+ = away)'); grid on;

    subplot(2,3,5);
    plot(t, X(:,26),'b-', t, X(:,27),'r-','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('rad');
    title('Accel Integrator States'); grid on;
    legend('int_{nz}','int_{ny}');

    subplot(2,3,6);
    de_eff = (X(:,15) - X(:,19))/2;
    dr_eff = (X(:,17) - X(:,21))/2;
    da_eff = mean(X(:,[15,17,19,21]), 2);
    plot(t, rad2deg(de_eff),'b-', t, rad2deg(dr_eff),'r-', ...
         t, rad2deg(da_eff),'g-','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('deg');
    title('\delta_e / \delta_r / \delta_a (effective)'); grid on;
    legend('\delta_e','\delta_r','\delta_a');

    sgtitle(sprintf('GNC Diagnostics — %s', label));
    drawnow;
end

function check_pass(name, passed)
    if passed, fprintf('  [PASS] %s\n', name);
    else,      fprintf('  [FAIL] %s\n', name);
    end
end

function status = timeout_check(~, ~, flag, timer_handle, timeout_s)
    status = 0;
    if isempty(flag)
        if toc(timer_handle) > timeout_s
            status = 1;
            fprintf('  [TIMEOUT] %.0f s wall-clock exceeded\n', timeout_s);
        end
    end
end
