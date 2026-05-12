%% run_montecarlo.m
%  Guided vs Unguided Monte Carlo comparison under wind + parameter uncertainty.

clear; clc; close all;

%% Settings
N_runs = 100;
rng(42);

%% Dispersion definitions
disp_config = struct();

disp_config.T_avg_sigma_pct    = 0.10;
disp_config.t_burn_sigma_pct   = 0.05;
disp_config.m0_sigma_pct       = 0.03;
disp_config.aero_sigma_pct     = 0.10;
disp_config.pitch_sigma_deg    = 1.0;
disp_config.yaw_sigma_deg      = 1.0;

disp_config.wind_speed_nominal = 3.0;
disp_config.wind_speed_sigma   = 1.0;
disp_config.wind_dir_nominal   = 0;
disp_config.wind_dir_sigma_deg = 30;

%% Baseline
par_base = rocket_params();

par_base.target_NED = [100; 100; -500];
los = par_base.target_NED;
par_base.pitch0 = atan2(-los(3), norm(los(1:2)));
par_base.yaw0   = atan2(los(2), los(1));

fprintf('====== MONTE CARLO: %d runs x 2 modes ======\n', N_runs);
fprintf('Target: [%.0f, %.0f, %.0f] m\n', par_base.target_NED);
fprintf('Launch: pitch=%.1f deg, yaw=%.1f deg\n', ...
    rad2deg(par_base.pitch0), rad2deg(par_base.yaw0));
fprintf('Wind nominal: %.1f m/s +/- %.1f, dir %.0f deg +/- %.0f deg\n', ...
    disp_config.wind_speed_nominal, disp_config.wind_speed_sigma, ...
    rad2deg(disp_config.wind_dir_nominal), disp_config.wind_dir_sigma_deg);
fprintf('Wind model: powerlaw + dryden\n\n');

%% Build gain table
par_base.gain_method = 'bode';
fprintf('--- Building gain table ---\n');
par_base = build_gain_table_for_sim(par_base, par_base.gain_method);
fprintf('Done.\n\n');

%% Storage
modes = {'guided', 'unguided'};
N_modes = 2;

for im = 1:N_modes
    res.(modes{im}).miss       = zeros(N_runs, 1);
    res.(modes{im}).apogee     = zeros(N_runs, 1);
    res.(modes{im}).max_alpha  = zeros(N_runs, 1);
    res.(modes{im}).max_beta   = zeros(N_runs, 1);
    res.(modes{im}).max_q_rate = zeros(N_runs, 1);
    res.(modes{im}).max_fin    = zeros(N_runs, 1);
    res.(modes{im}).t_closest  = zeros(N_runs, 1);
    res.(modes{im}).success    = false(N_runs, 1);
end

res.wind_spd   = zeros(N_runs, 1);
res.wind_dir   = zeros(N_runs, 1);
res.T_avg_act  = zeros(N_runs, 1);

MISS_LIMIT = 10;

%% Pre-generate dispersions
dispersions = struct();
for i = 1:N_runs
    dispersions(i).T_scale     = 1 + disp_config.T_avg_sigma_pct * randn();
    dispersions(i).tb_scale    = 1 + disp_config.t_burn_sigma_pct * randn();
    dispersions(i).m0_scale    = 1 + disp_config.m0_sigma_pct * randn();
    dispersions(i).aero_scales = 1 + disp_config.aero_sigma_pct * randn(1, 8);
    dispersions(i).dpitch      = deg2rad(disp_config.pitch_sigma_deg * randn());
    dispersions(i).dyaw        = deg2rad(disp_config.yaw_sigma_deg * randn());

    w_spd = disp_config.wind_speed_nominal + disp_config.wind_speed_sigma * randn();
    w_spd = max(w_spd, 0);
    w_dir = disp_config.wind_dir_nominal + deg2rad(disp_config.wind_dir_sigma_deg) * randn();
    dispersions(i).wind_speed = w_spd;
    dispersions(i).wind_dir   = w_dir;

    dispersions(i).gust_seed  = randi(1e6);
end

%% Run
fprintf('Running %d x %d = %d simulations...\n', N_runs, N_modes, N_runs*N_modes);
t_wall_start = tic;

for i = 1:N_runs
    d = dispersions(i);

    for im = 1:N_modes
        mode = modes{im};

        par = par_base;

        % Apply dispersions
        par.T_avg  = max(par_base.T_avg * d.T_scale, 50);
        par.t_burn = max(par_base.t_burn * d.tb_scale, 0.5);
        par.mdot   = par.m_prop / par.t_burn;

        par.m0 = max(par_base.m0 * d.m0_scale, par.m_prop + 1.0);

        aero_fields = {'CNa','CNd','Cma','Cmd','Cmq','Clp','Clda','CA0'};
        for f = 1:length(aero_fields)
            par.(aero_fields{f}) = par_base.(aero_fields{f}) * d.aero_scales(f);
        end
        par.CYb  = -par.CNa;  par.CYdr =  par.CNd;
        par.Cnb  = -par.Cma;  par.Cndr = -par.Cmd;
        par.Cnr  =  par.Cmq;

        par.pitch0 = par_base.pitch0 + d.dpitch;
        par.yaw0   = par_base.yaw0   + d.dyaw;

        par.wind_model      = 'powerlaw_dryden';
        par.wind_ref_speed  = d.wind_speed;
        par.wind_ref_height = 10.0;
        par.wind_direction  = d.wind_dir;
        par.wind_alpha      = 1/7;
        par.wind_h_min      = 1.0;

        par.wind_NED = -d.wind_speed * [cos(d.wind_dir); sin(d.wind_dir); 0];
        par.wind_est = [0; 0; 0];

        par.dryden_W20    = d.wind_speed;
        par.dryden_h_ref  = 500;
        par.dryden_V_ref  = 100;
        [par.gust_t, par.gust_NED, gust_interps] = ...
            generate_dryden_gust(par, par.t_sim_max, 0.01, d.gust_seed);
        par.gust_interp_N = gust_interps.N;
        par.gust_interp_E = gust_interps.E;
        par.gust_interp_D = gust_interps.D;

        par.suppress_debug = true;
        if strcmp(mode, 'unguided')
            par.disable_all_control = true;
        else
            par.disable_all_control = false;
        end

        run_timeout = 2.0;
        try
            q0 = gnc_utils.eul2quat_zyx(0, par.pitch0, par.yaw0);
            R0 = gnc_utils.quat2dcm_bn(q0);
            v0 = R0' * [par.V0; 0; 0];

            x0 = zeros(27, 1);
            x0(4:6)  = v0;
            x0(7:10) = q0(:);

            t_run_start = tic;
            timeout_fcn = @(t,x,flag) deal(ternary(toc(t_run_start) > run_timeout, 1, 0));

            opts = odeset('RelTol',1e-6, 'AbsTol',1e-8, 'MaxStep',0.01, ...
                'Events', @(t,x) gnc_utils.flight_events(t, x, par));

            [t, X, ~, ~, ie] = ode45(@(t,x) eom_6dof_wind(t, x, par), ...
                [0 par.t_sim_max], x0, opts);

            if toc(t_run_start) > run_timeout
                fprintf('  Run %d [%s] TIMEOUT (%.1fs)\n', i, mode, toc(t_run_start));
                res.(mode).miss(i) = NaN;
                res.(mode).success(i) = false;
                continue;
            end

            pos = X(:,1:3);  vel = X(:,4:6);
            alt = -pos(:,3);
            slant = vecnorm(pos - par_base.target_NED', 2, 2);
            [min_s, i_min] = min(slant);
            [~, i_apo] = max(alt);

            t_end_mission = min(t(i_min), t(i_apo));
            i_mission = t <= t_end_mission;
            alpha_m = zeros(sum(i_mission),1);
            beta_m  = zeros(sum(i_mission),1);
            idx = find(i_mission);
            for k = 1:length(idx)
                qq = X(idx(k),7:10)'; qq = qq/norm(qq);
                RR = gnc_utils.quat2dcm_bn(qq);
                vb = RR * vel(idx(k),:)';
                alpha_m(k) = abs(rad2deg(atan2(vb(3), max(vb(1),0.1))));
                beta_m(k)  = abs(rad2deg(asin(max(-1,min(1,vb(2)/max(norm(vb),1))))));
            end

            res.(mode).miss(i)       = min_s;
            res.(mode).apogee(i)     = max(alt);
            res.(mode).max_alpha(i)  = max(alpha_m);
            res.(mode).max_beta(i)   = max(beta_m);
            res.(mode).max_q_rate(i) = max(abs(rad2deg(X(i_mission,12))));
            res.(mode).max_fin(i)    = max(abs(rad2deg(X(:,15:2:21))), [], 'all');
            res.(mode).t_closest(i)  = t(i_min);
            res.(mode).success(i)    = min_s < MISS_LIMIT;

        catch ME
            fprintf('  Run %d [%s] FAILED: %s\n', i, mode, ME.message);
            res.(mode).miss(i) = NaN;
            res.(mode).success(i) = false;
        end
    end

    res.wind_spd(i) = d.wind_speed;
    res.wind_dir(i) = rad2deg(d.wind_dir);
    res.T_avg_act(i) = par.T_avg;

    if mod(i, 10) == 0
        fprintf('  %d/%d complete (%.1f s)\n', i, N_runs, toc(t_wall_start));
    end
end

wall_total = toc(t_wall_start);
fprintf('\nDone: %d runs in %.1f s (%.2f s/run avg)\n\n', ...
    N_runs, wall_total, wall_total/(N_runs*2));

%% Statistics
fprintf('============ MONTE CARLO COMPARISON ============\n');
fprintf('Runs: %d, Wind: powerlaw + dryden\n', N_runs);
fprintf('Wind: %.1f +/- %.1f m/s, dir %.0f +/- %.0f deg\n\n', ...
    disp_config.wind_speed_nominal, disp_config.wind_speed_sigma, ...
    rad2deg(disp_config.wind_dir_nominal), disp_config.wind_dir_sigma_deg);

for im = 1:N_modes
    mode = modes{im};
    valid = ~isnan(res.(mode).miss);
    miss_v = res.(mode).miss(valid);

    fprintf('--- %s ---\n', upper(mode));
    fprintf('  Valid: %d/%d\n', sum(valid), N_runs);
    fprintf('  Miss:  mean=%.1f, med=%.1f, std=%.1f, P95=%.1f, max=%.1f m\n', ...
        mean(miss_v), median(miss_v), std(miss_v), prctile(miss_v,95), max(miss_v));
    fprintf('  Success (<%dm): %.1f%% (%d/%d)\n', ...
        MISS_LIMIT, 100*sum(res.(mode).success(valid))/sum(valid), ...
        sum(res.(mode).success(valid)), sum(valid));
    fprintf('  Apogee: mean=%.0f +/- %.0f m\n', ...
        mean(res.(mode).apogee(valid)), std(res.(mode).apogee(valid)));
    fprintf('  Max |alpha|: mean=%.1f, max=%.1f deg\n', ...
        mean(res.(mode).max_alpha(valid)), max(res.(mode).max_alpha(valid)));
    fprintf('\n');
end

% Paired comparison
valid_both = ~isnan(res.guided.miss) & ~isnan(res.unguided.miss);
if sum(valid_both) > 0
    diff = res.unguided.miss(valid_both) - res.guided.miss(valid_both);
    guided_wins = sum(diff > 0);
    fprintf('--- PAIRED COMPARISON (N=%d) ---\n', sum(valid_both));
    fprintf('  Guided wins: %d/%d (%.1f%%)\n', guided_wins, sum(valid_both), ...
        100*guided_wins/sum(valid_both));
    fprintf('  Mean improvement: %.1f m (guided closer)\n', mean(diff));
    fprintf('  Median improvement: %.1f m\n', median(diff));
    fprintf('  Guided worse in: %d runs\n', sum(diff < 0));
end
fprintf('================================================\n');

%% Plots
figure('Name','MC Comparison','Position',[50 50 1500 900]);

subplot(2,3,1);
edges = linspace(0, max([res.guided.miss; res.unguided.miss], [], 'omitnan')*1.1, 30);
histogram(res.guided.miss, edges, 'FaceColor',[0.2 0.4 0.8], 'FaceAlpha',0.7); hold on;
histogram(res.unguided.miss, edges, 'FaceColor',[0.8 0.3 0.2], 'FaceAlpha',0.7);
xline(MISS_LIMIT, 'k--', 'LineWidth',1.5);
xlabel('Miss Distance (m)'); ylabel('Count');
title('Miss Distribution');
legend('Guided','Unguided','Limit');
grid on;

subplot(2,3,2);
miss_g = sort(res.guided.miss(~isnan(res.guided.miss)));
miss_u = sort(res.unguided.miss(~isnan(res.unguided.miss)));
plot(miss_g, (1:length(miss_g))'/length(miss_g)*100, 'b-', 'LineWidth',2); hold on;
plot(miss_u, (1:length(miss_u))'/length(miss_u)*100, 'r-', 'LineWidth',2);
xline(MISS_LIMIT, 'k--');
yline(95, 'k:', '95%');
xlabel('Miss Distance (m)'); ylabel('Cumulative %');
title('Miss CDF');
legend('Guided','Unguided','Location','southeast');
grid on;

subplot(2,3,3);
scatter(res.wind_spd, res.guided.miss, 20, 'b', 'filled', 'MarkerFaceAlpha',0.6); hold on;
scatter(res.wind_spd, res.unguided.miss, 20, 'r', 'filled', 'MarkerFaceAlpha',0.6);
yline(MISS_LIMIT, 'k--');
xlabel('Wind Speed (m/s)'); ylabel('Miss (m)');
title('Miss vs Wind Speed');
legend('Guided','Unguided');
grid on;

subplot(2,3,4);
scatter(res.wind_dir, res.guided.miss, 20, 'b', 'filled', 'MarkerFaceAlpha',0.6); hold on;
scatter(res.wind_dir, res.unguided.miss, 20, 'r', 'filled', 'MarkerFaceAlpha',0.6);
yline(MISS_LIMIT, 'k--');
xlabel('Wind Direction (deg)'); ylabel('Miss (m)');
title('Miss vs Wind Direction');
legend('Guided','Unguided');
grid on;

subplot(2,3,5);
max_ax = max([res.guided.miss; res.unguided.miss], [], 'omitnan') * 1.1;
scatter(res.unguided.miss, res.guided.miss, 25, res.wind_spd, 'filled'); hold on;
plot([0 max_ax], [0 max_ax], 'k--', 'LineWidth',1);
colorbar; colormap(parula);
xlabel('Unguided Miss (m)'); ylabel('Guided Miss (m)');
title('Paired: Guided vs Unguided (color=wind spd)');
grid on; axis equal;F
xlim([0 max_ax]); ylim([0 max_ax]);

subplot(2,3,6);
histogram(res.guided.apogee, 20, 'FaceColor',[0.2 0.4 0.8], 'FaceAlpha',0.7); hold on;
histogram(res.unguided.apogee, 20, 'FaceColor',[0.8 0.3 0.2], 'FaceAlpha',0.7);
xlabel('Apogee (m)'); ylabel('Count');
title('Apogee Distribution');
legend('Guided','Unguided');
grid on;

sgtitle(sprintf('Monte Carlo: %d runs | Guided %.1f%% vs Unguided %.1f%% success (<%dm)', ...
    N_runs, ...
    100*sum(res.guided.success)/sum(~isnan(res.guided.miss)), ...
    100*sum(res.unguided.success)/sum(~isnan(res.unguided.miss)), ...
    MISS_LIMIT));
