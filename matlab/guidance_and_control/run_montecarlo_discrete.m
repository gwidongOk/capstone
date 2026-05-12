%% run_montecarlo_discrete.m
%  Guided vs Unguided discrete MC comparison
%  Power-law + Dryden turbulence, paired runs
%
%  Usage: run_montecarlo_discrete

clear; clc; close all;

%% ========== SETTINGS ==========
N_runs   = 200;
rng(42);
Ts_ctrl  = 0.0025;    % 400 Hz
dt_plant = 0.0005;    % 2 kHz RK4

%% ========== DISPERSIONS ==========
disp_config.T_avg_sigma_pct    = 0.10;
disp_config.t_burn_sigma_pct   = 0.05;
disp_config.m0_sigma_pct       = 0.03;
disp_config.aero_sigma_pct     = 0.10;

disp_config.pitch_sigma_deg    = 1.0;
disp_config.yaw_sigma_deg      = 1.0;

disp_config.wind_speed_nominal = 3.0;    % m/s at 10m
disp_config.wind_speed_sigma   = 1.0;
disp_config.wind_dir_nominal   = 0;      % rad, FROM North
disp_config.wind_dir_sigma_deg = 0;

%% ========== BASELINE ==========
par_base = rocket_params();
par_base.target_NED = [100; 100; -500];
los = par_base.target_NED;
par_base.pitch0 = atan2(-los(3), norm(los(1:2)));
par_base.yaw0   = atan2(los(2), los(1));

fprintf('====== DISCRETE MC: %d runs x 2 modes ======\n', N_runs);
fprintf('Target: [%.0f, %.0f, %.0f] m\n', par_base.target_NED);
fprintf('Wind: %.1f +/- %.1f m/s, dir %.0f +/- %.0f deg\n', ...
    disp_config.wind_speed_nominal, disp_config.wind_speed_sigma, ...
    rad2deg(disp_config.wind_dir_nominal), disp_config.wind_dir_sigma_deg);
fprintf('Ctrl: %.0f Hz, Plant: %.0f Hz\n\n', 1/Ts_ctrl, 1/dt_plant);

%% ========== GAINS ==========
par_base = build_gain_table_for_sim(par_base, 'bode');
disc_base = discretize_controller(par_base, Ts_ctrl);
fprintf('Gains ready.\n\n');

%% ========== STORAGE ==========
modes = {'guided','unguided'};
for im = 1:2
    res.(modes{im}).miss    = nan(N_runs,1);
    res.(modes{im}).apogee  = nan(N_runs,1);
    res.(modes{im}).success = false(N_runs,1);
end
res.wind_spd = zeros(N_runs,1);
res.wind_dir = zeros(N_runs,1);
MISS_LIMIT = 10;

%% ========== PRE-GENERATE DISPERSIONS ==========
for i = 1:N_runs
    d(i).T_scale   = 1 + disp_config.T_avg_sigma_pct * randn();
    d(i).tb_scale  = 1 + disp_config.t_burn_sigma_pct * randn();
    d(i).m0_scale  = 1 + disp_config.m0_sigma_pct * randn();
    d(i).aero      = 1 + disp_config.aero_sigma_pct * randn(1,8);
    d(i).dpitch    = deg2rad(disp_config.pitch_sigma_deg * randn());
    d(i).dyaw      = deg2rad(disp_config.yaw_sigma_deg * randn());
    w_spd = max(0, disp_config.wind_speed_nominal + disp_config.wind_speed_sigma*randn());
    w_dir = disp_config.wind_dir_nominal + deg2rad(disp_config.wind_dir_sigma_deg)*randn();
    d(i).wind_speed = w_spd;
    d(i).wind_dir   = 2*pi * rand();
    d(i).gust_seed  = randi(1e6);
end

%% ========== RUN ==========
N_ppc = round(Ts_ctrl / dt_plant);
fprintf('Running %d x 2 = %d simulations...\n', N_runs, N_runs*2);
tw = tic;

for i = 1:N_runs
    di = d(i);

    for im = 1:2
        mode = modes{im};
        par = par_base;

        % Apply dispersions
        par.T_avg  = max(par_base.T_avg * di.T_scale, 50);
        par.t_burn = max(par_base.t_burn * di.tb_scale, 0.5);
        par.mdot   = par.m_prop / par.t_burn;
        par.m0     = max(par_base.m0 * di.m0_scale, par.m_prop+1);

        af = {'CNa','CNd','Cma','Cmd','Cmq','Clp','Clda','CA0'};
        for f = 1:8, par.(af{f}) = par_base.(af{f}) * di.aero(f); end
        par.CYb=-par.CNa; par.CYdr=par.CNd;
        par.Cnb=-par.Cma; par.Cndr=-par.Cmd; par.Cnr=par.Cmq;

        par.pitch0 = par_base.pitch0 + di.dpitch;
        par.yaw0   = par_base.yaw0   + di.dyaw;

        % Wind
        par.wind_model      = 'powerlaw_dryden';
        par.wind_ref_speed  = di.wind_speed;
        par.wind_ref_height = 10;
        par.wind_direction  = di.wind_dir;
        par.wind_alpha      = 1/7;
        par.wind_h_min      = 1;
        par.wind_NED = -di.wind_speed*[cos(di.wind_dir); sin(di.wind_dir); 0];
        par.wind_est = [0;0;0];

        par.dryden_W20   = di.wind_speed;
        par.dryden_h_ref = 500;
        par.dryden_V_ref = 100;
        [par.gust_t, par.gust_NED, gi] = generate_dryden_gust(par, par.t_sim_max, 0.01, di.gust_seed);
        par.gust_interp_N = gi.N;
        par.gust_interp_E = gi.E;
        par.gust_interp_D = gi.D;

        % Mode
        par.disable_all_control = strcmp(mode,'unguided');

        % --- Simulate ---
        try
            q0 = gnc_utils.eul2quat_zyx(0, par.pitch0, par.yaw0);
            R0 = gnc_utils.quat2dcm_bn(q0);
            v0 = R0'*[par.V0;0;0];

            xp = zeros(13,1); xp(4:6)=v0; xp(7:10)=q0(:);
            cs = discrete_sim_utils.init_ctrl_state(disc_base);
            as = zeros(8,1);
            xg = zeros(3,1);

            N_ctrl = ceil(par.t_sim_max / Ts_ctrl);
            min_slant = inf;
            max_alt = 0;
            prev_vD = -1;
            prev_vc = 0;
            mc_phase = 0;  % RAIL

            for k = 1:N_ctrl
                tn = (k-1)*Ts_ctrl;
                pos = xp(1:3); vel = xp(4:6);
                qt = xp(7:10); wb = xp(11:13);
                qn = norm(qt); if qn>1e-10, qt=qt/qn; end

                RB = gnc_utils.quat2dcm_bn(qt);

                % Airspeed for autopilot
                w_now = discrete_sim_utils.get_wind(tn, max(-pos(3),0), par);
                vel_air_NED = vel - w_now;
                vb_air = RB*vel_air_NED;
                V_air = max(norm(vb_air), 1.0);

                alt = max(-pos(3),0);
                rho = par.rho0*exp(-alt/par.h_scale);
                qbar = 0.5*rho*V_air^2;

                % Mass/thrust
                if tn < par.t_burn
                    m = par.m0 - par.mdot*tn; T = par.T_avg;
                else
                    m = par.m0 - par.m_prop; T = 0;
                end

                % Phase transition (one-directional)
                switch mc_phase
                    case 0  % RAIL
                        if tn > 0.01
                            mc_phase = 1;
                        end
                    case 1  % BOOST
                        if qbar > par.qbar_min_ctrl && tn > par.t_guide_on
                            mc_phase = 2;
                        end
                    case 2  % GUIDE
                        rv = pos - par.target_NED;
                        Rt = norm(rv);
                        if Rt > 0.1
                            vc = dot(rv, vel)/Rt;
                            if vc > 0 && prev_vc < 0 && tn > par.t_guide_on + 1.0
                                mc_phase = 3;
                            end
                            prev_vc = vc;
                        end
                    case 3  % CPA
                        vD = vel(3);
                        if prev_vD < 0 && vD >= 0
                            mc_phase = 4;
                        end
                end

                % Phase-dependent GNC
                switch mc_phase
                    case {0, 4}  % RAIL, DESCENT
                        fc = [0;0;0;0];
                    case 1  % BOOST
                        [fc, cs] = discrete_sim_utils.autopilot(tn, 0, 0, ...
                            wb, qt, V_air, qbar, par, disc_base, cs, vel_air_NED);
                    case 2  % GUIDE
                        [nzc,nyc,xdg] = guidance_module(tn, pos, vel, RB, qbar, m, par, xg);
                        xg = xg + Ts_ctrl*xdg;
                        [fc, cs] = discrete_sim_utils.autopilot(tn, nzc, nyc, ...
                            wb, qt, V_air, qbar, par, disc_base, cs, vel_air_NED);
                    case 3  % CPA
                        [fc, cs] = discrete_sim_utils.autopilot(tn, 0, 0, ...
                            wb, qt, V_air, qbar, par, disc_base, cs, vel_air_NED);
                end

                % Plant integration (w_now already computed above)
                for j = 1:N_ppc
                    as = discrete_sim_utils.actuator_step(as, fc, par, dt_plant);
                    fa = as([1,3,5,7]);
                    xp = discrete_sim_utils.rk4_plant_step(tn+(j-1)*dt_plant, xp, fa, T, m, par, dt_plant, w_now);
                    qn = norm(xp(7:10)); if qn>1e-10, xp(7:10)=xp(7:10)/qn; end
                end

                % Track min slant, max alt
                sl = norm(xp(1:3) - par.target_NED);
                if sl < min_slant, min_slant = sl; end
                alt_now = -xp(3);
                if alt_now > max_alt, max_alt = alt_now; end

                % Termination
                if mc_phase >= 3, break; end  % CPA passed -> miss is final
                vD_now = xp(6);
                if tn > par.t_guide_on && vD_now >= 0 && prev_vD < 0, break; end  % Apogee
                prev_vD = vD_now;
                if tn>1 && alt_now<0, break; end  % Ground
            end

            res.(mode).miss(i)    = min_slant;
            res.(mode).apogee(i)  = max_alt;
            res.(mode).success(i) = min_slant < MISS_LIMIT;

        catch ME
            fprintf('  Run %d [%s] FAILED: %s\n', i, mode, ME.message);
        end
    end

    res.wind_spd(i) = di.wind_speed;
    res.wind_dir(i) = rad2deg(di.wind_dir);

    if mod(i,20)==0
        fprintf('  %d/%d (%.1fs)\n', i, N_runs, toc(tw));
    end
end

fprintf('\nDone: %d runs in %.1f s (%.3f s/run)\n\n', N_runs, toc(tw), toc(tw)/(N_runs*2));

%% ========== STATISTICS ==========
fprintf('============ DISCRETE MC COMPARISON ============\n');
for im = 1:2
    mode = modes{im};
    v = ~isnan(res.(mode).miss);
    mv = res.(mode).miss(v);
    fprintf('--- %s ---\n', upper(mode));
    fprintf('  Valid: %d/%d\n', sum(v), N_runs);
    fprintf('  Miss: mean=%.1f, std=%.1f, P95=%.1f, max=%.1f m\n', ...
        mean(mv), std(mv), prctile(mv,95), max(mv));
    fprintf('  Success (<%dm): %.1f%%\n', MISS_LIMIT, 100*mean(res.(mode).success(v)));
    fprintf('  Apogee: %.0f +/- %.0f m\n\n', mean(res.(mode).apogee(v)), std(res.(mode).apogee(v)));
end

vb = ~isnan(res.guided.miss) & ~isnan(res.unguided.miss);
if sum(vb)>0
    df = res.unguided.miss(vb) - res.guided.miss(vb);
    fprintf('--- PAIRED (N=%d) ---\n', sum(vb));
    fprintf('  Guided wins: %d (%.1f%%)\n', sum(df>0), 100*mean(df>0));
    fprintf('  Mean improvement: %.1f m\n', mean(df));
end
fprintf('================================================\n');

%% ========== PLOTS ==========
figure('Position',[50 50 1500 900]);

subplot(2,3,1);
edges = linspace(0, max([res.guided.miss;res.unguided.miss],[],'omitnan')*1.1, 30);
histogram(res.guided.miss, edges, 'FaceColor',[0.2 0.4 0.8],'FaceAlpha',0.7); hold on;
histogram(res.unguided.miss, edges, 'FaceColor',[0.8 0.3 0.2],'FaceAlpha',0.7);
xline(MISS_LIMIT,'k--','LineWidth',1.5);
xlabel('Miss (m)'); ylabel('Count'); title('Miss Distribution');
legend('Guided','Unguided','Limit'); grid on;

subplot(2,3,2);
mg = sort(res.guided.miss(~isnan(res.guided.miss)));
mu = sort(res.unguided.miss(~isnan(res.unguided.miss)));
plot(mg,(1:length(mg))'/length(mg)*100,'b-','LineWidth',2); hold on;
plot(mu,(1:length(mu))'/length(mu)*100,'r-','LineWidth',2);
xline(MISS_LIMIT,'k--'); yline(95,'k:','95%');
xlabel('Miss (m)'); ylabel('CDF %'); title('Miss CDF');
legend('Guided','Unguided','Location','se'); grid on;

subplot(2,3,3);
scatter(res.wind_spd, res.guided.miss, 20,'b','filled','MarkerFaceAlpha',0.5); hold on;
scatter(res.wind_spd, res.unguided.miss, 20,'r','filled','MarkerFaceAlpha',0.5);
yline(MISS_LIMIT,'k--');
xlabel('Wind Speed (m/s)'); ylabel('Miss (m)'); title('Miss vs Wind');
legend('Guided','Unguided'); grid on;

subplot(2,3,4);
scatter(res.wind_dir, res.guided.miss, 20,'b','filled','MarkerFaceAlpha',0.5); hold on;
scatter(res.wind_dir, res.unguided.miss, 20,'r','filled','MarkerFaceAlpha',0.5);
yline(MISS_LIMIT,'k--');
xlabel('Wind Dir (deg)'); ylabel('Miss (m)'); title('Miss vs Wind Dir');
legend('Guided','Unguided'); grid on;

subplot(2,3,5);
mx = max([res.guided.miss;res.unguided.miss],[],'omitnan')*1.1;
scatter(res.unguided.miss, res.guided.miss, 25, res.wind_spd,'filled'); hold on;
plot([0 mx],[0 mx],'k--');
colorbar; xlabel('Unguided Miss (m)'); ylabel('Guided Miss (m)');
title('Paired (color=wind spd)'); grid on; axis equal; xlim([0 mx]); ylim([0 mx]);

subplot(2,3,6);
histogram(res.guided.apogee, 20,'FaceColor',[0.2 0.4 0.8],'FaceAlpha',0.7); hold on;
histogram(res.unguided.apogee, 20,'FaceColor',[0.8 0.3 0.2],'FaceAlpha',0.7);
xlabel('Apogee (m)'); ylabel('Count'); title('Apogee');
legend('Guided','Unguided'); grid on;

sgtitle(sprintf('Discrete MC: %d runs | Guided %.1f%% vs Unguided %.1f%% (<%dm)', ...
    N_runs, 100*mean(res.guided.success(~isnan(res.guided.miss))), ...
    100*mean(res.unguided.success(~isnan(res.unguided.miss))), MISS_LIMIT));