% Fixed-Q vs jerk-adaptive-Q ES-EKF Monte Carlo comparison.

clear; clc; close all;
addpath('..');

N_TRIALS  = 200;
BASE_SEED = 42;
TRUTH_CASE = 'strong_canard';
NOISE_CASE = 'nominal';

S = config();
truth = trajectory_base(TRUTH_CASE);

t = truth.t;
p_true = truth.p;
v_true = truth.v;
q_true = truth.q;
dt = truth.dt;
N = length(t);

m_ref_unit = S.m_ref_ned / norm(S.m_ref_ned);

state_err_f = zeros(N_TRIALS, N, 15);
state_err_j = zeros(N_TRIALS, N, 15);
state_sig_f = zeros(N_TRIALS, N, 15);
state_sig_j = zeros(N_TRIALS, N, 15);

nis_gps_sum  = zeros(1,2); nis_gps_count  = 0;
nis_baro_sum = zeros(1,2); nis_baro_count = 0;
nees_p_sum   = zeros(1,2); nees_p_count   = 0;
nees_v_sum   = zeros(1,2); nees_v_count   = 0;

fprintf('=== ES-EKF Fixed vs Jerk Monte Carlo ===\n');
fprintf('Trials: %d, Truth: %s, Noise: %s\n', N_TRIALS, TRUTH_CASE, NOISE_CASE);

for trial = 1:N_TRIALS
    sim = trajectory_noise(truth, BASE_SEED + trial, NOISE_CASE);
    rng(BASE_SEED + trial + 100000);

    a_meas = sim.a_meas;
    w_meas = sim.w_meas;
    z_gps = sim.z_gps;
    z_baro = sim.z_baro;
    gps_idx = sim.gps_idx;
    baro_idx = sim.baro_idx;
    GPS_hAcc = sim.GPS_hAcc;
    GPS_vAcc = sim.GPS_vAcc;
    true_ba = sim.true_ba;
    true_bg = sim.true_bg;
    std_acc = sim.std_acc;
    std_gyro = sim.std_gyro;

    R_nb_true0 = ESEKF.quat2dcm(q_true(:,1));
    f_static = R_nb_true0' * (-S.g_ned);
    m_static = R_nb_true0' * m_ref_unit;
    q0 = ESEKF.run_triad(f_static + true_ba, m_static);

    p0 = p_true(:,1) + [0.5; 0.5; 1.0] .* randn(3,1);
    v0 = v_true(:,1) + [0.1; 0.1; 0.2] .* randn(3,1);

    ekf_base = ESEKF(p0, v0, q0);
    get_imu = @() deal(f_static + true_ba + std_acc*randn(3,1), ...
                       true_bg + std_gyro*randn(3,1));
    get_mag = @() m_static + sqrt(S.var_mag)*randn(3,1);
    ekf_base.run_zupt_alignment(get_imu, dt, 1e-4, 30.0, get_mag);

    ekf_f = ESEKF(p0, v0, q0);
    ekf_j = ESEKF(p0, v0, q0);
    ekf_f.nom = ekf_base.nom; ekf_f.par.P = ekf_base.par.P;
    ekf_j.nom = ekf_base.nom; ekf_j.par.P = ekf_base.par.P;

    gps_ptr = 1;
    baro_ptr = 1;

    for k = 1:N
        ekf_f.predict(a_meas(:,k), w_meas(:,k), dt);
        ekf_j.predict_adaptive_jerk(a_meas(:,k), w_meas(:,k), dt);

        if gps_ptr <= length(gps_idx) && k == gps_idx(gps_ptr)
            if sim.gps_valid(gps_ptr)
                gps_filters = {ekf_f, ekf_j};
                for n = 1:2
                    [H_gps, y_gps, R_gps] = gps_innovation_terms(gps_filters{n}, z_gps(:,gps_ptr), GPS_hAcc, GPS_vAcc);
                    S_gps = H_gps * gps_filters{n}.par.P * H_gps' + R_gps;
                    nis_gps_sum(n) = nis_gps_sum(n) + y_gps' * (S_gps \ y_gps) / 6;
                end
                nis_gps_count = nis_gps_count + 1;

                ekf_f.update_gps(z_gps(:,gps_ptr), GPS_hAcc, GPS_vAcc);
                ekf_j.update_gps(z_gps(:,gps_ptr), GPS_hAcc, GPS_vAcc);
            end
            gps_ptr = gps_ptr + 1;
        end

        if baro_ptr <= length(baro_idx) && k == baro_idx(baro_ptr)
            baro_filters = {ekf_f, ekf_j};
            for n = 1:2
                [H_baro, y_baro, R_baro] = baro_innovation_terms(baro_filters{n}, z_baro(baro_ptr));
                S_baro = H_baro * baro_filters{n}.par.P * H_baro' + R_baro;
                nis_baro_sum(n) = nis_baro_sum(n) + y_baro' * (S_baro \ y_baro);
            end
            nis_baro_count = nis_baro_count + 1;

            ekf_f.update_baro(z_baro(baro_ptr));
            ekf_j.update_baro(z_baro(baro_ptr));
            baro_ptr = baro_ptr + 1;
        end

        nees_filters = {ekf_f, ekf_j};
        for n = 1:2
            ep = p_true(:,k) - nees_filters{n}.nom.p;
            ev = v_true(:,k) - nees_filters{n}.nom.v;
            Pp = nees_filters{n}.par.P(1:3, 1:3);
            Pv = nees_filters{n}.par.P(4:6, 4:6);
            nees_p_sum(n) = nees_p_sum(n) + ep' * (Pp \ ep) / 3;
            nees_v_sum(n) = nees_v_sum(n) + ev' * (Pv \ ev) / 3;
        end
        nees_p_count = nees_p_count + 1;
        nees_v_count = nees_v_count + 1;

        state_err_f(trial,k,:) = state_error_vector(ekf_f, p_true(:,k), v_true(:,k), q_true(:,k), true_ba, true_bg);
        state_err_j(trial,k,:) = state_error_vector(ekf_j, p_true(:,k), v_true(:,k), q_true(:,k), true_ba, true_bg);
        state_sig_f(trial,k,:) = 3.0 * sqrt(max(diag(ekf_f.par.P), 0.0));
        state_sig_j(trial,k,:) = 3.0 * sqrt(max(diag(ekf_j.par.P), 0.0));
    end
end

filter_names = {'Fixed', 'Jerk'};
mean_nis_gps  = nis_gps_sum  / nis_gps_count;
mean_nis_baro = nis_baro_sum / nis_baro_count;
mean_nees_p   = nees_p_sum   / nees_p_count;
mean_nees_v   = nees_v_sum   / nees_v_count;

fprintf('\n=== Normalized Consistency Summary (target ~= 1) ===\n');
fprintf('%-12s %10s %10s %10s %10s\n', 'Filter', 'NIS_GPS', 'NIS_BARO', 'NEES_P', 'NEES_V');
for n = 1:2
    fprintf('%-12s %10.3f %10.3f %10.3f %10.3f\n', filter_names{n}, ...
        mean_nis_gps(n), mean_nis_baro(n), mean_nees_p(n), mean_nees_v(n));
end
fprintf('\n');

state_labels = {'p_N','p_E','p_D', ...
                'v_N','v_E','v_D', ...
                'att_x','att_y','att_z', ...
                'b_{a,x}','b_{a,y}','b_{a,z}', ...
                'b_{g,x}','b_{g,y}','b_{g,z}'};
state_units = {'m','m','m', ...
               'm/s','m/s','m/s', ...
               'deg','deg','deg', ...
               'm/s^2','m/s^2','m/s^2', ...
               'deg/s','deg/s','deg/s'};
state_scale = [ones(1,6), (180/pi)*ones(1,3), ones(1,3), (180/pi)*ones(1,3)];

plot_error_state_components(t, state_err_f, state_sig_f, state_err_j, state_sig_j, ...
    state_labels, state_units, state_scale);

function [H, y, R] = gps_innovation_terms(ekf, z, hAcc, vAcc)
    H = zeros(6,15);
    H(1:3,1:3) = eye(3);
    H(4:6,4:6) = eye(3);
    y = z - [ekf.nom.p; ekf.nom.v];

    G4_LIMIT = 4.0 * 9.80665;
    high_g_penalty = 1.0;
    if ekf.last_accel_mag > G4_LIMIT
        excess = ekf.last_accel_mag - G4_LIMIT;
        high_g_penalty = 1.0 + (excess * excess * 10.0);
        high_g_penalty = min(high_g_penalty, 30.0);
    end

    pos_multiplier = ekf.cfg.GPS_ACC_INFLATION * high_g_penalty;
    var_h  = ekf.cfg.GPS_POS_R_SCALE * (hAcc * pos_multiplier)^2;
    var_v  = ekf.cfg.GPS_POS_R_SCALE * (vAcc * pos_multiplier)^2;
    var_vh = ekf.par.R_gps(4,4);
    var_vv = ekf.par.R_gps(6,6);
    R = diag([var_h, var_h, var_v, var_vh, var_vh, var_vv]);
end

function [H, y, R] = baro_innovation_terms(ekf, z)
    H = zeros(1,15);
    H(1,3) = -1;
    y = z - (-ekf.nom.p(3));
    R = ekf.par.R_baro;
end

function dx = state_error_vector(ekf, p_ref, v_ref, q_ref, ba_ref, bg_ref)
    dx = zeros(15,1);
    dx(1:3) = ekf.nom.p - p_ref;
    dx(4:6) = ekf.nom.v - v_ref;

    q_err = ESEKF.quat_mult(ESEKF.quat_conj(ekf.nom.q), q_ref);
    if q_err(1) < 0
        q_err = -q_err;
    end
    dx(7:9) = 2.0 * q_err(2:4);

    dx(10:12) = ekf.nom.b_a - ba_ref;
    dx(13:15) = ekf.nom.b_g - bg_ref;
end

function plot_error_state_components(t, err_f, sig_f, err_j, sig_j, labels, units, scale)
    coverage_f = zeros(1,15);
    coverage_j = zeros(1,15);

    fprintf('=== Error-State 3-Sigma Coverage ===\n');
    fprintf('%-10s %12s %12s\n', 'State', 'Fixed[%]', 'Jerk[%]');
    for idx = 1:15
        coverage_f(idx) = 100.0 * mean(abs(err_f(:,:,idx)) <= sig_f(:,:,idx), 'all');
        coverage_j(idx) = 100.0 * mean(abs(err_j(:,:,idx)) <= sig_j(:,:,idx), 'all');
        fprintf('%-10s %12.2f %12.2f\n', labels{idx}, coverage_f(idx), coverage_j(idx));
    end
    fprintf('\n');

    figure('Name', 'Error-State Component Monte Carlo Overlay', 'Position', [80 80 1700 950]);
    tiledlayout(5, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    for idx = 1:15
        nexttile; hold on; grid on;

        ef_trials = scale(idx) * squeeze(err_f(:,:,idx));
        ej_trials = scale(idx) * squeeze(err_j(:,:,idx));
        ef = scale(idx) * mean(err_f(:,:,idx), 1);
        ej = scale(idx) * mean(err_j(:,:,idx), 1);
        sf = scale(idx) * mean(sig_f(:,:,idx), 1);
        sj = scale(idx) * mean(sig_j(:,:,idx), 1);

        plot(t, ef_trials', 'Color', [0.70 0.82 1.00], 'LineWidth', 0.25, 'HandleVisibility', 'off');
        plot(t, ej_trials', 'Color', [1.00 0.78 0.78], 'LineWidth', 0.25, 'HandleVisibility', 'off');

        plot(t, ef, 'b', 'LineWidth', 1.1, 'DisplayName', 'Fixed Mean');
        plot(t, ef + sf, 'b:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        plot(t, ef - sf, 'b:', 'LineWidth', 1.0, 'DisplayName', 'Fixed Mean \pm3\sigma');

        plot(t, ej, 'k', 'LineWidth', 1.2, 'DisplayName', 'Jerk Mean');
        plot(t, ej + sj, 'r--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        plot(t, ej - sj, 'r--', 'LineWidth', 1.0, 'DisplayName', 'Jerk Mean \pm3\sigma');

        title(labels{idx}, 'Interpreter', 'tex');
        xlabel('Time [s]');
        ylabel(units{idx});

        y_abs = max(abs([ef_trials(:); ej_trials(:); ef(:) + sf(:); ef(:) - sf(:); ej(:) + sj(:); ej(:) - sj(:)]));
        if y_abs > 0
            ylim(1.1 * [-y_abs, y_abs]);
        end

        if idx == 1
            legend('Location', 'best', 'FontSize', 7);
        end
    end

    sgtitle('ES-EKF Error-State Components: Monte Carlo Overlay and \pm3\sigma', 'FontSize', 14);
end
