% Q scale sweep for Fixed-Q baseline.

clear; clc; close all;
addpath('..');

N_TRIALS = 10;
BASE_SEED = 42;
TRUTH_CASE = 'nominal';
NOISE_CASE = 'nominal';
Q_ACC_SCALE = 1000;
Q_GYRO_SCALE_LIST = [1 10 20 30 40 50 60 70 80 90 100];

base_cfg = config();
base_cfg.GPS_POS_R_SCALE = 1.0;
base_cfg.GPS_VEL_R_SCALE = 1.0;
base_cfg.BARO_R_SCALE = 1.0;

truth = trajectory_base(TRUTH_CASE);
t = truth.t;
p_true = truth.p;
v_true = truth.v;
q_true = truth.q;
dt = truth.dt;
N = length(t);

fprintf('\n=== Q Scale Sweep ===\n');
fprintf('Trials: %d, Truth: %s, Noise: %s\n', N_TRIALS, TRUTH_CASE, NOISE_CASE);
fprintf('%10s %10s %10s %10s %10s %10s\n', ...
    'QaccScale', 'QgyroScale', 'NIS_GPS_F', 'NIS_BARO_F', 'NEES_P_F', 'NEES_V_F');

summary = zeros(length(Q_GYRO_SCALE_LIST), 6);
m_ref_unit = base_cfg.m_ref_ned / norm(base_cfg.m_ref_ned);

for sidx = 1:length(Q_GYRO_SCALE_LIST)
    cfg = base_cfg;
    cfg.Q_ACC_BASE_SCALE = Q_ACC_SCALE;
    cfg.Q_GYRO_BASE_SCALE = Q_GYRO_SCALE_LIST(sidx);

    gps_nis_f = [];
    baro_nis_f = [];
    nees_p_f = [];
    nees_v_f = [];

    for trial = 1:N_TRIALS
        sim = trajectory_noise(truth, BASE_SEED + trial, NOISE_CASE);
        rng(BASE_SEED + trial + 100000);

        a_meas = sim.a_meas;
        w_meas = sim.w_meas;
        z_gps = sim.z_gps;
        z_baro = sim.z_baro;
        gps_idx = sim.gps_idx;
        baro_idx = sim.baro_idx;
        true_ba = sim.true_ba;
        true_bg = sim.true_bg;
        std_acc = sim.std_acc;
        std_gyro = sim.std_gyro;
        GPS_hAcc = sim.GPS_hAcc;
        GPS_vAcc = sim.GPS_vAcc;

        R_nb_true0 = ESEKF.quat2dcm(q_true(:,1));
        f_static = R_nb_true0' * (-cfg.g_ned);
        m_static = R_nb_true0' * m_ref_unit;
        q0 = ESEKF.run_triad(f_static + true_ba, m_static);

        p0 = p_true(:,1) + [0.5; 0.5; 1.0] .* randn(3,1);
        v0 = v_true(:,1) + [0.1; 0.1; 0.2] .* randn(3,1);

        ekf_base = ESEKF(p0, v0, q0, cfg);
        get_imu = @() deal(f_static + true_ba + std_acc*randn(3,1), ...
                           true_bg + std_gyro*randn(3,1));
        get_mag = @() m_static + sqrt(cfg.var_mag)*randn(3,1);
        ekf_base.run_zupt_alignment(get_imu, dt, 1e-4, 30.0, get_mag);

        ekf_f = ESEKF(p0, v0, q0, cfg);
        ekf_f.nom = ekf_base.nom;
        ekf_f.par.P = ekf_base.par.P;

        gps_ptr = 1;
        baro_ptr = 1;

        for k = 1:N
            ekf_f.predict(a_meas(:,k), w_meas(:,k), dt);

            if gps_ptr <= length(gps_idx) && k == gps_idx(gps_ptr)
                if sim.gps_valid(gps_ptr)
                    [Hf, yf, Rf] = gps_innovation_terms_sweep(ekf_f, z_gps(:,gps_ptr), GPS_hAcc, GPS_vAcc);
                    gps_nis_f(end+1) = normalized_nis_sweep(ekf_f, Hf, yf, Rf); %#ok<SAGROW>
                    ekf_f.update_gps(z_gps(:,gps_ptr), GPS_hAcc, GPS_vAcc);
                end
                gps_ptr = gps_ptr + 1;
            end

            if baro_ptr <= length(baro_idx) && k == baro_idx(baro_ptr)
                [Hf, yf, Rf] = baro_innovation_terms_sweep(ekf_f, z_baro(baro_ptr));
                baro_nis_f(end+1) = normalized_nis_sweep(ekf_f, Hf, yf, Rf); %#ok<SAGROW>
                ekf_f.update_baro(z_baro(baro_ptr));
                baro_ptr = baro_ptr + 1;
            end

            ep_f = ekf_f.nom.p - p_true(:,k);
            ev_f = ekf_f.nom.v - v_true(:,k);
            nees_p_f(end+1) = ep_f' / ekf_f.par.P(1:3,1:3) * ep_f / 3; %#ok<SAGROW>
            nees_v_f(end+1) = ev_f' / ekf_f.par.P(4:6,4:6) * ev_f / 3; %#ok<SAGROW>
        end
    end

    row = [cfg.Q_ACC_BASE_SCALE, cfg.Q_GYRO_BASE_SCALE, ...
           mean(gps_nis_f), mean(baro_nis_f), mean(nees_p_f), mean(nees_v_f)];
    summary(sidx,:) = row;
    fprintf('%10.1f %10.1f %10.3f %10.3f %10.3f %10.3f\n', row);
end

assignin('base', 'q_scale_sweep_summary', summary);

function val = normalized_nis_sweep(ekf, H, y, R)
    S = H * ekf.par.P * H' + R;
    val = y' * (S \ y) / size(H,1);
end

function [H, y, R] = gps_innovation_terms_sweep(ekf, z, hAcc, vAcc)
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
    var_h = ekf.cfg.GPS_POS_R_SCALE * (hAcc * pos_multiplier)^2;
    var_v = ekf.cfg.GPS_POS_R_SCALE * (vAcc * pos_multiplier)^2;
    var_vh = ekf.par.R_gps(4,4);
    var_vv = ekf.par.R_gps(6,6);
    R = diag([var_h, var_h, var_v, var_vh, var_vh, var_vv]);
end

function [H, y, R] = baro_innovation_terms_sweep(ekf, z)
    H = zeros(1,15);
    H(1,3) = -1;
    y = z - (-ekf.nom.p(3));
    R = ekf.par.R_baro;
end
