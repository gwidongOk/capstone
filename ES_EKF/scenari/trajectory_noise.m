function sim = trajectory_noise(truth, trial_seed, case_name)
%TRAJECTORY_NOISE  Sensor input generator for one Monte Carlo trial.

    if nargin < 2 || isempty(trial_seed)
        trial_seed = 42;
    end
    if nargin < 3 || isempty(case_name)
        case_name = 'nominal';
    end

    S = config;
    rng(trial_seed);

    t = truth.t;
    N = length(t);
    dt = truth.dt;

    imu_noise_scale = 1.0;
    gps_pos_actual_scale = 3.0;
    gps_pos_report_scale = 1.0;
    gps_vel_actual_scale = 1.0;
    gps_vel_report_scale = 1.0;
    baro_noise_scale = 1.0;
    acc_scale = eye(3);
    gyro_scale = eye(3);
    gps_dropout_windows = zeros(0,2);

    switch lower(case_name)
        case 'nominal'
        case 'high_imu_noise'
            imu_noise_scale = 3.0;
        case 'gps_degraded'
            gps_pos_actual_scale = gps_pos_actual_scale * 3.0;
            gps_pos_report_scale = gps_pos_report_scale * 3.0;
            gps_vel_actual_scale = gps_vel_actual_scale * 3.0;
            gps_vel_report_scale = gps_vel_report_scale * 3.0;
        case 'canard_gps_dropout'
            gps_dropout_windows = [5.8, 6.8];
        case 'imu_scale_factor'
            acc_scale  = diag([1.015, 0.985, 1.010]);
            gyro_scale = diag([1.003, 0.997, 1.002]);
        otherwise
            error('trajectory_noise:UnknownCase', 'Unknown noise case: %s', case_name);
    end

    true_ba = [ 0.05; -0.02;  0.01 ];
    true_bg = [ 0.001; -0.002; 0.003 ];

    std_acc  = imu_noise_scale * sqrt(S.var_acc);
    std_gyro = imu_noise_scale * sqrt(S.var_gyro);

    a_meas = acc_scale  * truth.f_body + true_ba + std_acc  * randn(3, N);
    w_meas = gyro_scale * truth.w_body + true_bg + std_gyro * randn(3, N);

    gps_idx  = 1 : round(S.dt_gps  / dt) : N;
    baro_idx = 1 : round(S.dt_baro / dt) : N;

    sig_p_gps = gps_pos_actual_scale * sqrt([S.var_gps_pos_h; S.var_gps_pos_h; S.var_gps_pos_v]);
    sig_v_gps = gps_vel_actual_scale * sqrt([S.var_gps_vel_h; S.var_gps_vel_h; S.var_gps_vel_v]);

    z_gps = [truth.p(:,gps_idx) + sig_p_gps .* randn(3, length(gps_idx));
             truth.v(:,gps_idx) + sig_v_gps .* randn(3, length(gps_idx))];

    gps_valid = true(1, length(gps_idx));
    for n = 1:size(gps_dropout_windows, 1)
        t0 = gps_dropout_windows(n, 1);
        t1 = gps_dropout_windows(n, 2);
        gps_valid = gps_valid & ~(t(gps_idx) >= t0 & t(gps_idx) <= t1);
    end

    z_baro = -truth.p(3, baro_idx) + baro_noise_scale * sqrt(S.var_baro) * randn(1, length(baro_idx));

    sim = truth;
    sim.a_meas = a_meas;
    sim.w_meas = w_meas;
    sim.z_gps = z_gps;
    sim.z_baro = z_baro;
    sim.gps_idx = gps_idx;
    sim.baro_idx = baro_idx;
    sim.gps_valid = gps_valid;
    sim.true_ba = true_ba;
    sim.true_bg = true_bg;
    sim.std_acc = std_acc;
    sim.std_gyro = std_gyro;
    sim.GPS_hAcc = gps_pos_report_scale * sqrt(S.var_gps_pos_h);
    sim.GPS_vAcc = gps_pos_report_scale * sqrt(S.var_gps_pos_v);
    sim.GPS_sAcc_h = gps_vel_report_scale * sqrt(S.var_gps_vel_h);
    sim.GPS_sAcc_v = gps_vel_report_scale * sqrt(S.var_gps_vel_v);
    sim.noise_case = case_name;
end
