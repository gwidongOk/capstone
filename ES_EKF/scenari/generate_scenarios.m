clear; clc; close all;
addpath('..');

motor_table = readtable('motor.csv');
t_motor      = motor_table{:, 1}; % Time(s)
thrust_motor = motor_table{:, 2}; % Thrust(N)
prop_mass_kg = (motor_table{:, 3} + motor_table{:, 4}) / 1000; %prop mass (kg)

% 발사체 제원 설정
rocket.m0         = 2.5;            % 이륙 시 초기 전체 질량 [kg]
rocket.Cd         = 0.55;           % 항력 계수
rocket.A          = 0.00785;        % 단면적 [m²]
rocket.rho        = 1.225;          % 공기 밀도 [kg/m³]
rocket.motor_t    = t_motor;        % 모터 시간 [s]
rocket.motor_T    = thrust_motor;   % 모터 추력 [N]
rocket.motor_mp   = prop_mass_kg;   % 시간에 따른 추진제 질량 [kg]
rocket.m_prop_i   = prop_mass_kg(1);% 초기 추진제 총 질량 [kg]

% 시나리오 정의
%                   name         label            t[s]  pitch[deg]  wind
scenarios(1) = sc('vertical', '수직 발사',         30,    -90,       0.0);
scenarios(2) = sc('tilted',   '5° 경사 발사',      30,    -85,       0.3);
scenarios(3) = sc('windy',    '강풍 외란',          30,    -88.5,     1.5);

% 데이터 생성 루프
S = SensorSpec; % 센서 스펙
for i = 1:length(scenarios)
    data     = build_scenario(scenarios(i), rocket, S);
    filename = sprintf('scenario_%s.mat', scenarios(i).name);
    save(filename, 'data');
    fprintf('[%d/%d] %s → %s  (최대고도 %.0f m)\n', ...
        i, length(scenarios), scenarios(i).label, filename, max(-data.p_true(3,:)));
end

% ═══════════════════════════════════════════════════════════════════════
% build_scenario: CSV 데이터를 반영한 3단계 시나리오 생성 함수
% ═══════════════════════════════════════════════════════════════════════
function data = build_scenario(sc, rocket, S)
    dt = S.dt_imu;
    N  = round(sc.t_total / dt);
    t  = (0:N-1)' * dt;   % 시뮬레이션 시간 벡터 [N×1]

    % ── STEP 1: bodyAcceleration & bodyAngularVelocity 정의 ─────────────
    
    % 1.1 추력 프로파일 (보간법 적용)
    T_k = interp1(rocket.motor_t, rocket.motor_T, t, 'linear', 0);
    
    % 1.2 질량 프로파일
    % 건조 질량(Dry Mass) = 초기 총질량 - 초기 추진제 질량
    m_dry = rocket.m0 - rocket.m_prop_i;
    % 현재 총질량 = 건조 질량 + 현재 추진제 질량 (연소 종료 후엔 마지막 값 유지)
    mp_k = interp1(rocket.motor_t, rocket.motor_mp, t, 'linear', rocket.motor_mp(end));
    m_k  = m_dry + mp_k;

    % 1.3 가속도 계산 (수직 방향 근사로 항력 계산)
    % vel_D > 0: 하강(항력 위로), vel_D < 0: 상승(항력 아래로)
    a_D   = 9.81 - T_k ./ m_k;
    vel_D = [0; cumsum(a_D(1:end-1)) * dt];
    drag_N    = 0.5 * rocket.rho * vel_D.^2 * rocket.Cd * rocket.A;
    f_body_x  = T_k ./ m_k + sign(vel_D) .* drag_N ./ m_k; 
    
    bodyAcceleration = [f_body_x, zeros(N,2)];   % [N×3]

    % 1.4 각속도: 롤 스핀 + 바람 외란(감쇠 진동)
    t_burn_end = rocket.motor_t(end);
    burning    = (t <= t_burn_end);
    roll_burn  = 4.0 * t .* burning;
    roll_coast = (4.0 * t_burn_end) * exp(-(t - t_burn_end)/6) .* ~burning;
    roll       = roll_burn + roll_coast;
    
    tau   = max(t - 0.3, 0) .* (t > 0.3);
    wn    = 2*pi*1.5;
    env   = sc.wind_amp * exp(-0.15*wn*tau) .* (t > 0.3);
    pitch = env .* cos(wn*tau);
    yaw   = env * 0.7 .* sin(wn*tau);
    
    bodyAngularVelocity = [roll, pitch, yaw];     % [N×3]

    % ── STEP 2: kinematicTrajectory → Ground Truth ──────────────────────
    q_init = quaternion([0, deg2rad(sc.pitch_init), 0], 'euler', 'ZYX', 'frame');
    traj   = kinematicTrajectory('SampleRate', round(1/dt), ...
                 'Position', [0 0 0], 'Velocity', [0 0 0], 'Orientation', q_init);
             
    [pos_ned, orient, vel_ned] = traj(bodyAcceleration, bodyAngularVelocity);
    p_true = pos_ned';                  % [3×N]
    v_true = vel_ned';                  % [3×N]
    q_true = compact(orient)';          % [4×N] [w; x; y; z]

    % ── STEP 3: 센서 측정값 생성 ─────────────────────────────────────────
    true_ba = [0.05; -0.02;  0.01  ];
    true_bg = [0.001; -0.002; 0.003];

    % IMU (가속도계, 자이로)
    a_meas = bodyAcceleration' + true_ba + sqrt(S.var_acc/dt)  * randn(3,N);
    w_meas = bodyAngularVelocity' + true_bg + sqrt(S.var_gyro/dt) * randn(3,N);

    % GPS
    gps_idx = 1 : round(S.dt_gps/dt) : N;
    sig_p   = [S.sigma_gps_pos_h; S.sigma_gps_pos_h; S.sigma_gps_pos_v];
    sig_v   = [S.sigma_gps_vel_h; S.sigma_gps_vel_h; S.sigma_gps_vel_v];
    z_gps   = [p_true(:,gps_idx) + sig_p .* randn(3, length(gps_idx));
               v_true(:,gps_idx) + sig_v .* randn(3, length(gps_idx))];

    % Barometer (기압계)
    baro_idx = 1 : round(S.dt_baro/dt) : N;
    z_baro   = -p_true(3, baro_idx) + sqrt(S.var_baro) * randn(1, length(baro_idx));

    % Magnetometer (지자기계)
    m_ref_unit = S.m_ref_ned / norm(S.m_ref_ned);
    mag_idx    = 1 : round(S.dt_mag/dt) : N;
    z_mag      = zeros(3, length(mag_idx));
    for k = 1:length(mag_idx)
        % 간단한 quat2dcm 유틸리티 또는 MATLAB 기본 함수 사용
        R_nb = quat2dcm(q_true(:, mag_idx(k))'); 
        z_mag(:,k) = R_nb' * m_ref_unit + sqrt(S.var_mag) * randn(3,1);
    end

    % ── 구조체 데이터 조립 ──────────────────────────────────────────────
    data.scenario = sc.name;     data.label    = sc.label;
    data.t        = t';          data.N        = N;       data.dt = dt;
    data.p_true   = p_true;      data.v_true   = v_true;  data.q_true = q_true;
    data.ba_true  = repmat(true_ba, 1, N);
    data.bg_true  = repmat(true_bg, 1, N);
    data.a_meas   = a_meas;      data.w_meas   = w_meas;
    data.z_gps    = z_gps;       data.gps_idx  = gps_idx;
    data.z_baro   = z_baro;      data.baro_idx = baro_idx;
    data.z_mag    = z_mag;       data.mag_idx  = mag_idx;
end

% 시나리오 설정 보조 함수
function s = sc(name, label, t_total, pitch_init, wind_amp)
    s.name       = name;
    s.label      = label;
    s.t_total    = t_total;
    s.pitch_init = pitch_init;
    s.wind_amp   = wind_amp;
end

% 참고: quat2dcm 함수가 NavUtils 등에 정의되어 있지 않을 경우를 위한 간이 함수
function dcm = quat2dcm(q)
    % q = [w x y z]
    w = q(1); x = q(2); y = q(3); z = q(4);
    dcm = [1 - 2*(y^2 + z^2),   2*(x*y - w*z),     2*(x*z + w*y);
           2*(x*y + w*z),       1 - 2*(x^2 + z^2), 2*(y*z - w*x);
           2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x^2 + y^2)];
end