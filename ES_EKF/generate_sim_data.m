function data = generate_sim_data(scenario)
%GENERATE_SIM_DATA  검증용 가짜 센서 데이터 생성
%
% 입력: scenario (string)
%   'static'    — 정지 상태 (Step 1,2 검증용)
%   'linear'    — 등속 직선운동 (Step 3 검증용)
%   'accel'     — 등가속 직선운동
%   'turn'      — 수평 선회 (Step 4 검증용)
%
% 출력: data (struct)
%   .t          [1×N]   시간 벡터 [s]
%   .N          [1×1]   총 스텝 수
%   .dt         [1×1]   IMU 샘플주기 [s]
%   .p_true     [3×N]   참 위치 NED [m]
%   .v_true     [3×N]   참 속도 NED [m/s]
%   .q_true     [4×N]   참 쿼터니언 [q0;q1;q2;q3]
%   .ba_true    [3×N]   참 가속도 바이어스 [m/s^2] (검증용 추가)
%   .bg_true    [3×N]   참 자이로 바이어스 [rad/s] (검증용 추가)
%   .a_meas     [3×N]   가속도계 측정값 (노이즈, 바이어스 포함)
%   .w_meas     [3×N]   자이로 측정값 (노이즈, 바이어스 포함)
%   .z_gps      [6×Ng]  GPS 측정값 [pN;pE;pD;vN;vE;vD]
%   .gps_idx    [1×Ng]  GPS 측정 시점 인덱스
%   .z_baro     [1×Nb]  기압 고도계 측정값 [m]
%   .baro_idx   [1×Nb]  Baro 측정 시점 인덱스
%   .z_mag      [3×Nm]  지자기 측정값 (body)
%   .mag_idx    [1×Nm]  Mag 측정 시점 인덱스

    if nargin < 1, scenario = 'static'; end

    %% ── 센서 스펙 (SensorSpec 참조) ─────────────────────────────────────
    S = SensorSpec;
    dt      = S.dt_imu;
    dt_gps  = S.dt_gps;
    dt_baro = S.dt_baro;
    dt_mag  = S.dt_mag;
    
    % 바이어스가 수렴하는 것을 확실히 보기 위해 60초로 넉넉하게 설정
    T       = 60;          % 시뮬 시간 [s]
    
    g_ned = S.g_ned;
    m_ref = S.m_ref_ned / norm(S.m_ref_ned);

    %% ── 시간 벡터 ────────────────────────────────────────────────────────
    N = round(T / dt);
    t = (0:N-1) * dt;

    %% ── 시나리오별 참 궤적 생성 ─────────────────────────────────────────
    p_true = zeros(3, N);
    v_true = zeros(3, N);
    q_true = repmat([1;0;0;0], 1, N);
    a_ned  = zeros(3, N);       % NED 가속도 (중력 제외)
    w_body = zeros(3, N);       % body 각속도

    switch scenario
        case 'static'
            % 원점 정지, 수평 북향 (p, v, q 전부 초기값 유지)
        case 'linear'
            % 북쪽 1 m/s 등속 직선
            for k = 1:N
                v_true(:,k) = [1; 0; 0];
            end
            for k = 2:N
                p_true(:,k) = p_true(:,k-1) + v_true(:,k-1) * dt;
            end
        case 'accel'
            % 북쪽 0.5 m/s² 등가속
            a_const = [0.5; 0; 0];      % NED 가속도
            for k = 1:N
                a_ned(:,k) = a_const;
                v_true(:,k) = a_const * t(k);
            end
            for k = 2:N
                p_true(:,k) = p_true(:,k-1) + v_true(:,k-1)*dt + 0.5*a_const*dt^2;
            end
        case 'turn'
            % 수평 원운동: 반경 50m, 속력 5m/s
            R_turn = 50;
            spd    = 5;
            omega  = spd / R_turn;      % yaw rate [rad/s]
            for k = 1:N
                theta_k = omega * t(k);
                p_true(:,k) = [R_turn*sin(theta_k); R_turn*(1-cos(theta_k)); 0];
                v_true(:,k) = [spd*cos(theta_k); spd*sin(theta_k); 0];
                % 쿼터니언: yaw = theta_k, roll=pitch=0
                q_true(:,k) = [cos(theta_k/2); 0; 0; sin(theta_k/2)];
                % NED 가속도 (구심 가속도)
                a_ned(:,k) = [-spd*omega*sin(theta_k); spd*omega*cos(theta_k); 0];
                % body 각속도: yaw rate만 존재
                w_body(:,k) = [0; 0; omega];
            end
        otherwise
            error('Unknown scenario: %s', scenario);
    end

    %% ── IMU 참값 역산 ────────────────────────────────────────────────────
    a_body_true = zeros(3, N);
    for k = 1:N
        R_nb = NavUtils.quat2dcm(q_true(:,k));
        % 비력 = R_nb' * (a_ned - g)   (가속도계가 실제로 측정하는 값)
        a_body_true(:,k) = R_nb' * (a_ned(:,k) - g_ned);
    end

    %% ── 가짜 바이어스 정의 (필터 검증용) ─────────────────────────────
    true_ba = [0.05; -0.02; 0.01];   % [m/s^2]
    true_bg = [0.001; -0.002; 0.003]; % [rad/s]

    %% ── IMU 측정값 (노이즈 및 바이어스 추가) ─────────────────────────
    % PSD 스펙을 이산 시간 분산으로 변환하기 위해 dt로 나눔
    noise_acc  = sqrt(S.var_acc / dt)  * randn(3, N);
    noise_gyro = sqrt(S.var_gyro / dt) * randn(3, N);

    data.a_meas = a_body_true + true_ba + noise_acc;
    data.w_meas = w_body      + true_bg + noise_gyro;

    %% ── GPS 측정값 ───────────────────────────────────────────────────────
    gps_step  = round(dt_gps / dt);
    data.gps_idx = 1:gps_step:N;
    Ng = length(data.gps_idx);
    data.z_gps = zeros(6, Ng);
    
    for i = 1:Ng
        ki = data.gps_idx(i);
        np = [S.sigma_gps_pos_h; S.sigma_gps_pos_h; S.sigma_gps_pos_v] .* randn(3,1);
        nv = [S.sigma_gps_vel_h; S.sigma_gps_vel_h; S.sigma_gps_vel_v] .* randn(3,1);
        data.z_gps(:,i) = [p_true(:,ki) + np; v_true(:,ki) + nv];
    end

    %% ── Baro 측정값 ─────────────────────────────────────────────────────
    baro_step = round(dt_baro / dt);
    data.baro_idx = 1:baro_step:N;
    Nb = length(data.baro_idx);
    data.z_baro = zeros(1, Nb);
    
    for i = 1:Nb
        ki = data.baro_idx(i);
        % 측정 센서는 이산값이므로 dt를 나누지 않고 분산값을 그대로 사용
        data.z_baro(i) = -p_true(3,ki) + sqrt(S.var_baro) * randn;
    end

    %% ── Mag 측정값 ──────────────────────────────────────────────────────
    mag_step = round(dt_mag / dt);
    data.mag_idx = 1:mag_step:N;
    Nm = length(data.mag_idx);
    data.z_mag = zeros(3, Nm);
    
    for i = 1:Nm
        ki = data.mag_idx(i);
        R_nb = NavUtils.quat2dcm(q_true(:,ki));
        data.z_mag(:,i) = R_nb' * m_ref + sqrt(S.var_mag) * randn(3,1);
    end

    %% ── 공통 출력 ────────────────────────────────────────────────────────
    data.t      = t;
    data.N      = N;
    data.dt     = dt;
    data.p_true = p_true;
    data.v_true = v_true;
    data.q_true = q_true;
    
    % 검증을 위해 바이어스 참값도 출력 데이터에 포함
    data.ba_true = repmat(true_ba, 1, N);
    data.bg_true = repmat(true_bg, 1, N);
end