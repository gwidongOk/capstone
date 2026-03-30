% test_esekf.m
clear; clc; close all;

%% 1. 가짜 센서 데이터 불러오기 (시나리오 선택)
% 'static', 'linear', 'accel', 'turn' 중 선택
scenario_name = 'accel'; 
disp(['시뮬레이션 데이터 생성 중... (시나리오: ', scenario_name, ')']);
data = generate_sim_data(scenario_name);

N = data.N;
t_imu = data.t;

%% 2. 필터 초기화 및 악조건(Robustness) 셋팅
% 참값 초기 상태
p0_true = data.p_true(:,1);
v0_true = data.v_true(:,1);
q0_true = data.q_true(:,1);

p0_est = p0_true + [5; -3; 2];% 북쪽 5m, 동쪽 -3m, 아래 2m 틀리게 시작
v0_est = v0_true*1.5+ [0.5; 0.5; -0.2];% 속도도 살짝 틀리게
%초기 자세
q0_est = TRIAD_Initial_Alignment(data.a_meas(:,1),data.z_mag(:,1));


% 필터 객체 생성 (일부러 틀린 자세로 시작)
ekf = ESEKF(p0_est, v0_est, q0_est);

% 데이터 저장을 위한 배열 할당
err_p = zeros(3, N);
err_v = zeros(3, N);
err_att = zeros(3, N);
err_ba = zeros(3, N);
err_bg = zeros(3, N);

sig3_p = zeros(3, N);
sig3_v = zeros(3, N);
sig3_att = zeros(3, N);
sig3_ba = zeros(3, N);
sig3_bg = zeros(3, N);

%% 3. 초기 상태 기록
err_p(:, 1)  = data.p_true(:,1) - ekf.nom.p;
err_v(:, 1)  = data.v_true(:,1) - ekf.nom.v;
err_ba(:, 1) = data.ba_true(:,1) - ekf.nom.b_a;
err_bg(:, 1) = data.bg_true(:,1) - ekf.nom.b_g;

q_err = NavUtils.quat_mult(NavUtils.quat_conj(data.q_true(:,1)), ekf.nom.q);
if q_err(1) < 0, q_err = -q_err; end
err_att(:, 1) = 2 * q_err(2:4);

P_diag = diag(ekf.par.P);
sig3_p(:, 1)   = 3 * sqrt(P_diag(1:3));
sig3_v(:, 1)   = 3 * sqrt(P_diag(4:6));
sig3_att(:, 1) = 3 * sqrt(P_diag(7:9));
sig3_ba(:, 1)  = 3 * sqrt(P_diag(10:12));
sig3_bg(:, 1)  = 3 * sqrt(P_diag(13:15));

%% 4. 메인 시뮬레이션 루프
for k = 2:N
    % ── [1] Predict (IMU는 매 스텝 들어옴) ─────────────────────────
    ekf.predict(data.a_meas(:,k), data.w_meas(:,k), data.dt);

    % ── [2] Update (각 센서의 인덱스와 일치할 때만 실행) ──────────────

    % GPS 업데이트 로직
    idx_gps = find(data.gps_idx == k, 1);
    if ~isempty(idx_gps)
         ekf.update_gps(data.z_gps(:, idx_gps));
    end

    % Baro 업데이트 로직
    idx_baro = find(data.baro_idx == k, 1);
    if ~isempty(idx_baro)
        ekf.update_baro(data.z_baro(idx_baro));
    end

    %Mag 업데이트 로직
    idx_mag = find(data.mag_idx == k, 1);
    if ~isempty(idx_mag)
        ekf.update_mag(data.z_mag(:, idx_mag));
    end

    % ── [3] 오차(Error) 및 3-Sigma 바운드 저장 ───────────────────────
    % 위치/속도/바이어스 오차 (True - Est)
    err_p(:, k)  = data.p_true(:,k) - ekf.nom.p;
    err_v(:, k)  = data.v_true(:,k) - ekf.nom.v;
    err_ba(:, k) = data.ba_true(:,k) - ekf.nom.b_a;
    err_bg(:, k) = data.bg_true(:,k) - ekf.nom.b_g;

    % 자세 오차 (쿼터니언 기반, angle wrapping 문제 없음)
    q_err = NavUtils.quat_mult(NavUtils.quat_conj(data.q_true(:,k)), ekf.nom.q);
    if q_err(1) < 0, q_err = -q_err; end
    err_att(:, k) = 2 * q_err(2:4);  % [rad] 소각 근사

    % 3-Sigma 바운드 (공분산 행렬 P의 대각성분 추출)
    P_diag = diag(ekf.par.P);
    sig3_p(:, k)   = 3 * sqrt(P_diag(1:3));
    sig3_v(:, k)   = 3 * sqrt(P_diag(4:6));
    sig3_att(:, k) = 3 * sqrt(P_diag(7:9));
    sig3_ba(:, k)  = 3 * sqrt(P_diag(10:12));
    sig3_bg(:, k)  = 3 * sqrt(P_diag(13:15));
end
disp('시뮬레이션 완료! 플롯을 생성합니다.');

%% 5. 결과 플롯 (Tab 기반 UI)
main_fig = figure('Name', 'ES-EKF 3-Sigma Analysis (Robustness Test)', 'Position', [100, 100, 1100, 750]);
tg = uitabgroup(main_fig);

plot_3sigma_tab(uitab(tg, 'Title', 'Position'), t_imu, err_p, sig3_p, 'Position Error [m]', {'North', 'East', 'Down'});
plot_3sigma_tab(uitab(tg, 'Title', 'Velocity'), t_imu, err_v, sig3_v, 'Velocity Error [m/s]', {'V_N', 'V_E', 'V_D'});
plot_3sigma_tab(uitab(tg, 'Title', 'Attitude'), t_imu, rad2deg(err_att), rad2deg(sig3_att), 'Attitude Error [deg]', {'Roll', 'Pitch', 'Yaw'});
plot_3sigma_tab(uitab(tg, 'Title', 'Gyro Bias'), t_imu, err_bg, sig3_bg, 'Gyro Bias Error [rad/s]', {'X', 'Y', 'Z'});
plot_3sigma_tab(uitab(tg, 'Title', 'Acc Bias'), t_imu, err_ba, sig3_ba, 'Acc Bias Error [m/s^2]', {'X', 'Y', 'Z'});

% ── 보조 플롯 함수 ──────────────────────────────────────────
function plot_3sigma_tab(parent_tab, t, err, sig3, fig_title, labels)
    colors = ['r', 'g', 'b'];
    for i = 1:3
        ax = subplot(3, 1, i, 'Parent', parent_tab);
        
        % 3-Sigma 바운드 (회색 점선)
        plot(ax, t, sig3(i,:), 'Color', [0.6 0.6 0.6], 'LineStyle', '--', 'LineWidth', 1.5); hold(ax, 'on');
        plot(ax, t, -sig3(i,:), 'Color', [0.6 0.6 0.6], 'LineStyle', '--', 'LineWidth', 1.5);
        
        % 실제 오차 (실선)
        plot(ax, t, err(i,:), colors(i), 'LineWidth', 1.2);
    
        
        ylabel(ax, labels{i}); grid(ax, 'on');
        if i == 1, title(ax, fig_title); end
        if i == 3, xlabel(ax, 'Time [s]'); end
        
        max_val = max(abs(err(i,:))) * 1.5; % 오차 기준 자동 스케일링
        if max_val > 1e-6
            ylim(ax, [-max_val, max_val]); 
        end
    end
end