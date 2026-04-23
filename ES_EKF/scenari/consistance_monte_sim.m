clear; clc; close all;
addpath('..');

% ═══════════════════════════════════════════════════════════════════════
% 설정
% ═══════════════════════════════════════════════════════════════════════
CSV_PATH   = 'openrocket.csv';
N_TRIALS   = 200; % 몬테카를로 반복 횟수
BASE_SEED  = 42; % 난수 생성기 시드(seed) 기준값

S  = SensorSpec;
dt = S.dt_imu;          % 416 Hz → dt ≈ 2.4e-3 s

% OpenRocket CSV 파싱
raw = readmatrix(CSV_PATH, 'CommentStyle', '#');
t_or      = raw(:, 1);
acc_or    = raw(:, 8:10);     % [X, Y, Z]_OR body acc (m/s²)
rate_or   = raw(:, 15:17);    % [Roll(Z), Pitch(Y), Yaw(X)]_OR (deg/s)
zenith0   = raw(1, 18);       % deg
azimuth0  = raw(1, 19);       % deg

% MATLAB Body Frame 정렬
acc_mat  = [acc_or(:,3),  acc_or(:,1),  acc_or(:,2)];  % [m/s²]
rate_mat = deg2rad([rate_or(:,1), rate_or(:,3), rate_or(:,2)]);  % [rad/s]

t_end = t_or(end);
t     = (0 : dt : t_end)';
N     = length(t);

acc_u  = interp1(t_or, acc_mat,  t, 'linear', 'extrap');   % [N×3]
rate_u = interp1(t_or, rate_mat, t, 'linear', 'extrap');   % [N×3]

% ═══════════════════════════════════════════════════════════════════════
% 초기 자세 (zenith, azimuth at t=0)
% ═══════════════════════════════════════════════════════════════════════
yaw_init   = deg2rad(azimuth0);
pitch_init = -deg2rad(zenith0);
roll_init  = 0;
q_init = quaternion([yaw_init, pitch_init, roll_init], 'euler', 'ZYX', 'frame');

% ═══════════════════════════════════════════════════════════════════════
% kinematicTrajectory → Ground Truth
% ═══════════════════════════════════════════════════════════════════════
traj = kinematicTrajectory('SampleRate', round(1/dt), ...
        'Position', [0 0 0], 'Velocity', [0 0 0], 'Orientation', q_init);
[pos_ned, orient, vel_ned] = traj(acc_u, rate_u);

p_true = pos_ned';                  % [3×N] NED
v_true = vel_ned';                  % [3×N] NED
q_true = compact(orient)';          % [4×N] [w; x; y; z]

fprintf('=== ES-EKF 일관성 검증 (OpenRocket truth + ZUPT 적용) ===\n');
fprintf('  시뮬 시간 : %.2f s  (N = %d 샘플, dt = %.4e s)\n', t_end, N, dt);
fprintf('  최대고도  : %.1f m\n', -min(-p_true(3,:)));
fprintf('  반복횟수  : %d trial\n\n', N_TRIALS);

% ═══════════════════════════════════════════════════════════════════════
% 측정 인덱스 (GPS / Baro / Mag) 및 참(Truth) 센서 데이터 생성
% ═══════════════════════════════════════════════════════════════════════
gps_idx  = 1 : round(S.dt_gps  / dt) : N;
baro_idx = 1 : round(S.dt_baro / dt) : N;
mag_idx  = 1 : round(S.dt_mag  / dt) : N;
m_ref_unit = S.m_ref_ned / norm(S.m_ref_ned);

% ── 참 센서 바이어스 (flight 상수) ────────────────────────────────────
true_ba = [0.05; -0.02;  0.01  ];      % [m/s²]
true_bg = [0.001; -0.002; 0.003];      % [rad/s]

% ── 동역학적 가속도를 관성센서 비력(Specific Force)으로 변환 ───
g_ned = [0; 0; 9.80665]; % NED 기준 중력
f_body_true = zeros(3, N);
for k = 1:N
    R_bn = local_quat2dcm(q_true(:, k)');
    g_body = R_bn' * g_ned; % NED 중력을 Body 프레임으로 투영
    f_body_true(:, k) = acc_u(k, :)' - g_body; % 비력 = 가속도 - 중력
end
w_body_true = rate_u';       % [3×N]

% ═══════════════════════════════════════════════════════════════════════
% 결과 저장 배열
% ═══════════════════════════════════════════════════════════════════════
err_p   = zeros(N_TRIALS, N);
err_v   = zeros(N_TRIALS, N);
err_ba  = zeros(N_TRIALS, N);
err_bg  = zeros(N_TRIALS, N);
nees_pv = zeros(N_TRIALS, N);

% ── 필터의 오차 공분산(Variance) 합을 저장할 배열 ──
cov_p   = zeros(N_TRIALS, N);
cov_v   = zeros(N_TRIALS, N);
cov_ba  = zeros(N_TRIALS, N);
cov_bg  = zeros(N_TRIALS, N);

% ═══════════════════════════════════════════════════════════════════════
% Monte Carlo 루프
% ═══════════════════════════════════════════════════════════════════════
for trial = 1:N_TRIALS
    rng(BASE_SEED + trial);
    
    std_acc  = sqrt(S.var_acc  / dt); 
    std_gyro = sqrt(S.var_gyro / dt); 
    
    a_meas = f_body_true + true_ba + std_acc  * randn(3, N); 
    w_meas = w_body_true + true_bg + std_gyro * randn(3, N); 
    
    sig_p = sqrt([S.var_gps_pos_h; S.var_gps_pos_h; S.var_gps_pos_v]);
    sig_v = sqrt([S.var_gps_vel_h; S.var_gps_vel_h; S.var_gps_vel_v]);
    z_gps = [p_true(:,gps_idx) + sig_p .* randn(3, length(gps_idx));
             v_true(:,gps_idx) + sig_v .* randn(3, length(gps_idx))];
             
    z_baro = -p_true(3, baro_idx) + sqrt(S.var_baro) * randn(1, length(baro_idx));
    
    z_mag = zeros(3, length(mag_idx));
    for k = 1:length(mag_idx)
        R_nb = local_quat2dcm(q_true(:, mag_idx(k))');
        z_mag(:,k) = R_nb' * m_ref_unit + sqrt(S.var_mag) * randn(3,1);
    end
    
    % ── EKF 초기화 (초기 위치/속도 오차 부여) ───────────────────────────
    p0 = p_true(:,1) + [0.5; 0.5; 1.0] .* randn(3,1);
    v0 = v_true(:,1) + [0.1; 0.1; 0.2] .* randn(3,1);
    q0 = q_true(:,1);
    ekf = ESEKF(p0, v0, q0);
    
    % 🚀 =================================================================
    % [ZUPT 추가 구간] 비행 시작 전 발사대에서 5초간 대기하며 정렬 수행
    % ====================================================================
    T_zupt = 5.0; % 5초 대기
    N_zupt = round(T_zupt / dt);
    
    % 발사대 정지 상태 참값 (가속도는 0이므로 중력의 반대 방향 비력만 측정)
    R_bn_init = local_quat2dcm(q_true(:,1)');
    f_zupt_true = R_bn_init' * (-g_ned);
    w_zupt_true = [0; 0; 0];
    
    for k_z = 1:N_zupt
        % 센서 노이즈 및 고정 바이어스 주입
        a_meas_zupt = f_zupt_true + true_ba + std_acc * randn(3,1);
        w_meas_zupt = w_zupt_true + true_bg + std_gyro * randn(3,1);
        
        % 예측 단계
        ekf.predict(a_meas_zupt, w_meas_zupt, dt);
        
        % 10Hz(0.1초) 주기로 ZUPT 업데이트 수행
        if mod(k_z, round(0.1 / dt)) == 0
            % (주의: ESEKF.m 클래스에 update_zupt 함수가 선언되어 있어야 합니다)
            ekf.update_zupt(); 
        end
    end
    % ====================================================================
    % ZUPT 종료. 이제 ekf는 바이어스와 초기 오차가 거의 보상된 상태로 비행을 시작합니다.
    
    gps_ptr = 1;  baro_ptr = 1;  mag_ptr = 1;
    
    % ── EKF 루프 (비행 시작) ────────────────────────────────────────────────
    for k = 1:N
        ekf.predict(a_meas(:,k), w_meas(:,k), dt);
        
        if gps_ptr <= length(gps_idx) && k == gps_idx(gps_ptr)
            ekf.update_gps(z_gps(:, gps_ptr));
            gps_ptr = gps_ptr + 1;
        end
        if baro_ptr <= length(baro_idx) && k == baro_idx(baro_ptr)
            ekf.update_baro(z_baro(baro_ptr));
            baro_ptr = baro_ptr + 1;
        end
        if mag_ptr <= length(mag_idx) && k == mag_idx(mag_ptr)
            ekf.update_mag(z_mag(:, mag_ptr));
            mag_ptr = mag_ptr + 1;
        end
        
        % 오차 기록
        ep  = ekf.nom.p   - p_true(:, k);
        ev  = ekf.nom.v   - v_true(:, k);
        eba = ekf.nom.b_a - true_ba;
        ebg = ekf.nom.b_g - true_bg;
        
        err_p(trial, k)  = norm(ep);
        err_v(trial, k)  = norm(ev);
        err_ba(trial, k) = norm(eba);
        err_bg(trial, k) = norm(ebg);
        
        % P 행렬에서 3차원 분산의 합(Trace) 추출
        cov_p(trial, k)  = trace(ekf.par.P(1:3, 1:3));
        cov_v(trial, k)  = trace(ekf.par.P(4:6, 4:6));
        cov_ba(trial, k) = trace(ekf.par.P(10:12, 10:12)); 
        cov_bg(trial, k) = trace(ekf.par.P(13:15, 13:15)); 
        
        e_pv  = [ep; ev];
        P_pv  = ekf.par.P(1:6, 1:6);
        nees_pv(trial, k) = e_pv' * (P_pv \ e_pv);
    end
end

% ═══════════════════════════════════════════════════════════════════════
% 통계 & 필터 3-Sigma 상한선 도출
% ═══════════════════════════════════════════════════════════════════════
mean_p    = mean(err_p,   1);
mean_v    = mean(err_v,   1);
mean_ba   = mean(err_ba,  1);
mean_bg   = mean(err_bg,  1);
mean_nees = mean(nees_pv, 1);

% 필터가 추정한 3-Sigma 상한선 계산 (Norm 기준)
bound_p  = 3 * sqrt(mean(cov_p, 1));
bound_v  = 3 * sqrt(mean(cov_v, 1));
bound_ba = 3 * sqrt(mean(cov_ba, 1));
bound_bg = 3 * sqrt(mean(cov_bg, 1));

last_n   = round(5 / dt);
idx_last = (N - last_n + 1) : N;
nees_last = mean(mean_nees(idx_last));

fprintf('\n─── 수렴 후 성능 (마지막 5초) ───\n');
fprintf('  위치 평균오차: %.3f m\n', mean(mean_p(idx_last)));
fprintf('  속도 평균오차: %.4f m/s\n', mean(mean_v(idx_last)));
fprintf('  NEES (6DOF)  : %.2f  (이상값 = 6)\n', nees_last);

% ═══════════════════════════════════════════════════════════════════════
% 시각화
% ═══════════════════════════════════════════════════════════════════════
figure('Name', sprintf('Consistency Sim with ZUPT [N=%d]', N_TRIALS), ...
       'Position', [50, 50, 1200, 780]);

% 1. 위치 오차
subplot(2,3,1); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_p(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
plot(t, mean_p, 'k', 'LineWidth', 2.5, 'DisplayName', '앙상블 평균');
plot(t, bound_p, 'r--', 'LineWidth', 2.0, 'DisplayName', '필터 +3\sigma'); 
legend('Location','northeast');
xlabel('Time [s]'); ylabel('Norm 오차 [m]'); title('위치 오차 (Norm)');

% 2. 속도 오차
subplot(2,3,2); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_v(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
plot(t, mean_v, 'k', 'LineWidth', 2.5, 'DisplayName', '앙상블 평균');
plot(t, bound_v, 'r--', 'LineWidth', 2.0, 'DisplayName', '필터 +3\sigma');
legend('Location','northeast');
xlabel('Time [s]'); ylabel('Norm 오차 [m/s]'); title('속도 오차 (Norm)');

% 3. 가속도 바이어스
subplot(2,3,3); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_ba(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
plot(t, mean_ba, 'k', 'LineWidth', 2.5, 'DisplayName', '앙상블 평균');
plot(t, bound_ba, 'r--', 'LineWidth', 2.0, 'DisplayName', '필터 +3\sigma');
legend('Location','northeast');
xlabel('Time [s]'); ylabel('Norm 오차 [m/s²]'); title('가속도 바이어스 오차 (Norm)');

% 4. 자이로 바이어스
subplot(2,3,4); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_bg(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
plot(t, mean_bg, 'k', 'LineWidth', 2.5, 'DisplayName', '앙상블 평균');
plot(t, bound_bg, 'r--', 'LineWidth', 2.0, 'DisplayName', '필터 +3\sigma');
legend('Location','northeast');
xlabel('Time [s]'); ylabel('Norm 오차 [rad/s]'); title('자이로 바이어스 오차 (Norm)');

% 5. NEES
subplot(2,3,5); hold on; grid on;
plot(t, mean_nees, 'k',   'LineWidth', 2.5, 'DisplayName', 'NEES');
yline(6,  'g--', 'LineWidth', 2.0, 'DisplayName', '이상값 = 6');
yline(3,  'r:',  'LineWidth', 1.5, 'DisplayName', '과신 경계');
yline(12, 'r:',  'LineWidth', 1.5, 'DisplayName', '과소신뢰 경계');
ylim([0, 30]); legend('Location','northeast');
xlabel('Time [s]'); ylabel('NEES'); title('NEES 일관성 (≈6)');

% 6. 오차 분포 히스토그램
subplot(2,3,6); hold on; grid on;
histogram(err_p(:,end), 15, 'FaceColor', [0.3 0.5 0.8]);
xline(mean(err_p(:,end)),   'r-',  'LineWidth', 2, ...
    'DisplayName', sprintf('평균=%.2f m', mean(err_p(:,end))));
xline(median(err_p(:,end)), 'g--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('중앙값=%.2f m', median(err_p(:,end))));
legend('Location','northeast');
xlabel('최종 위치 오차 (Norm) [m]'); ylabel('빈도'); title('최종 위치 오차 분포');

sgtitle(sprintf('ES-EKF Consistency w/ ZUPT [OpenRocket truth, N=%d]', N_TRIALS));

% ═══════════════════════════════════════════════════════════════════════
% Helper: quaternion [w x y z] → body-to-NED DCM
% ═══════════════════════════════════════════════════════════════════════
function dcm = local_quat2dcm(q)
    w = q(1); x = q(2); y = q(3); z = q(4);
    dcm = [1 - 2*(y^2 + z^2),   2*(x*y - w*z),     2*(x*z + w*y);
           2*(x*y + w*z),       1 - 2*(x^2 + z^2), 2*(y*z - w*x);
           2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x^2 + y^2)];
end