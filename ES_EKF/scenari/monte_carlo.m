%% monte_carlo.m
% generate_scenarios.m이 만든 .mat 파일을 불러와서
% ES-EKF를 N_trials번 반복 실행하고 성능을 통계적으로 검증한다.
%
% 실행 순서:
%   1. generate_scenarios.m 실행 → .mat 파일 생성
%   2. monte_carlo.m 실행        → EKF 성능 통계
%
% 검증 지표:
%   - 위치/속도 RMSE 평균 및 분포
%   - 바이어스 수렴 여부
%   - NEES (Normalized Estimation Error Squared): 필터 일관성
%       NEES ≈ 6 이면 OK (위치 3 + 속도 3 차원 기준)

clear; clc; close all;
addpath('..');

% ═══════════════════════════════════════════════════════════════════════
% 설정
% ═══════════════════════════════════════════════════════════════════════

TARGET_SCENARIO = 'vertical';   % 검증할 시나리오: vertical / tilted / windy
N_TRIALS        = 50;           % 반복 횟수 (많을수록 통계 신뢰도 상승)
BASE_SEED       = 42;           % 재현성을 위한 난수 시드

% ─────────────────────────────────────────────────────────────────────
% .mat 파일 로드 (Ground Truth 궤적 재사용, 노이즈만 매번 다르게 생성)
% ─────────────────────────────────────────────────────────────────────
filename = sprintf('scenario_%s.mat', TARGET_SCENARIO);
assert(isfile(filename), ...
    '파일이 없습니다: %s\n먼저 generate_scenarios.m을 실행하세요.', filename);

loaded = load(filename);
d0  = loaded.data;       % 기준 데이터 (Ground Truth 참조용)
N   = d0.N;
dt  = d0.dt;
t   = d0.t;
S   = SensorSpec;

fprintf('=== ES-EKF 몬테카를로 시뮬레이션 ===\n');
fprintf('  시나리오: %s (%s)\n', TARGET_SCENARIO, d0.label);
fprintf('  반복 횟수: %d회,  시간: %.0fs\n\n', N_TRIALS, t(end));

% ═══════════════════════════════════════════════════════════════════════
% 결과 저장 배열
% ═══════════════════════════════════════════════════════════════════════

err_p   = zeros(N_TRIALS, N);   % 위치 오차 크기 [m]
err_v   = zeros(N_TRIALS, N);   % 속도 오차 크기 [m/s]
err_ba  = zeros(N_TRIALS, N);   % 가속도 바이어스 오차 [m/s²]
err_bg  = zeros(N_TRIALS, N);   % 자이로 바이어스 오차 [rad/s]
nees_pv = zeros(N_TRIALS, N);   % NEES (위치+속도 6차원)

% ═══════════════════════════════════════════════════════════════════════
% 몬테카를로 루프
% ═══════════════════════════════════════════════════════════════════════
%
% Ground Truth(p_true, v_true, q_true)는 고정, 센서 노이즈만 매번 다르게 생성
% → 필터 알고리즘 자체의 통계적 성능을 측정

for trial = 1:N_TRIALS
    rng(BASE_SEED + trial);   % 재현 가능한 랜덤 시드

    % 이번 trial의 IMU 노이즈 (Ground Truth는 d0에서 그대로 사용)
    std_acc  = sqrt(S.var_acc  / dt);
    std_gyro = sqrt(S.var_gyro / dt);
    a_meas = d0.ba_true + d0.a_meas * 0;   % 초기화용 (아래서 덮어씀)

    % 참 비력 역산: a_meas - ba_true = f_body_true → 새 노이즈 추가
    f_body_true = d0.a_meas - d0.ba_true;   % [3×N] 참 비력
    w_true      = d0.w_meas - d0.bg_true;   % [3×N] 참 각속도
    a_meas_trial = f_body_true + d0.ba_true + std_acc  * randn(3, N);
    w_meas_trial = w_true      + d0.bg_true + std_gyro * randn(3, N);

    % ── EKF 초기화 ────────────────────────────────────────────────────
    % 초기 상태에 현실적인 불확실성 추가
    p0 = d0.p_true(:,1) + [0.5; 0.5; 1.0] .* randn(3,1);
    v0 = d0.v_true(:,1) + [0.1; 0.1; 0.2] .* randn(3,1);
    q0 = d0.q_true(:,1);
    ekf = ESEKF(p0, v0, q0);

    % 측정 인덱스 포인터
    gps_ptr  = 1;
    baro_ptr = 1;
    mag_ptr  = 1;

    % ── EKF 루프 ──────────────────────────────────────────────────────
    for k = 1:N
        % [Predict]
        ekf.predict(a_meas_trial(:,k), w_meas_trial(:,k), dt);

        % [Update - GPS] 25 Hz
        if gps_ptr <= length(d0.gps_idx) && k == d0.gps_idx(gps_ptr)
            ekf.update_gps(d0.z_gps(:, gps_ptr));
            gps_ptr = gps_ptr + 1;
        end

        % [Update - Baro] 50 Hz
        if baro_ptr <= length(d0.baro_idx) && k == d0.baro_idx(baro_ptr)
            ekf.update_baro(d0.z_baro(baro_ptr));
            baro_ptr = baro_ptr + 1;
        end

        % [Update - Mag] 50 Hz
        if mag_ptr <= length(d0.mag_idx) && k == d0.mag_idx(mag_ptr)
            ekf.update_mag(d0.z_mag(:, mag_ptr));
            mag_ptr = mag_ptr + 1;
        end

        % ── 오차 기록 ──────────────────────────────────────────────
        ep  = ekf.nom.p   - d0.p_true(:, k);
        ev  = ekf.nom.v   - d0.v_true(:, k);
        eba = ekf.nom.b_a - d0.ba_true(:, k);
        ebg = ekf.nom.b_g - d0.bg_true(:, k);

        err_p(trial, k)  = norm(ep);
        err_v(trial, k)  = norm(ev);
        err_ba(trial, k) = norm(eba);
        err_bg(trial, k) = norm(ebg);

        % NEES (위치+속도 6차원 서브셋)
        e_pv   = [ep; ev];
        P_pv   = ekf.par.P(1:6, 1:6);
        nees_pv(trial, k) = e_pv' * (P_pv \ e_pv);
    end

    if mod(trial, 10) == 0 || trial == 1
        fprintf('  Trial %3d/%d  |  최종 위치 오차 = %.2f m\n', ...
            trial, N_TRIALS, err_p(trial, end));
    end
end

% ═══════════════════════════════════════════════════════════════════════
% 통계 계산 및 요약 출력
% ═══════════════════════════════════════════════════════════════════════

mean_p  = mean(err_p,   1);    std_p  = std(err_p,  0, 1);
mean_v  = mean(err_v,   1);
mean_ba = mean(err_ba,  1);
mean_bg = mean(err_bg,  1);
mean_nees = mean(nees_pv, 1);  % 기댓값 = 6

% 마지막 5초 평균으로 수렴 후 성능 판단
last_n   = round(5 / dt);
idx_last = (N - last_n + 1) : N;
nees_mean_last = mean(mean_nees(idx_last));

fprintf('\n─── 수렴 후 성능 요약 (마지막 5초) ───\n');
fprintf('  위치 RMSE    : %.3f ± %.3f m\n',     mean(mean_p(idx_last)),  mean(std_p(idx_last)));
fprintf('  속도 RMSE    : %.4f m/s\n',           mean(mean_v(idx_last)));
fprintf('  가속도 Bias  : %.5f m/s²\n',          mean(mean_ba(idx_last)));
fprintf('  자이로 Bias  : %.6f rad/s\n',         mean(mean_bg(idx_last)));
fprintf('  NEES (6DOF)  : %.2f  (이상값=6.0)\n', nees_mean_last);

% 일관성 판정
ratio = nees_mean_last / 6;
if     ratio < 0.5,  fprintf('  → [경고] 필터 과신 (P가 너무 작음)\n');
elseif ratio > 2.0,  fprintf('  → [경고] 필터 과소신뢰 (P가 너무 큼)\n');
else,                fprintf('  → [OK] 필터 일관성 양호\n');
end

% ═══════════════════════════════════════════════════════════════════════
% 시각화
% ═══════════════════════════════════════════════════════════════════════

figure('Name', sprintf('몬테카를로 결과 [%s, N=%d]', TARGET_SCENARIO, N_TRIALS), ...
       'Position', [50, 50, 1100, 750]);

% ── 위치 RMSE 분포 ───────────────────────────────────────────────────
subplot(2, 3, 1); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_p(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
end
plot(t, mean_p,           'k',   'LineWidth', 2.5, 'DisplayName', '평균');
plot(t, mean_p + std_p,   'b--', 'LineWidth', 1.5, 'DisplayName', '±1σ');
plot(t, mean_p - std_p,   'b--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
legend('Location', 'northeast');
xlabel('Time [s]'); ylabel('오차 [m]'); title('위치 오차');

% ── 속도 RMSE ────────────────────────────────────────────────────────
subplot(2, 3, 2); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_v(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
end
plot(t, mean_v, 'b', 'LineWidth', 2.5);
xlabel('Time [s]'); ylabel('오차 [m/s]'); title('속도 오차');

% ── 가속도 바이어스 수렴 ─────────────────────────────────────────────
subplot(2, 3, 3); hold on; grid on;
plot(t, mean_ba, 'r', 'LineWidth', 2.5);
xlabel('Time [s]'); ylabel('오차 [m/s²]'); title('가속도 바이어스 수렴');

% ── 자이로 바이어스 수렴 ─────────────────────────────────────────────
subplot(2, 3, 4); hold on; grid on;
plot(t, mean_bg, 'm', 'LineWidth', 2.5);
xlabel('Time [s]'); ylabel('오차 [rad/s]'); title('자이로 바이어스 수렴');

% ── NEES 일관성 검증 ─────────────────────────────────────────────────
subplot(2, 3, 5); hold on; grid on;
plot(t, mean_nees, 'k', 'LineWidth', 2.5, 'DisplayName', 'NEES');
yline(6,   'g--', 'LineWidth', 2.0, 'DisplayName', '이상값 = 6');
yline(3,   'r:',  'LineWidth', 1.5, 'DisplayName', '과신 경계');
yline(12,  'r:',  'LineWidth', 1.5, 'DisplayName', '과소신뢰 경계');
ylim([0, 30]); legend('Location', 'northeast');
xlabel('Time [s]'); ylabel('NEES'); title('NEES 일관성 (≈6 이 이상적)');

% ── 최종 위치 오차 히스토그램 ────────────────────────────────────────
subplot(2, 3, 6); hold on; grid on;
histogram(err_p(:, end), 15, 'FaceColor', [0.3 0.5 0.8]);
xline(mean(err_p(:,end)),   'r-',  'LineWidth', 2, ...
    'DisplayName', sprintf('평균=%.2fm', mean(err_p(:,end))));
xline(median(err_p(:,end)), 'g--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('중앙값=%.2fm', median(err_p(:,end))));
legend('Location', 'northeast');
xlabel('최종 위치 오차 [m]'); ylabel('빈도'); title('최종 위치 오차 분포');

sgtitle(sprintf('ES-EKF 몬테카를로 [시나리오: %s, N=%d]', TARGET_SCENARIO, N_TRIALS));

% ═══════════════════════════════════════════════════════════════════════
% 결과 저장
% ═══════════════════════════════════════════════════════════════════════

save('mc_results.mat', 'err_p', 'err_v', 'err_ba', 'err_bg', ...
     'nees_pv', 't', 'N_TRIALS', 'TARGET_SCENARIO');
fprintf('\n결과 저장 완료: mc_results.mat\n');
