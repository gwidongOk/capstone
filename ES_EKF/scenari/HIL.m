clear; clc; close all;
addpath('..');

% --- 귀찮은 매트랩 시리얼 경고창 끄기 ---
warning('off', 'serialport:serialport:ReadWarning');

% ── 설정 ─────────────────────────────────────────────
CSV_PATH = 'openrocket.csv';
PORT     = "COM7";                       % ← 본인 포트 확인
BAUD     = 115200;                       % 안정적인 속도
SEED     = 42;

S  = SensorSpec;
dt = S.dt_imu;                           

% ── OpenRocket → ground truth ───────────────────────
raw       = readmatrix(CSV_PATH, 'CommentStyle', '#');
t_or      = raw(:,1);
acc_or    = raw(:,8:10);                 
rate_or   = raw(:,15:17);                
zenith0   = raw(1,18);
azimuth0  = raw(1,19);

acc_mat  = [acc_or(:,3),  acc_or(:,1),  acc_or(:,2)];
rate_mat = deg2rad([rate_or(:,1), rate_or(:,3), rate_or(:,2)]);

t_end = t_or(end);
t     = (0:dt:t_end)';
N     = length(t);
acc_u  = interp1(t_or, acc_mat,  t, 'linear','extrap');
rate_u = interp1(t_or, rate_mat, t, 'linear','extrap');

q_init = quaternion([deg2rad(azimuth0), -deg2rad(zenith0), 0], 'euler','ZYX','frame');
traj   = kinematicTrajectory('SampleRate', round(1/dt), ...
            'Position',[0 0 0], 'Velocity',[0 0 0], 'Orientation', q_init);
[pos_ned, orient, vel_ned] = traj(acc_u, rate_u);
p_true = pos_ned';
v_true = vel_ned';
q_true = compact(orient)';

fprintf('=== 미사일 HIL 검증 (OpenRocket truth) ===\n');
fprintf('  비행시간 %.2fs, N=%d, dt=%.3fms\n', t_end, N, dt*1e3);
fprintf('  최대고도 %.1fm\n\n', -min(-p_true(3,:)));

% ── 센서 측정 ────────────────────────────
rng(SEED);
gps_idx  = 1:round(S.dt_gps /dt):N;
baro_idx = 1:round(S.dt_baro/dt):N;

std_acc  = sqrt(S.var_acc  / dt);
std_gyro = sqrt(S.var_gyro / dt);
true_ba  = [0.05; -0.02; 0.01];
true_bg  = [0.001; -0.002; 0.003];

g_ned = [0;0;9.80665];
f_body_true = zeros(3,N);
for k = 1:N
    R_bn = NavUtils.quat2dcm(q_true(:,k));
    f_body_true(:,k) = acc_u(k,:)' - R_bn' * g_ned;
end
w_body_true = rate_u';

a_meas = f_body_true + true_ba + std_acc  * randn(3,N);
w_meas = w_body_true + true_bg + std_gyro * randn(3,N);

sig_p = sqrt([S.var_gps_pos_h; S.var_gps_pos_h; S.var_gps_pos_v]);
sig_v = sqrt([S.var_gps_vel_h; S.var_gps_vel_h; S.var_gps_vel_v]);
z_gps_p = p_true(:,gps_idx) + sig_p .* randn(3,length(gps_idx));
z_gps_v = v_true(:,gps_idx) + sig_v .* randn(3,length(gps_idx));
z_baro  = -p_true(3,baro_idx) + sqrt(S.var_baro) * randn(1,length(baro_idx));

hAcc = sqrt(S.var_gps_pos_h);
vAcc = sqrt(S.var_gps_pos_v);

% ── 초기 자세 (TRIAD) ───────────────────────────────
R_bn_true0     = NavUtils.quat2dcm(q_true(:,1));
f_static       = R_bn_true0' * (-g_ned);
m_static       = R_bn_true0' * (S.m_ref_ned / norm(S.m_ref_ned));
acc_meas_triad = f_static + true_ba + std_acc * randn(3,1);
mag_meas_triad = m_static + sqrt(S.var_mag) * randn(3,1);

q0_matlab = ESEKF.run_triad(acc_meas_triad, mag_meas_triad);
p0 = p_true(:,1) + [0.5;0.5;1.0].*randn(3,1);
v0 = v_true(:,1) + [0.1;0.1;0.2].*randn(3,1);
ekf_m = ESEKF(p0, v0, q0_matlab);

% ── MCU 연결 + reset + init ──────────────────────────
fprintf('MCU 연결 중 (%s @ %d)...\n', PORT, BAUD);
% 타임아웃을 10초에서 0.5초로 줄여서 멍때리는 현상 방지!
sp = serialport(PORT, BAUD, "Timeout", 0.5); 
flush(sp); pause(0.5); flush(sp);

[~,~,ok] = send_with_retry(sp, 0x01, [], 5);
if ~ok, error('MCU RESET 응답 없음. 보드 연결을 확인하세요.'); end

payload_init = typecast(single([p0;v0;q0_matlab]).','uint8');
[~,~,ok] = send_with_retry(sp, 0x02, payload_init, 5);
if ~ok, error('MCU INIT 응답 없음'); end

% ── TRIAD 비교 (seq=0 1회만 보냄, retry 없음) ────────
write(sp, [uint8([0xA5 0x03 0x00]), typecast(single([acc_meas_triad; mag_meas_triad; nan; nan]).','uint8')], "uint8");
[q0_mcu, us_triad, ok] = read_triad(sp, 0);
if ok
    fprintf('TRIAD diff (matlab vs mcu): %.3e   (mcu=%dµs)\n\n', norm(single(q0_matlab) - q0_mcu), us_triad);
end

% ── HIL 메인 루프 ────────────────────────────────────
fprintf('HIL 실행 중 (~%.0fs 예상)...\n', N*1.4e-3);

state_diff = zeros(16, N);
state_m    = zeros(16, N);   % MATLAB 측 풀 state
state_c    = zeros(16, N);   % ESP32 측 풀 state
p_mcu      = zeros(3, N);
us_predict = zeros(1, N);
us_gps     = nan(1, length(gps_idx));
us_baro    = nan(1, length(baro_idx));
us_total   = zeros(1, N);    % step 별 MCU 총 연산시간 (predict + GPS? + Baro?)

gps_ptr = 1; baro_ptr = 1;
tic; aborted = false; fail_op = '';

for k = 1:N
    % 1. Predict (자동 재전송 기능 탑재)
    ekf_m.predict(a_meas(:,k), w_meas(:,k), dt);
    payload = typecast(single([a_meas(:,k); w_meas(:,k); dt]).','uint8');
    [st, us_predict(k), ok] = send_with_retry(sp, 0x10, payload, 3);
    if ~ok, fail_op='predict'; aborted=true; break; end

    step_us = us_predict(k);

    % 2. GPS Update
    if gps_ptr <= length(gps_idx) && k == gps_idx(gps_ptr)
        ekf_m.update_gps([z_gps_p(:,gps_ptr); z_gps_v(:,gps_ptr)], hAcc, vAcc);
        payload_gps = typecast(single([z_gps_p(:,gps_ptr); z_gps_v(:,gps_ptr); hAcc; vAcc]).','uint8');
        [st, us_gps(gps_ptr), ok] = send_with_retry(sp, 0x20, payload_gps, 3);
        if ~ok, fail_op='gps'; aborted=true; break; end
        step_us = step_us + us_gps(gps_ptr);
        gps_ptr = gps_ptr + 1;
    end

    % 3. Baro Update
    if baro_ptr <= length(baro_idx) && k == baro_idx(baro_ptr)
        ekf_m.update_baro(z_baro(baro_ptr));
        payload_baro = typecast(single(z_baro(baro_ptr)),'uint8');
        [st, us_baro(baro_ptr), ok] = send_with_retry(sp, 0x21, payload_baro, 3);
        if ~ok, fail_op='baro'; aborted=true; break; end
        step_us = step_us + us_baro(baro_ptr);
        baro_ptr = baro_ptr + 1;
    end
    us_total(k) = step_us;

    % MATLAB vs MCU 비교
    m_st = single([ekf_m.nom.p; ekf_m.nom.v; ekf_m.nom.q; ekf_m.nom.b_a; ekf_m.nom.b_g]);
    state_diff(:,k) = double(m_st - st);
    state_m(:,k)    = double(m_st);
    state_c(:,k)    = double(st);
    p_mcu(:,k)      = double(st(1:3));

    if mod(k, 100) == 0
        elapsed = toc; eta = elapsed * (N - k) / k;
        avg_p = mean(us_predict(1:k));
        avg_t = mean(us_total(1:k));
        fprintf('  step %d/%d (%.1f%%) ETA=%.0fs  predict=%.0fµs  total/step=%.0fµs (%.1f%% of dt)\n', ...
                k, N, 100*k/N, eta, avg_p, avg_t, avg_t/(dt*1e6)*100);
    end
end

k_done = k;
if aborted
    fprintf('\n[중단] %d 스텝에서 %s 연산 통신 최종 실패!\n', k_done, fail_op);
else
    fprintf('\nHIL 종료 (실측 %.1fs, 완료 step=%d/%d)\n', toc, k_done, N);
end

% 완료된 부분 데이터 추출 및 결과 출력
sd  = state_diff(:, 1:k_done);
fprintf('\n[ACCURACY] max |MATLAB - MCU|  (n=%d steps)\n', k_done);
fprintf('  p  : %.3e m\n',     max(abs(sd(1:3,:)),  [], 'all'));
fprintf('  v  : %.3e m/s\n',   max(abs(sd(4:6,:)),  [], 'all'));
fprintf('  q  : %.3e\n',       max(abs(sd(7:10,:)), [], 'all'));

% ── 416Hz 제어 가능성 분석 ───────────────────────
ut = us_total(1:k_done);
ug = us_gps(~isnan(us_gps));
ub = us_baro(~isnan(us_baro));
imu_us = dt * 1e6;

fprintf('\n[MCU 연산시간 — IMU 1 step 당]\n');
fprintf('  predict()    mean/max = %5.0f / %5d µs\n', mean(us_predict(1:k_done)), max(us_predict(1:k_done)));
if ~isempty(ug)
    fprintf('  update_gps   mean/max = %5.0f / %5d µs   (n=%d)\n', mean(ug), max(ug), numel(ug));
end
if ~isempty(ub)
    fprintf('  update_baro  mean/max = %5.0f / %5d µs   (n=%d)\n', mean(ub), max(ub), numel(ub));
end
fprintf('  step 총합   mean/p99/max = %5.0f / %5.0f / %5d µs\n', ...
        mean(ut), prctile(ut, 99), max(ut));

fprintf('\n[416Hz 제어 가능성] IMU period = %.0fµs\n', imu_us);
fprintf('  평균 load = %.1f%%\n', mean(ut)/imu_us*100);
fprintf('  최대 load = %.1f%%  (step 중 최악의 경우)\n', max(ut)/imu_us*100);
n_over = sum(ut > imu_us);
fprintf('  주기 초과 step: %d / %d (%.3f%%)\n', n_over, k_done, n_over/k_done*100);
if max(ut) < imu_us * 0.7
    fprintf('  → 416Hz 제어 + ESEKF 동시 실행 충분히 여유로움 ✓\n');
elseif max(ut) < imu_us
    fprintf('  → 416Hz 가능하나 여유 적음 (제어 알고리즘 부담 고려 필요)\n');
else
    fprintf('  → 일부 step에서 주기 초과 — 제어주기 분리 또는 ESEKF 최적화 필요\n');
end

save('HIL_results.mat', 'state_diff', 'state_m', 'state_c', 'us_predict', 'us_gps', 'us_baro', 'us_total', ...
     't', 'p_true', 'v_true', 'q_true', 'true_ba', 'true_bg', 'p_mcu', 'q0_matlab', 'k_done');
fprintf('저장 완료: HIL_results.mat\n\n');

% ─────────────────────────────────────────────────────
% MATLAB vs ESP32-S3 컴포넌트별 비교 플롯
% ─────────────────────────────────────────────────────
tt   = t(1:k_done);
sm   = state_m(:, 1:k_done);
sc   = state_c(:, 1:k_done);
labels = {'N','E','D'};

% Position
figure('Name','HIL: Position','Position',[80 80 1200 350]);
for i = 1:3
    subplot(1,3,i); hold on; grid on;
    plot(tt, p_true(i,1:k_done), 'k--', 'LineWidth',1.0, 'DisplayName','Truth');
    plot(tt, sm(i,:), 'b', 'LineWidth',1.2, 'DisplayName','MATLAB');
    plot(tt, sc(i,:), 'r:', 'LineWidth',1.2, 'DisplayName','ESP32-S3');
    xlabel('Time [s]'); ylabel(sprintf('p_%s [m]',labels{i}));
    title(sprintf('Position %s', labels{i})); legend('Location','best');
end
sgtitle('Position (NED)');

% Velocity
figure('Name','HIL: Velocity','Position',[80 460 1200 350]);
for i = 1:3
    subplot(1,3,i); hold on; grid on;
    plot(tt, v_true(i,1:k_done), 'k--', 'LineWidth',1.0, 'DisplayName','Truth');
    plot(tt, sm(3+i,:), 'b', 'LineWidth',1.2, 'DisplayName','MATLAB');
    plot(tt, sc(3+i,:), 'r:', 'LineWidth',1.2, 'DisplayName','ESP32-S3');
    xlabel('Time [s]'); ylabel(sprintf('v_%s [m/s]',labels{i}));
    title(sprintf('Velocity %s', labels{i})); legend('Location','best');
end
sgtitle('Velocity (NED)');

% Quaternion
qlbl = {'w','x','y','z'};
figure('Name','HIL: Quaternion','Position',[80 80 1200 600]);
for i = 1:4
    subplot(2,2,i); hold on; grid on;
    plot(tt, q_true(i,1:k_done), 'k--', 'LineWidth',1.0, 'DisplayName','Truth');
    plot(tt, sm(6+i,:), 'b', 'LineWidth',1.2, 'DisplayName','MATLAB');
    plot(tt, sc(6+i,:), 'r:', 'LineWidth',1.2, 'DisplayName','ESP32-S3');
    xlabel('Time [s]'); ylabel(sprintf('q_%s',qlbl{i}));
    title(sprintf('Quaternion %s', qlbl{i})); legend('Location','best');
end
sgtitle('Quaternion');

% Bias (acc + gyro)
xyz = {'x','y','z'};
figure('Name','HIL: Bias','Position',[80 460 1200 600]);
for i = 1:3
    subplot(2,3,i); hold on; grid on;
    yline(true_ba(i), 'k--', 'LineWidth',1.0, 'DisplayName','Truth');
    plot(tt, sm(10+i-1,:), 'b', 'LineWidth',1.2, 'DisplayName','MATLAB');
    plot(tt, sc(10+i-1,:), 'r:', 'LineWidth',1.2, 'DisplayName','ESP32-S3');
    xlabel('Time [s]'); ylabel(sprintf('ba_%s [m/s²]',xyz{i}));
    title(sprintf('Accel bias %s', xyz{i})); legend('Location','best');
end
for i = 1:3
    subplot(2,3,3+i); hold on; grid on;
    yline(true_bg(i), 'k--', 'LineWidth',1.0, 'DisplayName','Truth');
    plot(tt, sm(13+i-1,:), 'b', 'LineWidth',1.2, 'DisplayName','MATLAB');
    plot(tt, sc(13+i-1,:), 'r:', 'LineWidth',1.2, 'DisplayName','ESP32-S3');
    xlabel('Time [s]'); ylabel(sprintf('bg_%s [rad/s]',xyz{i}));
    title(sprintf('Gyro bias %s', xyz{i})); legend('Location','best');
end
sgtitle('Bias (accel / gyro)');

% Diff (요약)
figure('Name','HIL: Diff','Position',[100 100 1100 350]);
subplot(1,3,1); semilogy(tt, vecnorm(state_diff(1:3, 1:k_done))); grid on;
xlabel('Time [s]'); ylabel('||\Deltap|| [m]'); title('Position diff');
subplot(1,3,2); semilogy(tt, vecnorm(state_diff(4:6, 1:k_done))); grid on;
xlabel('Time [s]'); ylabel('||\Deltav|| [m/s]'); title('Velocity diff');
subplot(1,3,3); semilogy(tt, vecnorm(state_diff(7:10,1:k_done))); grid on;
xlabel('Time [s]'); ylabel('||\Deltaq||'); title('Quaternion diff');
sgtitle('MATLAB vs ESP32-S3 차이 (norm, log scale)');

% ─────────────────────────────────────────────────────
% 헬퍼 함수: 재전송 로직 (실패하면 최대 max_retries 번 재시도)
% ─────────────────────────────────────────────────────
function [st, us, ok] = send_with_retry(sp, cmd_byte, payload, max_retries)
    % seq: 호출당 1씩 증가 (0~255 wrap). retry는 같은 seq 사용 → MCU 중복 감지
    persistent seq
    if isempty(seq), seq = uint8(0); end
    seq = uint8(mod(double(seq) + 1, 256));

    for r = 1:max_retries
        flush(sp);
        hdr = uint8([0xA5 cmd_byte seq]);
        if isempty(payload)
            write(sp, hdr, "uint8");
        else
            write(sp, [hdr, payload], "uint8");
        end

        [st, us, ok] = read_state(sp, seq);
        if ok, return; end

        if cmd_byte ~= 0x01 && cmd_byte ~= 0x02
            fprintf('  [패킷 유실] 재전송 (%d/%d)  seq=%d\n', r, max_retries, seq);
        end
        pause(0.05);
    end
end

function [st, us, ok] = read_state(s, expected_seq)
    % response: [0xA5][SEQ][64 byte state][4 byte us] = 70B
    % expected_seq 가 일치할 때만 valid → false sync(payload 내 0xA5) 차단
    st = zeros(16,1,'single'); us = 0; ok = false;
    t_start = tic;
    while true
        if toc(t_start) > 0.5, return; end
        b = read(s, 1, "uint8");
        if isempty(b) || b ~= 0xA5, continue; end
        sb = read(s, 1, "uint8");
        if isempty(sb), continue; end
        if sb == expected_seq, break; end
        % seq 불일치 → false sync, 계속 탐색
    end
    pl = uint8(read(s, 68, "uint8"));
    if length(pl) < 68, return; end
    st = typecast(pl(1:64), 'single').';
    us = double(typecast(pl(65:68), 'uint32'));
    ok = true;
end

function [q, us, ok] = read_triad(s, expected_seq)
    % response: [0xA5][SEQ][16 byte q][4 byte us] = 22B
    q = zeros(4,1,'single'); us = 0; ok = false;
    t_start = tic;
    while true
        if toc(t_start) > 0.5, return; end
        b = read(s, 1, "uint8");
        if isempty(b) || b ~= 0xA5, continue; end
        sb = read(s, 1, "uint8");
        if isempty(sb), continue; end
        if sb == expected_seq, break; end
    end
    pl = uint8(read(s, 20, "uint8"));
    if length(pl) < 20, return; end
    q  = typecast(pl(1:16),  'single').';
    us = double(typecast(pl(17:20), 'uint32'));
    ok = true;
end