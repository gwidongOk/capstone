% =========================================================
% 쿼터니언 적분 방법 비교 v3
% 시나리오: 정적 상태 (ω = 0) + 자이로 노이즈
% 참값: q_true = [1,0,0,0] 유지 (움직임 없음)
% 비교: 오일러 1차 vs RK4 선형보간
% =========================================================

clear; clc; close all;

% ─── 설정 ─────────────────────────────────────────────────
fs          = 416;       % IMU 샘플링 주파수 (Hz)
T           = 60.0;      % 총 시간 (초)
GYRO_VAR    = 1.18e-6;   % 자이로 분산 (rad/s)²
RANDOM_SEED = 42;

% ─── 시간축 ───────────────────────────────────────────────
t  = (0 : 1/fs : T)';
N  = length(t);
dt = 1 / fs;

% ─── 노이즈 생성 (참값 ω = 0, 노이즈만 존재) ─────────────
rng(RANDOM_SEED);
gyro_std = sqrt(GYRO_VAR);
wx = gyro_std * randn(N,1);
wy = gyro_std * randn(N,1);
wz = gyro_std * randn(N,1);

fprintf('자이로 std: %.4e rad/s  (%.4e deg/s)\n', gyro_std, rad2deg(gyro_std));

% ─── 참값: 항등 쿼터니언 고정 ────────────────────────────
q_true = repmat([1 0 0 0], N, 1);

% ─── 적분 ─────────────────────────────────────────────────
qdot = @(q,w) 0.5 * quat_mult(q, [0; w(1); w(2); w(3)]);

q0 = [1;0;0;0];
q_euler  = zeros(N,4);  q_euler(1,:)  = q0';
q_rk4lin = zeros(N,4);  q_rk4lin(1,:) = q0';

for i = 1:N-1
    w_cur  = [wx(i);   wy(i);   wz(i)  ];
    w_next = [wx(i+1); wy(i+1); wz(i+1)];
    w_mid  = 0.5 * (w_cur + w_next);

    % 오일러 1차
    q = q_euler(i,:)';
    q = q + dt * qdot(q, w_cur);
    q_euler(i+1,:) = (q / norm(q))';

    % RK4 선형보간
    q  = q_rk4lin(i,:)';
    k1 = qdot(q,            w_cur );
    k2 = qdot(q + dt/2*k1,  w_mid );
    k3 = qdot(q + dt/2*k2,  w_mid );
    k4 = qdot(q + dt  *k3,  w_next);
    q  = q + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    q_rk4lin(i+1,:) = (q / norm(q))';
end

% ─── 참값 기준 각도 오차 ──────────────────────────────────
err_euler  = zeros(N,1);
err_rk4lin = zeros(N,1);
for i = 1:N
    err_euler(i)  = angle_err(q_true(i,:)', q_euler(i,:)' );
    err_rk4lin(i) = angle_err(q_true(i,:)', q_rk4lin(i,:)');
end

% ─── 오일러각 변환 ────────────────────────────────────────
eul_euler  = rad2deg(quat2eul_zyx(q_euler));
eul_rk4lin = rad2deg(quat2eul_zyx(q_rk4lin));

% ─── 수치 요약 ────────────────────────────────────────────
fprintf('\n===== 수치 요약 (참값: 정적 상태) =====\n');
fprintf('%-18s  최대: %8.5f deg   RMS: %8.5f deg\n', 'Euler 1st',    max(err_euler),  rms(err_euler));
fprintf('%-18s  최대: %8.5f deg   RMS: %8.5f deg\n', 'RK4 선형보간', max(err_rk4lin), rms(err_rk4lin));

% =========================================================
% 시각화 — 하나의 Figure, 4 subplot
% =========================================================
figure('Name','정적 상태 자이로 노이즈 적분 비교', ...
       'Position',[100 80 1300 850]);

% ── subplot 1: Roll ──
subplot(4,1,1);
plot(t, eul_euler(:,1),  'b-', 'LineWidth',0.8); hold on;
plot(t, eul_rk4lin(:,1), 'r-', 'LineWidth',0.8);
yline(0, 'k--', 'LineWidth',1.0);
ylabel('Roll [deg]'); grid on;
legend('Euler 1st','RK4 선형보간','참값','Location','best');
title(sprintf('정적 상태 자이로 노이즈 적분 비교  |  분산=%.2e (rad/s)²  seed=%d', ...
      GYRO_VAR, RANDOM_SEED));

% ── subplot 2: Pitch ──
subplot(4,1,2);
plot(t, eul_euler(:,2),  'b-', 'LineWidth',0.8); hold on;
plot(t, eul_rk4lin(:,2), 'r-', 'LineWidth',0.8);
yline(0, 'k--', 'LineWidth',1.0);
ylabel('Pitch [deg]'); grid on;

% ── subplot 3: Yaw ──
subplot(4,1,3);
plot(t, eul_euler(:,3),  'b-', 'LineWidth',0.8); hold on;
plot(t, eul_rk4lin(:,3), 'r-', 'LineWidth',0.8);
yline(0, 'k--', 'LineWidth',1.0);
ylabel('Yaw [deg]'); grid on;

% ── subplot 4: 참값 기준 각도 오차 ──
subplot(4,1,4);
plot(t, err_euler,  'b-', 'LineWidth',0.8); hold on;
plot(t, err_rk4lin, 'r-', 'LineWidth',0.8);
ylabel('각도 오차 [deg]'); grid on;
legend('Euler 1st','RK4 선형보간','Location','best');
title('참값 기준 누적 각도 오차');
xlabel('Time [s]');

% =========================================================
% 보조 함수
% =========================================================
function qout = quat_mult(p, q)
    ps=p(1); px=p(2); py=p(3); pz=p(4);
    qs=q(1); qx=q(2); qy=q(3); qz=q(4);
    qout = [ps*qs - px*qx - py*qy - pz*qz;
            ps*qx + px*qs + py*qz - pz*qy;
            ps*qy - px*qz + py*qs + pz*qx;
            ps*qz + px*qy - py*qx + pz*qs];
end

function e = angle_err(q1, q2)
    dq = quat_mult([q1(1);-q1(2);-q1(3);-q1(4)], q2);
    e  = 2 * acosd(min(abs(dq(1)), 1.0));
end

function eul = quat2eul_zyx(Q)
    N = size(Q,1);  eul = zeros(N,3);
    for i = 1:N
        s=Q(i,1); x=Q(i,2); y=Q(i,3); z=Q(i,4);
        eul(i,1) = atan2(2*(s*x+y*z), 1-2*(x^2+y^2));
        eul(i,2) = asin(max(-1,min(1, 2*(s*y-z*x))));
        eul(i,3) = atan2(2*(s*z+x*y), 1-2*(y^2+z^2));
    end
end