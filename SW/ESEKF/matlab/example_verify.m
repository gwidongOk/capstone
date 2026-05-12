% example_verify.m
% MATLAB ESEKF vs MCU ESEKF — 같은 입력으로 시간 + 정확도 비교
% serialport 직접 호출, 별도 helper 없음

clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', 'ES_EKF'));

PORT = "COM5";
DT   = 1/416;
N    = 500;

s = serialport(PORT, 921600, "Timeout", 2);
flush(s);

% ── 초기 상태 ──
p0 = single([0;0;0]);
v0 = single([0;0;0]);
q0 = single([1;0;0;0]);

% MCU: reset + init
write(s, uint8([0xA5 0x01]), "uint8"); read_state(s);
write(s, uint8([0xA5 0x02]), "uint8");
write(s, typecast([p0; v0; q0].', 'uint8'), "uint8");
read_state(s);

% MATLAB ESEKF
ekf_m = ESEKF([0;0;0], [0;0;0], [1;0;0;0]);

% ── 시나리오 (정지 IMU) ──
a_seq = single(repmat([0; 0; -9.81], 1, N));
w_seq = single(zeros(3, N));

err = zeros(16, N);
us  = zeros(1, N);

for k = 1:N
    % MATLAB predict
    ekf_m.predict(a_seq(:,k), w_seq(:,k), DT);

    % MCU predict — sync + cmd + payload(28B)
    write(s, uint8([0xA5 0x10]), "uint8");
    write(s, typecast([a_seq(:,k); w_seq(:,k); single(DT)].', 'uint8'), "uint8");
    [st, us(k)] = read_state(s);

    m_st = single([ekf_m.nom.p; ekf_m.nom.v; ekf_m.nom.q; ekf_m.nom.b_a; ekf_m.nom.b_g]);
    err(:,k) = double(m_st - st);
end

% ── 결과 ──
fprintf('\n[CPU TIME] predict() on ESP32-S3 (n=%d)\n', N);
fprintf('  min/mean/max : %d / %.0f / %d µs\n', min(us), mean(us), max(us));
fprintf('  IMU period %.2fms 대비: %.1f%%\n', DT*1e3, mean(us)/(DT*1e6)*100);

fprintf('\n[ACCURACY] max |MATLAB - MCU|\n');
fprintf('  p  : %.3e m\n',     max(abs(err(1:3,:)),  [], 'all'));
fprintf('  v  : %.3e m/s\n',   max(abs(err(4:6,:)),  [], 'all'));
fprintf('  q  : %.3e\n',       max(abs(err(7:10,:)), [], 'all'));
fprintf('  ba : %.3e m/s²\n',  max(abs(err(11:13,:)),[], 'all'));
fprintf('  bg : %.3e rad/s\n', max(abs(err(14:16,:)),[], 'all'));

figure;
subplot(1,2,1); plot(us); xlabel('step'); ylabel('µs');
title(sprintf('predict() time (mean=%.0fµs)', mean(us))); grid on;
subplot(1,2,2);
semilogy(vecnorm(err(1:3,:)),'r'); hold on;
semilogy(vecnorm(err(4:6,:)),'g'); semilogy(vecnorm(err(7:10,:)),'b');
xlabel('step'); ylabel('||err||'); legend('p','v','q'); grid on;
title('MATLAB vs MCU diff');

% ── 헬퍼 ──
function [st, us] = read_state(s)
    while read(s, 1, "uint8") ~= 0xA5, end          % sync seek
    pl = uint8(read(s, 68, "uint8"));
    st = typecast(pl(1:64), 'single').';            % p v q ba bg (16x1 single)
    us = double(typecast(pl(65:68), 'uint32'));
end
