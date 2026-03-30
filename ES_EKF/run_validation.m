%% run_validation.m  — ES-EKF 검증 스크립트
%  4가지 시나리오에 대해 EKF를 실행하고 참값 대비 오차 + 3σ 바운드를 시각화

clear; clc; close all;

scenarios = {'static', 'linear', 'accel', 'turn'};

for si = 1:length(scenarios)
    scenario = scenarios{si};
    fprintf('\n===== Scenario: %s =====\n', scenario);

    %% 1. 시뮬레이션 데이터 생성
    data = generate_sim_data(scenario);

    %% 2. EKF 초기화
    ekf = ESEKF(data.p_true(:,1), data.v_true(:,1), data.q_true(:,1));

    %% 3. 기록 배열
    N  = data.N;
    dt = data.dt;

    p_est = zeros(3, N);   v_est = zeros(3, N);   q_est = zeros(4, N);
    P_diag = zeros(15, N);

    p_est(:,1) = ekf.nom.p;
    v_est(:,1) = ekf.nom.v;
    q_est(:,1) = ekf.nom.q;
    P_diag(:,1) = diag(ekf.par.P);

    %% 4. 센서 인덱스 카운터
    gi = 1;  bi = 1;  mi = 1;

    %% 5. 메인 루프
    for k = 2:N
        % --- Predict ---
        ekf.predict(data.a_meas(:,k), data.w_meas(:,k), dt);

        % --- GPS Update ---
        if gi <= length(data.gps_idx) && k == data.gps_idx(gi)
            ekf.update_gps(data.z_gps(:,gi));
            gi = gi + 1;
        end

        % --- Baro Update ---
        if bi <= length(data.baro_idx) && k == data.baro_idx(bi)
            ekf.update_baro(data.z_baro(bi));
            bi = bi + 1;
        end

        % --- Mag Update ---
        if mi <= length(data.mag_idx) && k == data.mag_idx(mi)
            ekf.update_mag(data.z_mag(:,mi));
            mi = mi + 1;
        end

        % --- 기록 ---
        p_est(:,k)  = ekf.nom.p;
        v_est(:,k)  = ekf.nom.v;
        q_est(:,k)  = ekf.nom.q;
        P_diag(:,k) = diag(ekf.par.P);
    end

    %% 6. 오차 계산
    err_p = p_est - data.p_true;
    err_v = v_est - data.v_true;

    % 자세 오차 (오일러각 [deg])
    err_att = zeros(3, N);
    for k = 1:N
        q_err = NavUtils.quat_mult(NavUtils.quat_conj(data.q_true(:,k)), q_est(:,k));
        if q_err(1) < 0, q_err = -q_err; end
        err_att(:,k) = 2 * q_err(2:4) * (180/pi);  % 소각 근사
    end

    sigma3_p   = 3 * sqrt(P_diag(1:3, :));
    sigma3_v   = 3 * sqrt(P_diag(4:6, :));
    sigma3_att = 3 * sqrt(P_diag(7:9, :)) * (180/pi);

    t = data.t;

    %% 7. 플롯
    labels_p   = {'North [m]', 'East [m]', 'Down [m]'};
    labels_v   = {'vN [m/s]', 'vE [m/s]', 'vD [m/s]'};
    labels_att = {'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]'};

    figure('Name', sprintf('Position Error — %s', scenario), 'NumberTitle', 'off');
    for i = 1:3
        subplot(3,1,i);
        hold on;
        plot(t, err_p(i,:), 'b', 'LineWidth', 1);
        plot(t,  sigma3_p(i,:), 'r--', 'LineWidth', 0.8);
        plot(t, -sigma3_p(i,:), 'r--', 'LineWidth', 0.8);
        ylabel(labels_p{i}); grid on;
        if i == 1, title(sprintf('[%s] Position Error \\pm 3\\sigma', scenario)); end
        if i == 3, xlabel('Time [s]'); end
    end

    figure('Name', sprintf('Velocity Error — %s', scenario), 'NumberTitle', 'off');
    for i = 1:3
        subplot(3,1,i);
        hold on;
        plot(t, err_v(i,:), 'b', 'LineWidth', 1);
        plot(t,  sigma3_v(i,:), 'r--', 'LineWidth', 0.8);
        plot(t, -sigma3_v(i,:), 'r--', 'LineWidth', 0.8);
        ylabel(labels_v{i}); grid on;
        if i == 1, title(sprintf('[%s] Velocity Error \\pm 3\\sigma', scenario)); end
        if i == 3, xlabel('Time [s]'); end
    end

    figure('Name', sprintf('Attitude Error — %s', scenario), 'NumberTitle', 'off');
    for i = 1:3
        subplot(3,1,i);
        hold on;
        plot(t, err_att(i,:), 'b', 'LineWidth', 1);
        plot(t,  sigma3_att(i,:), 'r--', 'LineWidth', 0.8);
        plot(t, -sigma3_att(i,:), 'r--', 'LineWidth', 0.8);
        ylabel(labels_att{i}); grid on;
        if i == 1, title(sprintf('[%s] Attitude Error \\pm 3\\sigma', scenario)); end
        if i == 3, xlabel('Time [s]'); end
    end

    %% 8. 통계 출력
    fprintf('  Position RMSE: N=%.4f  E=%.4f  D=%.4f [m]\n', ...
        rms(err_p(1,:)), rms(err_p(2,:)), rms(err_p(3,:)));
    fprintf('  Velocity RMSE: N=%.4f  E=%.4f  D=%.4f [m/s]\n', ...
        rms(err_v(1,:)), rms(err_v(2,:)), rms(err_v(3,:)));
    fprintf('  Attitude RMSE: R=%.4f  P=%.4f  Y=%.4f [deg]\n', ...
        rms(err_att(1,:)), rms(err_att(2,:)), rms(err_att(3,:)));

    % 3σ 바운드 내 비율 (consistency check)
    in_bound_p = mean(all(abs(err_p) <= sigma3_p, 1)) * 100;
    in_bound_v = mean(all(abs(err_v) <= sigma3_v, 1)) * 100;
    in_bound_a = mean(all(abs(err_att) <= sigma3_att, 1)) * 100;
    fprintf('  3σ bound coverage: pos=%.1f%%  vel=%.1f%%  att=%.1f%%\n', ...
        in_bound_p, in_bound_v, in_bound_a);
end

fprintf('\nDone.\n');
