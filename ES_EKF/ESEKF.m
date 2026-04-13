classdef ESEKF < handle
%ESEKF  Error-State Extended Kalman Filter (ES-EKF)
%
% 프로퍼티:
%   nom   NominalState  — 명목 상태 (p, v, q, b_a, b_g)
%   par   FilterParams  — 필터 행렬 (P, Q, R_gps, R_baro, R_mag)
%   g     [3x1]         — 중력벡터 NED [m/s²]
%   m_ref [3x1]         — 지자기 기준벡터 NED (단위벡터)
%
% 사용법:
%   ekf = ESEKF(p0, v0, q0);
%   ekf.predict(a_m, w_m, dt);
%   ekf.update_gps(z_gps);
%   ekf.update_baro(z_baro);
%   ekf.update_mag(z_mag);

    properties
        nom   NominalState     % 명목 상태 x
        par   FilterParams     % 필터 행렬 (P, Q, R)
        g     (3,1) double     % 중력벡터 NED [m/s²]
        m_ref (3,1) double     % 지자기 기준벡터 NED [3x1]
    end

    methods (Access = public)

        %% ── 생성자 ───────────────────────────────────────────────────────
        function obj = ESEKF(p0, v0, q0)
        %   p0 [3x1] 초기 위치 NED [m]
        %   v0 [3x1] 초기 속도 NED [m/s]
        %   q0 [4x1] 초기 쿼터니언 [q0;q1;q2;q3]

            % ── 명목 상태 ─────────────────────────────────────────────
            obj.nom = NominalState();
            obj.nom.p = p0;
            obj.nom.v = v0;
            obj.nom.q = q0;

            % ── 물리 상수 (SensorSpec 참조) ──────────────────────────
            S = SensorSpec;
            obj.g     = S.g_ned;
            obj.m_ref = S.m_ref_ned / norm(S.m_ref_ned);

            % ── 필터 행렬 ─────────────────────────────────────────────
            obj.par = FilterParams();

            % P0 — 초기 오차 공분산
            P0_pos = 3;        % [m]
            P0_vel = 1;        % [m/s]
            P0_att = 0.087;    % [rad] ≈ 5°
            P0_ba  = 0.5;      % [m/s²]
            P0_bg  = 0.01;     % [rad/s]
            obj.par.P = diag([P0_pos^2 * ones(1,3), ...
                              P0_vel^2 * ones(1,3), ...
                              P0_att^2 * ones(1,3), ...
                              P0_ba^2  * ones(1,3), ...
                              P0_bg^2  * ones(1,3)]);

            % Q — 프로세스 노이즈 (연속시간 파라미터 저장, predict에서 이산화)
            % var_acc/var_gyro: 샘플 분산 [단위²·s⁻¹]
            % var_ba/var_bg:    랜덤워크 분산/dt [단위²·s⁻¹]
            obj.par.var_acc  = S.var_acc;
            obj.par.var_gyro = S.var_gyro;
            obj.par.var_ba   = S.var_ba;
            obj.par.var_bg   = S.var_bg;

            % R — 측정 노이즈
            obj.par.R_gps  = diag([S.sigma_gps_pos_h^2, S.sigma_gps_pos_h^2, ...
                                   S.sigma_gps_pos_v^2, ...
                                   S.sigma_gps_vel_h^2, S.sigma_gps_vel_h^2, ...
                                   S.sigma_gps_vel_v^2]);
            obj.par.R_baro = S.var_baro;
            obj.par.R_mag  = S.var_mag * eye(3);
            obj.par.R_mag_heading = S.var_mag_heading;
        end

        %% ── predict ──────────────────────────────────────────────────────
        function predict(obj, a_m, w_m, dt)
        %   a_m [3x1] 가속도계 측정값 [m/s²]
        %   w_m [3x1] 자이로 측정값 [rad/s]
        %   dt  [1x1] IMU 샘플 간격 [s]

            a_hat = a_m - obj.nom.b_a;
            w_hat = w_m - obj.nom.b_g;

            % 명목 상태 전파
            R_nb  = NavUtils.quat2dcm(obj.nom.q);
            a_ned = R_nb * a_hat + obj.g;

            obj.nom.p = obj.nom.p + obj.nom.v*dt + 0.5*a_ned*dt^2;
            obj.nom.v = obj.nom.v + a_ned*dt;

            theta = w_hat * dt;
            th_n  = norm(theta);
            if th_n > 1e-10
                dq = [cos(th_n/2); sin(th_n/2)/th_n * theta];
            else
                dq = [1; theta/2];
            end
            obj.nom.q = NavUtils.quat_mult(obj.nom.q, dq);
            obj.nom.q = obj.nom.q / norm(obj.nom.q);

            % 오차 공분산 전파
            Fc = zeros(15);
            Fc(1:3,  4:6)  = eye(3);
            Fc(4:6,  7:9)  = -R_nb * NavUtils.skew(a_hat);
            Fc(4:6,  10:12)= -R_nb;
            Fc(7:9,  7:9)  = -NavUtils.skew(w_hat);
            Fc(7:9,  13:15)= -eye(3);
            %Fc = 연속시간 시스템 행렬 (미분 방정식의 계수) F  = e^(Fc·dt) = 이산시간 상태천이행렬
            Fdt       = Fc * dt;
            F       = eye(15) + Fdt + Fdt^2/2 + Fdt^3/6; % 테일러 급수 3차

            % 이산 프로세스 노이즈: Qd = G·Qc·G'·dt
            %   속도 ← n_a·dt  →  Qd(v) = var_acc · dt²
            %   자세 ← n_g·dt  →  Qd(θ) = var_gyro · dt²
            %   바이어스 ← 랜덤워크 →  Qd(b) = var_b  (이미 per-step)
            Qd = zeros(15);
            Qd(4:6,   4:6)   = obj.par.var_acc  * dt * eye(3);
            Qd(7:9,   7:9)   = obj.par.var_gyro * dt * eye(3);
            Qd(10:12, 10:12) = obj.par.var_ba * dt * eye(3);
            Qd(13:15, 13:15) = obj.par.var_bg * dt * eye(3);

            obj.par.P = F * obj.par.P * F' + Qd;
            obj.par.P = 0.5*(obj.par.P + obj.par.P');
        end

        %% ── update_gps ───────────────────────────────────────────────────
        function update_gps(obj, z)
        %   z [6x1] = [pN;pE;pD;vN;vE;vD]
            H = zeros(6,15);
            H(1:3,1:3) = eye(3);
            H(4:6,4:6) = eye(3);

            y = z - [obj.nom.p; obj.nom.v];
            obj.measurement_update(H, y, obj.par.R_gps);
        end

        %% ── update_baro ──────────────────────────────────────────────────
        function update_baro(obj, z)
        %   z [scalar] 고도 = -pD [m]

            H      = zeros(1,15);
            H(1,3) = -1;

            y = z - (-obj.nom.p(3));

            obj.measurement_update(H, y, obj.par.R_baro);
        end
        function update_mag(obj, z)
            z_n = z / norm(z);
            R_nb = NavUtils.quat2dcm(obj.nom.q);
            m_pred = R_nb' * obj.m_ref;
            H = zeros(3,15);
            H(1:3,7:9) = NavUtils.skew(m_pred);
            y = z_n - m_pred;
            obj.measurement_update(H, y, obj.par.R_mag);
        end
    end % methods

    %% ── 내부 공통 함수 ──────────────────────────────────────────────────
    methods (Access = private)

        function measurement_update(obj, H, y, R)

            S  = H * obj.par.P * H' + R;
            K  = obj.par.P * H' / S;
            dx = K * y;

            % P 업데이트 (Joseph form)
            IKH       = eye(15) - K*H;
            obj.par.P = IKH * obj.par.P * IKH' + K * R * K';
            obj.par.P = 0.5*(obj.par.P + obj.par.P');

            % 오차 dx를 명목 상태에 주입
            obj.nom.p   = obj.nom.p   + dx(1:3);
            obj.nom.v   = obj.nom.v   + dx(4:6);
            obj.nom.b_a = obj.nom.b_a + dx(10:12);
            obj.nom.b_g = obj.nom.b_g + dx(13:15);

            dth   = dx(7:9);
            dth_n = norm(dth);
            if dth_n > 1e-10
                dq = [cos(dth_n/2); sin(dth_n/2)/dth_n * dth];
            else
                dq = [1; dth/2];
            end
            obj.nom.q = NavUtils.quat_mult(obj.nom.q, dq/norm(dq));
            obj.nom.q = obj.nom.q / norm(obj.nom.q);

            % 리셋 야코비안
            %른 상태변수는 NED 기준좌표계 값이기에 오차공분산을 보정할 필요가 없지만 
            % 자세는 body축과 ned의 관계이기에 P의 좌표계를 보정된 새 자세로 바꿀 필요 있음.
            % δθ가 관여하는 행 전체와 열 전체가 변화함
            G          = eye(15);
            G(7:9,7:9) = eye(3) - NavUtils.skew(dth/2);
            obj.par.P  = G * obj.par.P * G';
            obj.par.P  = 0.5*(obj.par.P + obj.par.P');
        end

    end % methods (Access = private)

end % classdef
