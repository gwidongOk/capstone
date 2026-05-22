classdef ESEKF < handle
%ESEKF  Error-State Extended Kalman Filter (ES-EKF)
%   nom: nominal state struct (p, v, q, b_a, b_g)
%   par: covariance and noise parameter struct
%   cfg: constants from config.m

    properties

        cfg
        nom
        par
        g     (3,1) double
        last_accel_mag (1,1) double = 0

        last_a_m = []
        last_w_m = []
        ema_a = zeros(3,1);
        ema_w = zeros(3,1);
        current_jerk_scale_a (1,1) double = 1.0;
    end

    methods (Access = public)

        function obj = ESEKF(p0, v0, q0, cfg)
        % Constructor
            if nargin < 4 || isempty(cfg)
                obj.cfg = config();
            else
                obj.cfg = cfg;
            end
            S = obj.cfg;

            obj.nom = struct();
            obj.nom.p   = p0;
            obj.nom.v   = v0;
            obj.nom.q   = q0;
            obj.nom.b_a = zeros(3,1);
            obj.nom.b_g = zeros(3,1);

            obj.g = S.g_ned;

            obj.par = struct();
            obj.par.P = zeros(15);
            obj.par.R_gps = zeros(6);
            obj.par.R_baro = 0;
            obj.par.R_mag = zeros(3);

            P0_pos = S.P0_pos;
            P0_vel = S.P0_vel;
            P0_att = S.P0_att;
            P0_ba  = S.P0_ba;
            P0_bg  = S.P0_bg;
            obj.par.P = diag([P0_pos^2 * ones(1,3), ...
                              P0_vel^2 * ones(1,3), ...
                              P0_att^2 * ones(1,3), ...
                              P0_ba^2  * ones(1,3), ...
                              P0_bg^2  * ones(1,3)]);

            obj.par.var_acc  = obj.cfg.Q_ACC_BASE_SCALE  * S.var_acc;
            obj.par.var_gyro = obj.cfg.Q_GYRO_BASE_SCALE * S.var_gyro;
            obj.par.var_ba   = S.var_ba;
            obj.par.var_bg   = S.var_bg;

            obj.par.R_gps  = diag([obj.cfg.GPS_POS_R_SCALE * S.var_gps_pos_h, ...
                                   obj.cfg.GPS_POS_R_SCALE * S.var_gps_pos_h, ...
                                   obj.cfg.GPS_POS_R_SCALE * S.var_gps_pos_v, ...
                                   obj.cfg.GPS_VEL_R_SCALE * S.var_gps_vel_h, ...
                                   obj.cfg.GPS_VEL_R_SCALE * S.var_gps_vel_h, ...
                                   obj.cfg.GPS_VEL_R_SCALE * S.var_gps_vel_v]);
            obj.par.R_baro = obj.cfg.BARO_R_SCALE * S.var_baro;
        end

        function predict(obj, a_m, w_m, dt)
        % Fixed-Q propagation
            a_hat = a_m - obj.nom.b_a;
            w_hat = w_m - obj.nom.b_g;

            obj.last_accel_mag = norm(a_hat);

            R_nb  = ESEKF.quat2dcm(obj.nom.q);
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
            obj.nom.q = ESEKF.quat_mult(obj.nom.q, dq);
            obj.nom.q = obj.nom.q / norm(obj.nom.q);

            Fc = zeros(15);
            Fc(1:3,  4:6)  = eye(3);
            Fc(4:6,  7:9)  = -R_nb * ESEKF.skew(a_hat);
            Fc(4:6,  10:12)= -R_nb;
            Fc(7:9,  7:9)  = -ESEKF.skew(w_hat);
            Fc(7:9,  13:15)= -eye(3);

            Fdt       = Fc * dt;
            F       = eye(15) + Fdt + Fdt^2/2 + Fdt^3/6;

            Qd = zeros(15);
            Qd(4:6,   4:6)   = obj.par.var_acc  * dt^2 * eye(3);
            Qd(7:9,   7:9)   = obj.par.var_gyro * dt^2 * eye(3);
            Qd(10:12, 10:12) = obj.par.var_ba * dt * eye(3);
            Qd(13:15, 13:15) = obj.par.var_bg * dt * eye(3);

            obj.par.P = F * obj.par.P * F' + Qd;
            obj.par.P = 0.5*(obj.par.P + obj.par.P');
        end

        function predict_adaptive_jerk(obj, a_m, w_m, dt)

            a_hat = a_m - obj.nom.b_a;
            w_hat = w_m - obj.nom.b_g;
            obj.last_accel_mag = norm(a_hat);
            
            if isempty(obj.last_a_m)
                obj.ema_a = a_m;
                obj.ema_w = w_m;
                obj.last_a_m = a_m;
                obj.last_w_m = w_m;
            end
            
            obj.ema_a = obj.cfg.JERK_ALPHA * a_m + (1 - obj.cfg.JERK_ALPHA) * obj.ema_a;
            obj.ema_w = obj.cfg.JERK_ALPHA * w_m + (1 - obj.cfg.JERK_ALPHA) * obj.ema_w;

            delta_a = norm(obj.ema_a - obj.last_a_m) / dt; 
            delta_w = norm(obj.ema_w - obj.last_w_m) / dt;
            
            obj.last_a_m = obj.ema_a;
            obj.last_w_m = obj.ema_w;
            
            qs_a = 1.0;
            qs_w = 1.0;
            
            if delta_a > obj.cfg.JERK_THRESH
                qs_a = (delta_a / obj.cfg.JERK_THRESH)^2;
                qs_a = min(qs_a, obj.cfg.JERK_SCALE_MAX);
            end
            
            if delta_w > obj.cfg.ANG_ACC_THRESH
                qs_w = (delta_w / obj.cfg.ANG_ACC_THRESH)^2;
                qs_w = min(qs_w, obj.cfg.JERK_SCALE_MAX);
            end

            R_nb  = ESEKF.quat2dcm(obj.nom.q);
            a_ned = R_nb * a_hat + obj.g;
            
            obj.nom.p = obj.nom.p + obj.nom.v*dt + 0.5*a_ned*dt^2;
            obj.nom.v = obj.nom.v + a_ned*dt;
            
            theta = w_hat * dt; 
            th_n = norm(theta);
            if th_n > 1e-10
                dq = [cos(th_n/2); sin(th_n/2)/th_n * theta];
            else
                dq = [1; theta/2];
            end
            obj.nom.q = ESEKF.quat_mult(obj.nom.q, dq);
            obj.nom.q = obj.nom.q / norm(obj.nom.q);
            
            Fc = zeros(15);
            Fc(1:3,  4:6)   = eye(3);
            Fc(4:6,  7:9)   = -R_nb * ESEKF.skew(a_hat);
            Fc(4:6,  10:12) = -R_nb;
            Fc(7:9,  7:9)   = -ESEKF.skew(w_hat);
            Fc(7:9,  13:15) = -eye(3);
            
            Fdt = Fc * dt;
            F   = eye(15) + Fdt + Fdt^2/2 + Fdt^3/6;
            
            Qd = zeros(15);
            Qd(4:6,   4:6)   = obj.par.var_acc  * qs_a * dt^2 * eye(3);
            Qd(7:9,   7:9)   = obj.par.var_gyro * qs_w * dt^2 * eye(3);
            Qd(10:12, 10:12) = obj.par.var_ba * dt * eye(3);
            Qd(13:15, 13:15) = obj.par.var_bg * dt * eye(3);
            
            obj.par.P = F * obj.par.P * F' + Qd;
            obj.par.P = 0.5*(obj.par.P + obj.par.P');
            obj.current_jerk_scale_a = qs_a;
        end

        function update_gps(obj, z, hAcc, vAcc)
            H = zeros(6,15);
            H(1:3,1:3) = eye(3);
            H(4:6,4:6) = eye(3);
            y = z - [obj.nom.p; obj.nom.v];
            
            G4_LIMIT = 4.0 * 9.80665;
            high_g_penalty = 1.0;
            if obj.last_accel_mag > G4_LIMIT
                excess = obj.last_accel_mag - G4_LIMIT;
                high_g_penalty = 1.0 + (excess * excess * 10.0);
                high_g_penalty = min(high_g_penalty, 30.0);
            end
            
            if nargin >= 4 && ~isempty(hAcc) && ~isempty(vAcc)

                pos_multiplier = obj.cfg.GPS_ACC_INFLATION * high_g_penalty;
                var_h  = obj.cfg.GPS_POS_R_SCALE * (hAcc * pos_multiplier)^2;
                var_v  = obj.cfg.GPS_POS_R_SCALE * (vAcc * pos_multiplier)^2;
                var_vh = obj.par.R_gps(4,4);
                var_vv = obj.par.R_gps(6,6);
                R = diag([var_h, var_h, var_v, var_vh, var_vh, var_vv]);
            else

                R = obj.par.R_gps * high_g_penalty;
            end
            
            obj.measurement_update(H, y, R);
        end

        function update_baro(obj, z)

            H      = zeros(1,15);
            H(1,3) = -1;

            y = z - (-obj.nom.p(3));

            obj.measurement_update(H, y, obj.par.R_baro);
        end

        function update_zupt(obj)

            z_vel = [0; 0; 0];
            
            v_pred = obj.nom.v;
            
            y = z_vel - v_pred;
            
            H = zeros(3, 15);
            H(1:3, 4:6) = eye(3);
            
            R_zupt = eye(3) * 1e-4; 
            
            obj.measurement_update(H, y, R_zupt);
        end

        function update_acc_static(obj, a_m)

            R_nb = ESEKF.quat2dcm(obj.nom.q);

            z_pred = R_nb' * (-obj.g) + obj.nom.b_a;
            y = a_m(:) - z_pred;
            
            H = zeros(3, 15);
            H(:, 7:9)   = ESEKF.skew(R_nb' * (-obj.g));
            H(:, 10:12) = eye(3);
            
            S = obj.cfg;
            R_acc = eye(3) * S.var_acc; 
            
            obj.measurement_update(H, y, R_acc);
        end

        function update_gyro_static(obj, w_m)

            z_pred = obj.nom.b_g;
            y = w_m(:) - z_pred;
            
            H = zeros(3, 15);
            H(:, 13:15) = eye(3);
            
            S = obj.cfg;
            R_gyro = eye(3) * S.var_gyro;
            
            obj.measurement_update(H, y, R_gyro);
        end

        function update_mag(obj, z_m)

            if norm(z_m) < 1e-4, return; end
            z_m = z_m(:) / norm(z_m);
            
            R_nb = ESEKF.quat2dcm(obj.nom.q);
            S = obj.cfg;
            m_ref = S.m_ref_ned / norm(S.m_ref_ned);
            z_pred = R_nb' * m_ref;
            
            y = z_m - z_pred;
            
            H = zeros(3, 15);
            H(:, 7:9) = ESEKF.skew(z_pred);
            
            R_mag = eye(3) * S.var_mag;
            
            obj.measurement_update(H, y, R_mag);
        end

        function [t_spent, converged] = run_zupt_alignment(obj, imu_provider, dt, threshold, max_time, mag_provider)
            arguments
                obj
                imu_provider
                dt (1,1) double
                threshold (1,1) double = 1e-4
                max_time (1,1) double = 30.0
                mag_provider = []
            end

            t = 0;
            converged = false;
            steps_per_update = round(0.1 / dt);

            while t < max_time

                for i = 1:steps_per_update
                    [a_m, w_m] = imu_provider();
                    obj.predict(a_m, w_m, dt);
                    t = t + dt;
                end
                
                P_before = sum(diag(obj.par.P(7:15, 7:15)));
                
                obj.update_zupt();
                obj.update_acc_static(a_m);
                obj.update_gyro_static(w_m);
                
                if ~isempty(mag_provider)
                    z_m = mag_provider();
                    obj.update_mag(z_m);
                end
                
                P_after = sum(diag(obj.par.P(7:15, 7:15)));
                
                if P_before > 0 && abs(P_before - P_after) / P_before < threshold
                    converged = true;
                    break;
                end
            end
            t_spent = t;
        end

    end % methods (Access = public)

    methods (Static)

        function q = run_triad(acc, mag, lat, lon)

            acc = acc(:);
            mag = mag(:);
            
            r1 = [0; 0; 1]; % Down
            
            if nargin >= 4 && ~isempty(lat) && ~isempty(lon)
                [D_deg, I_deg] = ESEKF.wmm_korea(lat, lon);
                cD = cosd(D_deg); sD = sind(D_deg);
                cI = cosd(I_deg); sI = sind(I_deg);
                r2 = [cI*cD; cI*sD; sI];
            else
                S  = config;
                r2 = S.m_ref_ned / norm(S.m_ref_ned);
            end

            b1 = -acc / norm(acc);
            b2 =  mag / norm(mag);

            if abs(dot(b1, b2)) > 0.999
                warning('ESEKF.run_triad: acc and mag vectors are nearly parallel.');
            end

            u1 = r1;
            u2 = cross(u1, r2) / norm(cross(u1, r2));
            u3 = cross(u1, u2);
            M_ref = [u1, u2, u3];

            v1 = b1;
            v2 = cross(v1, b2) / norm(cross(v1, b2));
            v3 = cross(v1, v2);
            M_body = [v1, v2, v3];

            % 4. R_b2n = M_ref * M_body'
            C_bn = M_ref * M_body';
            q    = ESEKF.dcm2quat(C_bn);
        end

        function R = quat2dcm(q)
            q = q / norm(q);
            q0=q(1); q1=q(2); q2=q(3); q3=q(4);
            R = [1-2*(q2^2+q3^2),  2*(q1*q2-q0*q3),  2*(q1*q3+q0*q2);
                 2*(q1*q2+q0*q3),  1-2*(q1^2+q3^2),  2*(q2*q3-q0*q1);
                 2*(q1*q3-q0*q2),  2*(q2*q3+q0*q1),  1-2*(q1^2+q2^2)];
        end

        function q = dcm2quat(R)
            tr = trace(R);
            [~, idx] = max([tr, R(1,1), R(2,2), R(3,3)]);
            switch idx
                case 1
                    q0 = 0.5*sqrt(1+tr); k = 0.25/q0;
                    q1 = k*(R(3,2)-R(2,3)); q2 = k*(R(1,3)-R(3,1));
                    q3 = k*(R(2,1)-R(1,2));
                case 2
                    q1 = 0.5*sqrt(1+2*R(1,1)-tr); k = 0.25/q1;
                    q0 = k*(R(3,2)-R(2,3)); q2 = k*(R(1,2)+R(2,1));
                    q3 = k*(R(1,3)+R(3,1));
                case 3
                    q2 = 0.5*sqrt(1+2*R(2,2)-tr); k = 0.25/q2;
                    q0 = k*(R(1,3)-R(3,1)); q1 = k*(R(1,2)+R(2,1));
                    q3 = k*(R(2,3)+R(3,2));
                otherwise
                    q3 = 0.5*sqrt(1+2*R(3,3)-tr); k = 0.25/q3;
                    q0 = k*(R(2,1)-R(1,2)); q1 = k*(R(1,3)+R(3,1));
                    q2 = k*(R(2,3)+R(3,2));
            end
            q = [q0; q1; q2; q3];
            if q(1) < 0, q = -q; end
            q = q / norm(q);
        end

        function q_out = quat_mult(p, r)
            p0=p(1); p1=p(2); p2=p(3); p3=p(4);
            Q = [p0,-p1,-p2,-p3;
                 p1, p0,-p3, p2;
                 p2, p3, p0,-p1;
                 p3,-p2, p1, p0];
            q_out = Q * r;
        end

        function S = skew(v)
            S = [ 0, -v(3),  v(2);
                  v(3), 0, -v(1);
                 -v(2), v(1), 0 ];
        end

        function qc = quat_conj(q)
            qc = [q(1); -q(2); -q(3); -q(4)];
        end

        function euler = quat2euler(q)
            q = q / norm(q);
            q0=q(1); q1=q(2); q2=q(3); q3=q(4);
            roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
            sinp = 2*(q0*q2 - q3*q1);
            if abs(sinp) >= 1
                pitch = sign(sinp) * pi/2;
            else
                pitch = asin(sinp);
            end
            yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
            euler = [roll; pitch; yaw];
        end
    end

    methods (Access = private)

        function measurement_update(obj, H, y, R)

            S  = H * obj.par.P * H' + R;
            K  = obj.par.P * H' / S;
            dx = K * y;

            IKH       = eye(15) - K*H;
            obj.par.P = IKH * obj.par.P * IKH' + K * R * K';
            obj.par.P = 0.5*(obj.par.P + obj.par.P');

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
            obj.nom.q = ESEKF.quat_mult(obj.nom.q, dq/norm(dq));
            obj.nom.q = obj.nom.q / norm(obj.nom.q);

            G          = eye(15);
            G(7:9,7:9) = eye(3) - ESEKF.skew(dth/2);
            obj.par.P  = G * obj.par.P * G';
            obj.par.P  = 0.5*(obj.par.P + obj.par.P');
        end

    end % methods (Access = private)
    
    methods (Static, Access = private)
        function [D, I] = wmm_korea(lat, lon)

            if lat < 32 || lat > 40 || lon < 124 || lon > 132
                warning('ESEKF.wmm_korea: location is outside Korea approximation bounds.');
            end
            D = -7.9 - 0.35*(lat - 36) + 0.15*(lon - 127);
            I = 51.0 + 1.40*(lat - 36) + 0.05*(lon - 127);
        end
    end

end % classdef
