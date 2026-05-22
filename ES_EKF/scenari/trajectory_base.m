function truth = trajectory_base(case_name)
%TRAJECTORY_BASE  Truth trajectory generator.

    if nargin < 1 || isempty(case_name)
        case_name = 'nominal';
    end

    S  = config;
    dt = S.dt_imu;
    g  = norm(S.g_ned);

    T_end         = 20.0;
    burn_time     = 2.2;
    f_axial_boost = 7.1 * g;

    enable_boost_vibration = true;
    vibration_std_lateral  = 0.8;
    vibration_std_axial    = 1.2;

    enable_canard = true;
    canard_t_start = 6.0;
    canard_t_end   = 6.2;
    canard_lat_g   = 8.0 * g;
    canard_buffet_std      = 1.2 * g;
    canard_gyro_buffet_std = deg2rad(35);

    enable_ejection_shock = true;

    switch lower(case_name)
        case 'nominal'
        case 'smooth'
            vibration_std_lateral = 0.2;
            vibration_std_axial   = 0.3;
            canard_buffet_std = 0.0;
            canard_gyro_buffet_std = 0.0;
        case 'strong_canard'
            canard_t_end = 6.15;
            canard_lat_g = 10.0 * g;
            canard_buffet_std = 1.8 * g;
            canard_gyro_buffet_std = deg2rad(50);
        case 'no_canard'
            enable_canard = false;
        otherwise
            error('trajectory_base:UnknownCase', 'Unknown truth case: %s', case_name);
    end

    t_full = 0 : dt : T_end;
    N = length(t_full);

    vib_lateral = vibration_std_lateral * [ ...
        sin(2*pi*35*t_full) + 0.35*sin(2*pi*73*t_full); ...
        cos(2*pi*41*t_full) + 0.30*sin(2*pi*67*t_full)];
    vib_axial = vibration_std_axial * ...
        (sin(2*pi*48*t_full) + 0.25*sin(2*pi*91*t_full));
    vib_gyro = deg2rad(3) * [ ...
        sin(2*pi*42*t_full); ...
        cos(2*pi*37*t_full); ...
        sin(2*pi*53*t_full)];

    p_true      = zeros(3, N);
    v_true      = zeros(3, N);
    q_true      = zeros(4, N);
    f_body_true = zeros(3, N);
    w_body_true = zeros(3, N);
    q_true(:,1) = [cos(pi/4); 0; sin(pi/4); 0];

    for k = 1:N-1
        tk = t_full(k);
        roll_rate  = deg2rad(50);
        pitch_rate = deg2rad(2 * sin(tk));
        yaw_rate   = deg2rad(1 * cos(tk));

        if tk < burn_time
            fx = f_axial_boost;
            fy = 0;
            fz = 0;
            if enable_boost_vibration
                fx = fx + vib_axial(k);
                fy = vib_lateral(1, k);
                fz = vib_lateral(2, k);
                roll_rate  = roll_rate  + vib_gyro(1, k);
                pitch_rate = pitch_rate + vib_gyro(2, k);
                yaw_rate   = yaw_rate   + vib_gyro(3, k);
            end
            f_body_true(:,k) = [fx; fy; fz];
            w_body_true(:,k) = [roll_rate; pitch_rate; yaw_rate];
        elseif enable_canard && tk >= canard_t_start && tk <= canard_t_end
            tau = (tk - canard_t_start) / (canard_t_end - canard_t_start);
            pulse = sin(pi * tau)^2;
            snap = sin(5 * pi * tau);

            fy = canard_lat_g * pulse + canard_buffet_std * snap;
            fz = 0.55 * canard_lat_g * pulse + 0.5 * canard_buffet_std * sin(7 * pi * tau);

            pitch_rate = pitch_rate + deg2rad(180) * pulse + canard_gyro_buffet_std * snap;
            yaw_rate   = yaw_rate   + deg2rad(110) * pulse + 0.6 * canard_gyro_buffet_std * sin(7 * pi * tau);

            f_body_true(:,k) = [0; fy; fz];
            w_body_true(:,k) = [roll_rate; pitch_rate; yaw_rate];
        else
            f_body_true(:,k) = [0; 0; 0];
            w_body_true(:,k) = [roll_rate; pitch_rate; yaw_rate];
        end

        if enable_ejection_shock && tk > 15.0 && tk < 15.2
            f_body_true(:,k) = [-15.0 * g; 0; 0];
            w_body_true(:,k) = [deg2rad(100); deg2rad(100); deg2rad(100)];
        end

        R_nb = ESEKF.quat2dcm(q_true(:,k));
        qdot = 0.5 * [0 -w_body_true(1,k) -w_body_true(2,k) -w_body_true(3,k);
                      w_body_true(1,k) 0 w_body_true(3,k) -w_body_true(2,k);
                      w_body_true(2,k) -w_body_true(3,k) 0 w_body_true(1,k);
                      w_body_true(3,k) w_body_true(2,k) -w_body_true(1,k) 0] * q_true(:,k);
        q_true(:,k+1) = q_true(:,k) + qdot * dt;
        q_true(:,k+1) = q_true(:,k+1) / norm(q_true(:,k+1));

        a_ned = R_nb * f_body_true(:,k) + S.g_ned;
        v_true(:,k+1) = v_true(:,k) + a_ned * dt;
        p_true(:,k+1) = p_true(:,k) + v_true(:,k) * dt + 0.5 * a_ned * dt^2;
    end

    f_body_true(:,N) = f_body_true(:,N-1);
    w_body_true(:,N) = w_body_true(:,N-1);

    [~, idx_apogee] = max(-p_true(3,:));

    truth.t = t_full(1:idx_apogee);
    truth.p = p_true(:, 1:idx_apogee);
    truth.v = v_true(:, 1:idx_apogee);
    truth.q = q_true(:, 1:idx_apogee);
    truth.f_body = f_body_true(:, 1:idx_apogee);
    truth.w_body = w_body_true(:, 1:idx_apogee);
    truth.dt = dt;
    truth.info = struct('T_end', truth.t(end), 'g', g, 'case_name', case_name);
end
