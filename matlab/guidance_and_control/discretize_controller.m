%% discretize_controller.m
%  Tustin (bilinear) discretization of all dynamic blocks.
%  Pipeline: rocket_params -> design_gains -> discretize_controller -> eom_6dof_discrete

function disc = discretize_controller(par_in, Ts_in)

    run_standalone = (nargin == 0);

    if run_standalone
        clear; clc; close all;
        par = rocket_params();
        par = design_gains(par);
        Ts = 0.0025;
        fprintf('====== DISCRETIZATION (standalone mode) ======\n');
        fprintf('Sample rate: %.0f Hz (Ts = %.4f s)\n\n', 1/Ts, Ts);
    else
        par = par_in;
        Ts = Ts_in;
    end

    disc = struct();
    disc.Ts = Ts;
    disc.Fs = 1/Ts;

    %% 1. Roll PI: Kp + Ki/s -> Tustin PI
    disc.roll_pi.Kp = par.Kp_roll;
    disc.roll_pi.Ki = par.Ki_roll;
    disc.roll_pi.Ti_coeff = Ts / 2;

    fprintf('--- Roll PI (Tustin) ---\n');
    fprintf('  Kp = %.6f, Ki = %.6f\n', disc.roll_pi.Kp, disc.roll_pi.Ki);
    fprintf('  Tustin integrator coeff = Ts/2 = %.6f\n\n', disc.roll_pi.Ti_coeff)

    %% 3. Actuator 2nd-order: Tustin IIR
    %  H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
    wn = par.wn_act;
    zeta = par.zeta_act;

    K = 2/Ts;
    denom = K^2 + 2*zeta*wn*K + wn^2;

    b0_act = wn^2 / denom;
    b1_act = 2*wn^2 / denom;
    b2_act = wn^2 / denom;
    a1_act = 2*(wn^2 - K^2) / denom;
    a2_act = (K^2 - 2*zeta*wn*K + wn^2) / denom;

    disc.actuator.b = [b0_act, b1_act, b2_act];
    disc.actuator.a = [1, a1_act, a2_act];
    disc.actuator.wn = wn;
    disc.actuator.zeta = zeta;

    fprintf('--- Actuator 2nd-order (Tustin) ---\n');
    fprintf('  wn = %.1f rad/s (%.1f Hz), zeta = %.2f\n', wn, wn/(2*pi), zeta);
    fprintf('  b = [%.8f, %.8f, %.8f]\n', disc.actuator.b);
    fprintf('  a = [1, %.8f, %.8f]\n\n', a1_act, a2_act);

    %% 4. PN LOS rate filter: 1/(tau*s+1) -> Tustin 1st-order IIR
    tau = 0.05;
    if isfield(par, 'pn_filter_tau'), tau = par.pn_filter_tau; end

    alpha_f = 2*tau/Ts;
    b0_lpf = 1 / (1 + alpha_f);
    b1_lpf = 1 / (1 + alpha_f);
    a1_lpf = (1 - alpha_f) / (1 + alpha_f);

    disc.los_lpf.b = [b0_lpf, b1_lpf];
    disc.los_lpf.a = [1, a1_lpf];
    disc.los_lpf.tau = tau;

    fprintf('--- LOS Rate LPF (Tustin) ---\n');
    fprintf('  tau = %.4f s\n', tau);
    fprintf('  b = [%.8f, %.8f]\n', disc.los_lpf.b);
    fprintf('  a = [1, %.8f]\n\n', a1_lpf);

    %% 5. Proportional gains
    disc.gains.KR  = par.KR;
    disc.gains.KA  = par.KA;
    disc.gains.KDC = par.KDC;
    disc.gains.wI  = par.wI;
    disc.gains.K_phi = par.K_phi;

    fprintf('--- Proportional Gains (no discretization needed) ---\n');
    fprintf('  KR=%.6f  KA=%.6f  KDC=%.6f  wI=%.6f  K_phi=%.4f\n\n', ...
        par.KR, par.KA, par.KDC, par.wI, par.K_phi);

    %% 6. Gain table
    if isfield(par, 'gain_table')
        disc.gain_table = par.gain_table;
        fprintf('--- Gain Table ---\n');
        fprintf('  V breakpoints: %s m/s\n', mat2str(par.gain_table.V_bp));
    end

    %% 7. Anti-windup parameters
    disc.delta_max = par.delta_max;
    disc.delta_rate_max = par.delta_rate_max;

    fprintf('--- Anti-Windup ---\n');
    fprintf('  delta_max = %.4f rad (%.1f deg)\n', par.delta_max, rad2deg(par.delta_max));
    fprintf('  delta_rate_max = %.4f rad/s (%.1f deg/s)\n\n', ...
        par.delta_rate_max, rad2deg(par.delta_rate_max));

    %% 8. Export summary
    fprintf('====== DISCRETIZATION COMPLETE ======\n');
    fprintf('All coefficients stored in disc struct.\n');
    fprintf('Sample rate: %.0f Hz (Ts = %.4f s)\n', disc.Fs, disc.Ts);

    if run_standalone
        validate_discretization(par, disc);
    end

    if nargout == 0 && ~run_standalone
        clear disc;
    end
end

%% Validation: Continuous vs Discrete
function validate_discretization(par, disc)

    Ts = disc.Ts;
    s = tf('s');
    z = tf('z', Ts);

    fprintf('\n====== VALIDATION ======\n');

    figure('Name','Discretization Validation','Position',[50 50 1500 900],...
           'Renderer','painters');

    % (a) Actuator step response
    wn = par.wn_act;  zeta = par.zeta_act;
    G_act_c = tf(wn^2, [1, 2*zeta*wn, wn^2]);
    G_act_d = tf(disc.actuator.b, disc.actuator.a, Ts);

    subplot(2,3,1);
    t_step = 0:Ts:0.05;
    [y_c, t_c] = step(G_act_c, t_step);
    [y_d, t_d] = step(G_act_d, t_step);
    plot(t_c*1000, y_c, 'b-', 'LineWidth', 1.5); hold on;
    stairs(t_d*1000, y_d, 'r--', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Response');
    title('Actuator Step Response');
    legend('Continuous','Discrete (Tustin)','Location','best');
    grid on;

    y_c_interp = interp1(t_c, y_c, t_d, 'linear', 'extrap');
    act_err = max(abs(y_c_interp - y_d));
    fprintf('  Actuator max step error: %.6f (%.4f%%)\n', act_err, act_err*100);

    % (b) Actuator Bode
    subplot(2,3,2);
    w_bode = logspace(0, log10(pi/Ts), 500);
    [mag_c, ph_c] = bode(G_act_c, w_bode);
    [mag_d, ph_d] = bode(G_act_d, w_bode);
    mag_c = squeeze(mag_c); ph_c = squeeze(ph_c);
    mag_d = squeeze(mag_d); ph_d = squeeze(ph_d);

    yyaxis left;
    semilogx(w_bode, 20*log10(mag_c), 'b-', w_bode, 20*log10(mag_d), 'r--', 'LineWidth', 1.2);
    ylabel('Magnitude (dB)');
    yyaxis right;
    semilogx(w_bode, ph_c, 'b:', w_bode, ph_d, 'r:', 'LineWidth', 1.0);
    ylabel('Phase (deg)');
    xlabel('\omega (rad/s)');
    title('Actuator Bode');
    legend('Cont mag','Disc mag','Cont ph','Disc ph','Location','southwest');
    grid on;

    % (c) Roll PI inner loop step
    Kp_r = par.Kp_roll;  Ki_r = par.Ki_roll;
    C_pi_c = Kp_r + Ki_r/s;

    rho = par.rho0 * exp(-par.design_h / par.h_scale);
    Q = 0.5 * rho * par.design_V^2;
    L_p     = (Q*par.Sref*par.d^2*par.Clp) / (2*par.Ixx*par.design_V);
    L_delta = (Q*par.Sref*par.d*par.Clda) / par.Ixx;
    G_roll = tf(L_delta, [1, -L_p]);

    L_inner_c = C_pi_c * G_act_c * G_roll;
    T_inner_c = feedback(L_inner_c, 1);

    C_pi_d_num = [Kp_r + Ki_r*Ts/2,  -Kp_r + Ki_r*Ts/2];
    C_pi_d_den = [1, -1];
    C_pi_d = tf(C_pi_d_num, C_pi_d_den, Ts);

    G_roll_d = c2d(G_roll, Ts, 'tustin');
    L_inner_d = C_pi_d * G_act_d * G_roll_d;
    T_inner_d = feedback(L_inner_d, 1);

    subplot(2,3,3);
    t_roll = 0:Ts:0.3;
    [yr_c, tr_c] = step(T_inner_c, t_roll);
    [yr_d, tr_d] = step(T_inner_d, t_roll);
    plot(tr_c*1000, yr_c, 'b-', 'LineWidth', 1.5); hold on;
    stairs(tr_d*1000, yr_d, 'r--', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Response');
    title('Roll Inner CL Step');
    legend('Continuous','Discrete','Location','best');
    grid on;

    yr_c_interp = interp1(tr_c, yr_c, tr_d, 'linear', 'extrap');
    roll_err = max(abs(yr_c_interp - yr_d));
    fprintf('  Roll inner max step error: %.6f\n', roll_err);

    % (d) LOS LPF step
    tau = disc.los_lpf.tau;
    G_lpf_c = tf(1, [tau, 1]);
    G_lpf_d = tf(disc.los_lpf.b, disc.los_lpf.a, Ts);

    subplot(2,3,4);
    t_lpf = 0:Ts:0.3;
    [yl_c, tl_c] = step(G_lpf_c, t_lpf);
    [yl_d, tl_d] = step(G_lpf_d, t_lpf);
    plot(tl_c*1000, yl_c, 'b-', 'LineWidth', 1.5); hold on;
    stairs(tl_d*1000, yl_d, 'r--', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Response');
    title('LOS LPF Step (1st order)');
    legend('Continuous','Discrete','Location','best');
    grid on;

    % (e) Full pitch autopilot CL
    KR = par.KR;  KA = par.KA;  KDC = par.KDC;
    plant_ss_c = build_pitch_plant(par);
    P_act_c = G_act_c * plant_ss_c;
    P_az_c = P_act_c(1);  P_q_c = P_act_c(2);
    G_qcmd2az_c = minreal(P_az_c * KR / (1 + KR * P_q_c));
    L_accel_c = KA * G_qcmd2az_c;
    T_accel_c = feedback(L_accel_c, 1);
    T_full_c  = KDC * T_accel_c;

    plant_ss_d = c2d(plant_ss_c, Ts, 'tustin');
    P_act_d = G_act_d * plant_ss_d;
    P_az_d = P_act_d(1);  P_q_d = P_act_d(2);
    G_qcmd2az_d = minreal(P_az_d * KR / (1 + KR * P_q_d));
    L_accel_d = KA * G_qcmd2az_d;
    T_accel_d = feedback(L_accel_d, 1);
    T_full_d  = KDC * T_accel_d;

    subplot(2,3,5);
    t_pitch = 0:Ts:0.8;
    [yp_c, tp_c] = step(T_full_c, t_pitch);
    [yp_d, tp_d] = step(T_full_d, t_pitch);
    plot(tp_c*1000, yp_c, 'b-', 'LineWidth', 1.5); hold on;
    stairs(tp_d*1000, yp_d, 'r--', 'LineWidth', 1.5);
    xlabel('Time (ms)'); ylabel('Response');
    title('Pitch Full CL Step (nz_{cmd} → nz)');
    legend('Continuous','Discrete','Location','best');
    grid on;

    yp_c_interp = interp1(tp_c, yp_c, tp_d, 'linear', 'extrap');
    pitch_err = max(abs(yp_c_interp - yp_d));
    fprintf('  Pitch CL max step error: %.6f\n', pitch_err);

    % (f) Sample rate sensitivity
    subplot(2,3,6);
    Ts_test = [0.001, 0.005, 0.01, 0.02, 0.05];
    err_act_arr = zeros(size(Ts_test));
    err_pitch_arr = zeros(size(Ts_test));

    for j = 1:length(Ts_test)
        Ts_j = Ts_test(j);

        K_j = 2/Ts_j;
        wn_a = par.wn_act; z_a = par.zeta_act;
        d_j = K_j^2 + 2*z_a*wn_a*K_j + wn_a^2;
        b_j = wn_a^2/d_j * [1, 2, 1];
        a_j = [1, 2*(wn_a^2-K_j^2)/d_j, (K_j^2-2*z_a*wn_a*K_j+wn_a^2)/d_j];
        G_act_j = tf(b_j, a_j, Ts_j);

        t_j = 0:Ts_j:0.05;
        [y_cj, t_cj] = step(G_act_c, t_j);
        [y_dj, ~] = step(G_act_j, t_j);
        y_ci = interp1(t_cj, y_cj, t_j', 'linear', 'extrap');
        err_act_arr(j) = max(abs(y_ci - y_dj));

        plant_j = c2d(plant_ss_c, Ts_j, 'tustin');
        P_j = G_act_j * plant_j;
        Pz_j = P_j(1); Pq_j = P_j(2);
        Gq2a_j = minreal(Pz_j*KR/(1+KR*Pq_j));
        T_j = KDC * feedback(KA*Gq2a_j, 1);
        t_pj = 0:Ts_j:0.8;
        [ypc_j, tpc_j] = step(T_full_c, t_pj);
        [ypd_j, ~] = step(T_j, t_pj);
        ypc_i = interp1(tpc_j, ypc_j, t_pj', 'linear', 'extrap');
        err_pitch_arr(j) = max(abs(ypc_i - ypd_j));
    end

    semilogy(1./Ts_test, err_act_arr, 'bo-', 1./Ts_test, err_pitch_arr, 'rs-', 'LineWidth', 1.5);
    xlabel('Sample Rate (Hz)'); ylabel('Max Step Error');
    title('Sample Rate Sensitivity');
    legend('Actuator','Pitch CL','Location','best');
    grid on;
    xline(100, 'k--', '100Hz target');

    fprintf('\n--- Sample Rate Sensitivity ---\n');
    fprintf('  %6s | %10s | %10s\n', 'Fs(Hz)', 'Act err', 'Pitch err');
    for j = 1:length(Ts_test)
        fprintf('  %6.0f | %10.6f | %10.6f\n', 1/Ts_test(j), err_act_arr(j), err_pitch_arr(j));
    end

    sgtitle(sprintf('Discretization Validation (Ts=%.0fms, Fs=%.0fHz)', Ts*1000, 1/Ts));
    drawnow;

    fprintf('\n====== VALIDATION COMPLETE ======\n');
end

%% Helper: Pitch plant state-space
function plant_ss = build_pitch_plant(par)
    V   = par.design_V;
    h   = par.design_h;
    m   = par.design_m;
    rho = par.rho0 * exp(-h / par.h_scale);
    Q   = 0.5 * rho * V^2;
    S   = par.Sref;
    d   = par.d;

    Z_alpha = -(Q*S*par.CNa) / (m*V);
    Z_delta = -(Q*S*par.CNd) / (m*V);
    M_alpha = (Q*S*d*par.Cma) / par.Iyy;
    M_q     = (Q*S*d^2*par.Cmq) / (2*par.Iyy*V);
    M_delta = (Q*S*d*par.Cmd) / par.Iyy;

    A = [Z_alpha, 1; M_alpha, M_q];
    B = [Z_delta; M_delta];
    C = [-V*Z_alpha, 0; 0, 1];
    D = [-V*Z_delta; 0];
    plant_ss = ss(A, B, C, D);
end
