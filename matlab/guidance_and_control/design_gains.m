function par = design_gains(par)
%DESIGN_GAINS  3-loop autopilot + roll damper gain design.
%  par.gain_method: 'bode' | 'pole' | 'zarchan'
%  Outputs: par.KR, par.KA, par.KDC, par.wI, par.K_phi, par.Kp_roll, par.Ki_roll

fprintf('====== GAIN DESIGN (method: %s) ======\n', par.gain_method);

%% 1. Design condition
V   = par.design_V;  h = par.design_h;  m = par.design_m;
rho = par.rho0 * exp(-h / par.h_scale);
Q   = 0.5 * rho * V^2;
S   = par.Sref;  d = par.d;

fprintf('Design point: V=%.0f m/s, h=%.0f m, m=%.2f kg\n', V, h, m);
fprintf('  qbar = %.1f Pa\n', Q);

%% 2. Dimensional derivatives (pitch)
Z_alpha = -(Q*S*par.CNa) / (m*V);
Z_delta = -(Q*S*par.CNd) / (m*V);
M_alpha = (Q*S*d*par.Cma) / par.Iyy;
M_q     = (Q*S*d^2*par.Cmq) / (2*par.Iyy*V);
M_delta = (Q*S*d*par.Cmd) / par.Iyy;

fprintf('\nDimensional derivatives (pitch):\n');
fprintf('  Z_alpha=%.4f  Z_delta=%.4f  M_alpha=%.4f  M_q=%.4f  M_delta=%.4f\n', ...
    Z_alpha, Z_delta, M_alpha, M_q, M_delta);

%% 3. Plant state-space
A_plant = [Z_alpha, 1; M_alpha, M_q];
B_plant = [Z_delta; M_delta];
C_plant = [-V*Z_alpha, 0; 0, 1];
D_plant = [-V*Z_delta; 0];
plant_ss = ss(A_plant, B_plant, C_plant, D_plant);

G_az = plant_ss(1);  G_q = plant_ss(2);

%% 4. Actuator
G_act = tf(par.wn_act^2, [1, 2*par.zeta_act*par.wn_act, par.wn_act^2]);

%% 5. Pitch gains
wI = 0;
switch lower(par.gain_method)
    case 'bode'
        [KR, KA, KDC, wI, Pm_r, Pm_a, info_full, Ms] = ...
            design_pitch_bode(par, G_act, G_q, G_az, plant_ss);
    case 'pole'
        [KR, KA, KDC, wI, Pm_r, Pm_a, info_full, Ms] = ...
            design_pitch_pole(par, G_act, G_q, G_az, plant_ss, V);
    case 'zarchan'
        [KR, KA, KDC, wI, Pm_r, Pm_a, info_full, Ms] = ...
            design_pitch_zarchan(par, G_act, G_q, G_az, plant_ss, ...
                                 Z_alpha, Z_delta, M_alpha, M_q, M_delta, V);
    otherwise
        error('Unknown gain_method: %s', par.gain_method);
end

%% 6. Roll gains
[K_phi, Kp_roll, Ki_roll, Pm_ri, Pm_ro] = design_roll(par, G_act, Q, S, d, V);

%% 7. Store
par.KR = KR;  par.KA = KA;  par.KDC = KDC;
par.wI = wI;
par.K_phi = K_phi;  par.Kp_roll = Kp_roll;  par.Ki_roll = Ki_roll;

par.design_info = struct('Pm_rate', Pm_r, 'Pm_accel', Pm_a, ...
    'Pm_roll_inner', Pm_ri, 'Pm_roll_outer', Pm_ro, ...
    'ts_pitch', info_full.SettlingTime, 'OS_pitch', info_full.Overshoot, ...
    'Ms', Ms, 'Q_design', Q, 'method', par.gain_method);

fprintf('\n====== DESIGN COMPLETE ======\n');
fprintf('  KR=%.6f  KA=%.6f  KDC=%.6f  wI=%.6f\n', KR, KA, KDC, wI);
fprintf('  K_phi=%.4f  Kp_roll=%.4f  Ki_roll=%.4f\n\n', K_phi, Kp_roll, Ki_roll);

end

%% Bode sequential loop closure (pitch/yaw)
function [KR, KA, KDC, wI, Pm_r, Pm_a, info_full, Ms] = ...
         design_pitch_bode(par, G_act, G_q, G_az, plant_ss)

    s = tf('s');

    G_rate_ol = G_act * G_q;
    [mag_r, ~] = bode(G_rate_ol, par.wc_rate);
    KR = 1.0 / squeeze(mag_r);

    [~, Pm_r] = margin(KR * G_rate_ol);
    fprintf('\n--- [Bode] Rate Loop ---\n  KR=%.6f, PM=%.1f deg\n', KR, Pm_r);

    P_act = G_act * plant_ss;
    G_qcmd_to_az = minreal(P_act(1)*KR / (1 + KR*P_act(2)));
    [mag_a, ~] = bode(G_qcmd_to_az, par.wc_accel);
    KA = 1.0 / squeeze(mag_a);

    wI = par.wI_accel;

    if wI > 0
        L_accel = (KA + wI/s) * G_qcmd_to_az;
        fprintf('\n--- [Bode] Accel Loop (PI: KA + wI/s) ---\n  KA=%.6f, wI=%.2f\n', KA, wI);
    else
        L_accel = KA * G_qcmd_to_az;
        fprintf('\n--- [Bode] Accel Loop (P only) ---\n  KA=%.6f\n', KA);
    end

    T_accel = feedback(L_accel, 1);
    [~, Pm_a] = margin(L_accel);
    fprintf('  PM=%.1f deg\n', Pm_a);

    KDC = 1.0 / dcgain(T_accel);
    T_full = KDC * T_accel;
    info_full = stepinfo(T_full);
    fprintf('\n--- [Bode] Full CL ---\n  KDC=%.6f, ts=%.3fs, OS=%.1f%%\n', ...
        KDC, info_full.SettlingTime, info_full.Overshoot);

    if wI > 0
        L_full = G_act * ((KA + wI/s)*G_az + G_q) * KR;
    else
        L_full = G_act * (KA*G_az + G_q) * KR;
    end
    try [Ms,~] = getPeakGain(minreal(1/(1+L_full))); fprintf('  Ms=%.2f\n', Ms);
    catch, Ms = NaN; end

    if Pm_r < 45, fprintf('  WARNING: Rate PM=%.1f deg < 45 deg\n', Pm_r); end
    if Pm_a < 30, fprintf('  WARNING: Accel PM=%.1f deg < 30 deg\n', Pm_a);
    elseif Pm_a < 45, fprintf('  WARNING: Accel PM=%.1f deg < 45 deg\n', Pm_a); end
    if info_full.Overshoot > 20, fprintf('  WARNING: OS=%.1f%% > 20%%\n', info_full.Overshoot); end
end

%% Adaptive pole placement + PM verification (pitch/yaw)
function [KR, KA, KDC, wI, Pm_r, Pm_a, info_full, Ms] = ...
         design_pitch_pole(par, G_act, G_q, G_az, plant_ss, V)

    s = tf('s');
    g = par.g;

    if isfield(par, 'pole_zeta'),     zeta_d = par.pole_zeta;
    else,                             zeta_d = 0.7; end

    if isfield(par, 'pole_wn_scale'), wn_scale = par.pole_wn_scale;
    else,                             wn_scale = 1.0; end

    wn_max = par.wn_act / 3;

    if isfield(par, 'pole_pi_ratio'), pi_ratio = par.pole_pi_ratio;
    else,                             pi_ratio = 0.3; end

    if isfield(par, 'pole_PM_min'),   PM_min = par.pole_PM_min;
    else,                             PM_min = 30; end

    if isfield(par, 'pole_max_iter'), max_iter = par.pole_max_iter;
    else,                             max_iter = 15; end

    Za = plant_ss.A(1,1);
    Ma = plant_ss.A(2,1);
    Mq = plant_ss.A(2,2);
    Zd = plant_ss.B(1);
    Md = plant_ss.B(2);

    % Open-loop short-period frequency
    p0 = Za*Mq - Ma;
    wsp = sqrt(abs(p0));

    fprintf('\n--- [Pole] Adaptive Pole Placement + PM Verification ---\n');
    fprintf('  wsp=%.1f rad/s, zeta=%.2f, wn_scale=%.1f, wn_max=%.1f, PM_min=%d deg\n', ...
        wsp, zeta_d, wn_scale, wn_max, PM_min);

    % Augmented model: [alpha, q, xi] with accel error integrator
    A3 = [Za,       1,   0;
          Ma,      Mq,   0;
          V*Za/g,   0,   0];

    B3 = [Zd;
          Md;
          V*Zd/g];

    wn_try  = min(wn_scale * wsp, wn_max);
    best_PM_a = -inf;
    best = struct();
    converged = false;

    for iter = 1:max_iter
        pi_val = pi_ratio * wn_try;

        p_dom = -zeta_d*wn_try + 1j*wn_try*sqrt(1-zeta_d^2);
        desired = [p_dom; conj(p_dom); -pi_val];

        try
            K_fb = place(A3, B3, desired);
        catch
            wn_try = wn_try * 0.85;
            continue;
        end

        k_a  = K_fb(1);
        k_q  = K_fb(2);
        k_xi = K_fb(3);

        % Recover 3-loop gains
        KR_try = k_q / (1 - (k_a * Zd) / Za);
        KA_try = k_a / (k_q * (V*Za)/g);
        wI_try = -k_xi / k_q;

        if KR_try <= 0 || wI_try <= 0
            if iter <= 3
                fprintf('    iter %2d: wn=%5.1f -> KR=%.4f wI=%.2f (sign issue, reducing wn)\n', ...
                    iter, wn_try, KR_try, wI_try);
            end
            wn_try = wn_try * 0.85;
            continue;
        end

        [~, Pm_r_try] = margin(KR_try * G_act * G_q);

        P_act = G_act * plant_ss;
        G_qcmd_to_az = minreal(P_act(1)*KR_try / (1 + KR_try*P_act(2)));
        if wI_try > 0
            L_accel = (KA_try + wI_try/s) * G_qcmd_to_az;
        else
            L_accel = KA_try * G_qcmd_to_az;
        end
        [~, Pm_a_try] = margin(L_accel);

        if Pm_a_try > best_PM_a && Pm_r_try > 0
            best_PM_a = Pm_a_try;
            best.KR = KR_try;  best.KA = KA_try;  best.wI = wI_try;
            best.Pm_r = Pm_r_try;  best.Pm_a = Pm_a_try;
            best.wn = wn_try;  best.iter = iter;
        end

        if Pm_a_try >= PM_min && Pm_r_try >= PM_min
            converged = true;
            fprintf('    iter %2d: wn=%5.1f -> KR=%+.4f KA=%+.4f wI=%+.2f | PM_r=%.1f deg PM_a=%.1f deg OK\n', ...
                iter, wn_try, KR_try, KA_try, wI_try, Pm_r_try, Pm_a_try);
            break;
        else
            if Pm_r_try < 0 || Pm_a_try < 0
                wn_try = wn_try * 0.8;
            elseif Pm_r_try < PM_min
                wn_try = wn_try * 0.9;
            else
                wn_try = wn_try * 0.9;
            end

            if iter <= 5 || mod(iter,5) == 0
                fprintf('    iter %2d: wn=%5.1f -> PM_r=%.1f deg PM_a=%.1f deg (wn->%.1f)\n', ...
                    iter, wn_try/0.9, Pm_r_try, Pm_a_try, wn_try);
            end
        end
    end

    if ~converged
        if best_PM_a > -inf
            fprintf('    WARNING: did not converge, using best PM_a=%.1f deg (wn=%.1f, iter%d)\n', ...
                best_PM_a, best.wn, best.iter);
        else
            error('Pole placement: all iterations failed');
        end
    end

    KR = best.KR;
    KA = best.KA;
    wI = best.wI;
    Pm_r = best.Pm_r;
    Pm_a = best.Pm_a;

    fprintf('  Final: wn=%.1f, KR=%.4f, KA=%.4f, wI=%.4f\n', best.wn, KR, KA, wI);

    P_act = G_act * plant_ss;
    G_qcmd_to_az = minreal(P_act(1)*KR / (1 + KR*P_act(2)));
    if wI > 0
        L_accel = (KA + wI/s) * G_qcmd_to_az;
    else
        L_accel = KA * G_qcmd_to_az;
    end
    T_accel = feedback(L_accel, 1);

    KDC = 1.0 / dcgain(T_accel);
    if isinf(KDC) || isnan(KDC), KDC = 1.0; end
    T_full = KDC * T_accel;

    info_full = stepinfo(T_full);
    fprintf('  KDC=%.4f, ts=%.3fs, OS=%.1f%%\n', KDC, info_full.SettlingTime, info_full.Overshoot);

    if wI > 0
        L_full = G_act * ((KA + wI/s)*G_az + G_q) * KR;
    else
        L_full = G_act * (KA*G_az + G_q) * KR;
    end
    try [Ms,~] = getPeakGain(minreal(1/(1+L_full))); fprintf('  Ms=%.2f\n', Ms);
    catch, Ms = NaN; end

    if Pm_r < 30, fprintf('  WARNING: Rate PM=%.1f deg < 30 deg\n', Pm_r); end
    if Pm_a < 30, fprintf('  WARNING: Accel PM=%.1f deg < 30 deg\n', Pm_a); end
    if info_full.Overshoot > 25, fprintf('  WARNING: OS=%.1f%% > 25%%\n', info_full.Overshoot); end
end

%% Roll damper
function [K_phi, Kp_roll, Ki_roll, Pm_ri, Pm_ro] = design_roll(par, G_act, Q, S, d, V)

    s = tf('s');

    L_p     = (Q*S*d^2*par.Clp) / (2*par.Ixx*V);
    L_delta = (Q*S*d*par.Clda)  / par.Ixx;
    roll_tf = tf(L_delta, [1, -L_p]);
    fprintf('\n--- Roll ---\n  L_p=%.4f, L_delta=%.4f\n', L_p, L_delta);

    G_roll_ol = G_act * roll_tf;
    [mag_ri, ~] = bode(G_roll_ol, par.wc_roll_inner);
    Kp_roll = 1.0 / squeeze(mag_ri);
    Ki_roll = Kp_roll * par.roll_pi_zero;

    C_pi = Kp_roll + Ki_roll/s;
    inner_cl = feedback(C_pi * G_roll_ol, 1);
    [~, Pm_ri] = margin(C_pi * G_roll_ol);
    fprintf('  Inner: Kp=%.4f, Ki=%.4f, PM=%.1f deg\n', Kp_roll, Ki_roll, Pm_ri);

    % Outer phi loop via sequential loop closure
    L_outer_1 = inner_cl * tf(1, [1 0]);
    [mag_ro, ~] = bode(L_outer_1, par.wc_roll_outer);
    K_phi = 1.0 / squeeze(mag_ro);

    L_outer = K_phi * L_outer_1;
    [~, Pm_ro] = margin(L_outer);

    outer_cl = feedback(L_outer, 1);
    try
        info_roll = stepinfo(outer_cl, 'SettlingTimeThreshold', 0.02);
        ts_ro = info_roll.SettlingTime;
        os_ro = info_roll.Overshoot;
    catch
        ts_ro = NaN;  os_ro = NaN;
    end
    warning('on', 'Control:analysis:stepinfo:NotSettled');

    fprintf('  Outer: K_phi=%.4f, PM=%.1f deg, ts=%.3fs, OS=%.1f%%\n', ...
        K_phi, Pm_ro, ts_ro, os_ro);

    if Pm_ro < 30
        fprintf('  WARNING: Roll Outer PM=%.1f deg < 30 deg\n', Pm_ro);
    end
end

%% Zarchan full coefficient matching (canard)
function [KR, KA, KDC, wI, Pm_r, Pm_a, info_full, Ms] = ...
         design_pitch_zarchan(par, G_act, G_q, G_az, plant_ss, ...
                              Za, Zd, Ma, Mq, Md, V)
%  Closed-form 3-loop gain solution via characteristic polynomial matching.
%  Variable substitution: x1=KR, x2=KR*KA, x3=KR*wI
%  Solves 3x3 linear system matching s^2, s^1, s^0 coefficients.

    s = tf('s');
    g = par.g;
    lam = V / g;

    p1 = -(Za + Mq);
    p0 = Za*Mq - Ma;
    f0 = Md*Za - Zd*Ma;

    wsp = sqrt(p0);

    if isfield(par, 'zarchan_zeta'),    zeta_d = par.zarchan_zeta;
    else,                               zeta_d = 0.7; end

    if isfield(par, 'zarchan_wn_scale'), wn_scale = par.zarchan_wn_scale;
    else,                                wn_scale = 0.9; end

    wn_max = par.wn_act / 3;

    if isfield(par, 'zarchan_pi_ratio'), pi_ratio = par.zarchan_pi_ratio;
    else,                                pi_ratio = 0.3; end

    wn = min(wn_scale * wsp, wn_max);
    pi_val = pi_ratio * wn;

    fprintf('\n--- [Zarchan] Full Coefficient Matching ---\n');
    fprintf('  wsp=%.1f, wn=%.1f (%.0f%% of wsp, max=%.0f)\n', wsp, wn, wn/wsp*100, wn_max);
    fprintf('  zeta=%.2f, pi=%.1f (%.0f%% of wn)\n', zeta_d, pi_val, pi_ratio*100);

    c2 = 2*zeta_d*wn + pi_val;
    c1 = wn^2 + 2*zeta_d*wn*pi_val;
    c0 = wn^2 * pi_val;

    A_sys = [Md,   lam*Zd*(Mq + c2),    -lam*Zd;
             f0,   lam*(Zd*c1 - f0),     lam*Zd*Mq;
             0,    lam*Zd*c0,           -lam*f0];

    b_sys = [c2 - p1;
             c1 - p0;
             c0];

    det_A = det(A_sys);
    fprintf('  det(A) = %.4e\n', det_A);

    if abs(det_A) < 1e-12
        error('Zarchan coefficient matching: singular matrix (det=%.2e)', det_A);
    end

    x = A_sys \ b_sys;
    KR = x(1);
    KA = x(2) / x(1);
    wI = x(3) / x(1);

    alpha3 = 1 - lam*Zd*x(2);

    fprintf('  KR=%.4f, KA=%.4f, wI=%.4f, alpha3=%.4f\n', KR, KA, wI, alpha3);

    % s^0 verification
    s0_err = abs(-lam*f0*x(3) - alpha3*c0);
    fprintf('  s^0 verification err = %.2e\n', s0_err);

    % CL poles (servo ignored)
    char_roots = roots([alpha3, alpha3*c2, alpha3*c1, alpha3*c0]);
    fprintf('  CL poles (no servo): ');
    for ir = 1:length(char_roots)
        fprintf('%.2f%+.2fj  ', real(char_roots(ir)), imag(char_roots(ir)));
    end
    fprintf('\n');

    if KR < 0, fprintf('  WARNING: KR < 0\n'); end
    if wI < 0, fprintf('  WARNING: wI < 0\n'); end
    if KA < 0, fprintf('  WARNING: KA < 0\n'); end

    G_rate_ol = G_act * G_q;
    [~, Pm_r] = margin(KR * G_rate_ol);
    fprintf('  Rate loop PM = %.1f deg\n', Pm_r);

    P_act = G_act * plant_ss;
    G_qcmd_to_az = minreal(P_act(1)*KR / (1 + KR*P_act(2)));

    if wI > 0
        L_accel = (KA + wI/s) * G_qcmd_to_az;
    else
        L_accel = KA * G_qcmd_to_az;
    end
    T_accel = feedback(L_accel, 1);
    [~, Pm_a] = margin(L_accel);
    fprintf('  Accel loop PM = %.1f deg\n', Pm_a);

    KDC = 1.0 / dcgain(T_accel);
    if isinf(KDC) || isnan(KDC), KDC = 1.0; end
    T_full = KDC * T_accel;

    info_full = stepinfo(T_full);
    fprintf('  ts=%.3fs, OS=%.1f%%, KDC=%.4f\n', ...
        info_full.SettlingTime, info_full.Overshoot, KDC);

    if wI > 0
        L_full = G_act * ((KA + wI/s)*G_az + G_q) * KR;
    else
        L_full = G_act * (KA*G_az + G_q) * KR;
    end
    try [Ms,~] = getPeakGain(minreal(1/(1+L_full))); fprintf('  Ms=%.2f\n', Ms);
    catch, Ms = NaN; end

    if Pm_r < 30, fprintf('  WARNING: Rate PM=%.1f deg < 30 deg\n', Pm_r); end
    if Pm_a < 30, fprintf('  WARNING: Accel PM=%.1f deg < 30 deg\n', Pm_a); end
    if info_full.Overshoot > 25, fprintf('  WARNING: OS=%.1f%% > 25%%\n', info_full.Overshoot); end
end
