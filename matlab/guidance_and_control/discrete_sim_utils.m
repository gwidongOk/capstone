classdef discrete_sim_utils
%DISCRETE_SIM_UTILS  Static methods for discrete-time 6-DOF simulation.

methods(Static)

%% Controller state init
function ctrl = init_ctrl_state(~)
    ctrl.pitch_int = 0;    ctrl.pitch_e_prev = 0;
    ctrl.yaw_int = 0;      ctrl.yaw_e_prev = 0;
    ctrl.roll_int = 0;     ctrl.roll_e_prev = 0;
    ctrl.prev_vclose = 0;  ctrl.prev_vD = -1;
end

%% Wind model
function w = get_wind(t, alt, par)
    if ~isfield(par,'wind_model') || isempty(par.wind_model)
        if isfield(par,'wind_NED') && ~isempty(par.wind_NED)
            w = par.wind_NED(:);
        else
            w = [0;0;0];
        end
        return;
    end
    switch lower(par.wind_model)
        case 'none',     w = [0;0;0];
        case 'constant', w = par.wind_NED(:);
        case 'powerlaw'
            h = max(alt, par.wind_h_min);
            U = par.wind_ref_speed*(h/par.wind_ref_height)^par.wind_alpha;
            w = -U*[cos(par.wind_direction); sin(par.wind_direction); 0];
        case 'powerlaw_dryden'
            h = max(alt, par.wind_h_min);
            U = par.wind_ref_speed*(h/par.wind_ref_height)^par.wind_alpha;
            wm = -U*[cos(par.wind_direction); sin(par.wind_direction); 0];
            tc = max(par.gust_t(1), min(par.gust_t(end), t));
            g = [par.gust_interp_N(tc); par.gust_interp_E(tc); par.gust_interp_D(tc)];
            w = wm + g;
        otherwise, w = [0;0;0];
    end
end

%% Discrete autopilot
function [fin_cmd, ctrl] = autopilot(t, nz_cmd, ny_cmd, ...
        omega_b, quat, V, qbar, par, disc, ctrl, vel_air_NED)

    Ts = disc.Ts;
    p = omega_b(1); q_rate = omega_b(2); r = omega_b(3);

    if isfield(par,'disable_all_control') && par.disable_all_control
        fin_cmd = [0;0;0;0]; return;
    end

    if isfield(par,'V_ctrl_on') && par.V_ctrl_on > 0
        ctrl_active = (V > par.V_ctrl_on);
    else
        ctrl_active = (qbar > par.qbar_min_ctrl);
    end

    % Body-x velocity for nz_fb: nz_fb = q * u_body / g
    if nargin >= 11 && ~isempty(vel_air_NED)
        R_BN = gnc_utils.quat2dcm_bn(quat);
        vb = R_BN * vel_air_NED(:);
        u_body = max(vb(1), 1.0);
    else
        u_body = V;
    end

    % Gain lookup
    if isfield(par,'gain_table')
        Vb = par.gain_table.V_bp;
        Vc = max(Vb(1), min(Vb(end), V));
        KR_g  = interp1(Vb,par.gain_table.KR, Vc);
        KA_g  = interp1(Vb,par.gain_table.KA, Vc);
        KDC_g = interp1(Vb,par.gain_table.KDC,Vc);
        K_phi_g   = interp1(Vb,par.gain_table.K_phi,  Vc);
        Kp_roll_g = interp1(Vb,par.gain_table.Kp_roll,Vc);
        Ki_roll_g = interp1(Vb,par.gain_table.Ki_roll,Vc);
        if isfield(par.gain_table,'wI')
            wI_g = interp1(Vb,par.gain_table.wI,Vc);
        elseif isfield(par,'wI'), wI_g = par.wI;
        else, wI_g = 0; end
    else
        KR_g=par.KR; KA_g=par.KA; KDC_g=par.KDC;
        K_phi_g=par.K_phi; Kp_roll_g=par.Kp_roll; Ki_roll_g=par.Ki_roll;
        if isfield(par,'wI'), wI_g=par.wI; else, wI_g=0; end
    end

    % Pitch/Yaw
    if ctrl_active
        nz_fb = q_rate*u_body/par.g;
        ny_fb = -r*u_body/par.g;
        ez = KDC_g*nz_cmd - nz_fb;
        ey = KDC_g*ny_cmd - ny_fb;
        if wI_g > 0
            ctrl.pitch_int = ctrl.pitch_int + (Ts/2)*(ez + ctrl.pitch_e_prev);
            ctrl.yaw_int   = ctrl.yaw_int   + (Ts/2)*(ey + ctrl.yaw_e_prev);
        end
        de_v = KR_g*(KA_g*ez + wI_g*ctrl.pitch_int - q_rate);
        dr_v = KR_g*(KA_g*ey + wI_g*ctrl.yaw_int   - r);
        % Anti-windup rollback
        if wI_g > 0
            if abs(de_v)>par.delta_max && sign(ez)==sign(de_v)
                ctrl.pitch_int = ctrl.pitch_int - (Ts/2)*(ez+ctrl.pitch_e_prev); end
            if abs(dr_v)>par.delta_max && sign(ey)==sign(dr_v)
                ctrl.yaw_int = ctrl.yaw_int - (Ts/2)*(ey+ctrl.yaw_e_prev); end
        end
        ctrl.pitch_e_prev = ez;
        ctrl.yaw_e_prev   = ey;
    else
        de_v = -KR_g*q_rate;
        dr_v = -KR_g*r;
    end

    if isfield(par,'disable_pitch_yaw') && par.disable_pitch_yaw
        de_v = 0; dr_v = 0;
    end

    % Roll
    q0=quat(1); q1=quat(2); q2=quat(3); q3=quat(4);
    th = asin(max(-1,min(1, 2*(q0*q2-q3*q1))));
    if abs(th) < deg2rad(70)
        phi = atan2(2*(q0*q1+q2*q3), 1-2*(q1^2+q2^2));
        p_cmd = K_phi_g * (mod(-phi+pi,2*pi)-pi);
    elseif abs(th) > deg2rad(80)
        p_cmd = 0;
    else
        phi = atan2(2*(q0*q1+q2*q3), 1-2*(q1^2+q2^2));
        blend = (abs(th)-deg2rad(70))/deg2rad(10);
        p_cmd = K_phi_g*(mod(-phi+pi,2*pi)-pi)*(1-blend);
    end
    ep = p_cmd - p;
    ctrl.roll_int = ctrl.roll_int + (Ts/2)*(ep + ctrl.roll_e_prev);
    da_raw = Kp_roll_g*ep + Ki_roll_g*ctrl.roll_int;
    da_v = max(-par.delta_max, min(par.delta_max, da_raw));
    if abs(da_raw)>par.delta_max && sign(ep)==sign(da_raw)
        ctrl.roll_int = ctrl.roll_int - (Ts/2)*(ep+ctrl.roll_e_prev); end
    ctrl.roll_e_prev = ep;

    % Fin mixing
    if isfield(par,'roll_alloc_max') && par.roll_alloc_max>0
        rl = par.roll_alloc_max;
    else, rl = par.delta_max; end
    da_a = max(-rl,min(rl,da_v));
    rem = par.delta_max - abs(da_a);
    de_a = max(-rem,min(rem,de_v));
    dr_a = max(-rem,min(rem,dr_v));
    fin_cmd = [de_a+da_a; dr_a+da_a; -de_a+da_a; -dr_a+da_a];
    fin_cmd = max(-par.delta_max, min(par.delta_max, fin_cmd));

    persistent dbg_count
    if isempty(dbg_count), dbg_count = 0; end
    if dbg_count < 5
        fprintf('  [DBG] t=%.4f V=%.1f qbar=%.0f ctrl=%d de=%.4f dr=%.4f da=%.4f fin=[%.4f %.4f %.4f %.4f]\n', ...
            t, V, qbar, ctrl_active, de_v, dr_v, da_v, fin_cmd(1), fin_cmd(2), fin_cmd(3), fin_cmd(4));
        dbg_count = dbg_count + 1;
    end
end

%% Actuator step (RK4)
function act = actuator_step(act, fin_cmd, par, dt)
    wn = par.wn_act; z = par.zeta_act;
    for i = 1:4
        ip=2*i-1; iv=2*i;
        c=fin_cmd(i); x1=act(ip); x2=act(iv);
        [k11,k12]=discrete_sim_utils.act_deriv(x1,x2,c,wn,z,par);
        [k21,k22]=discrete_sim_utils.act_deriv(x1+dt/2*k11,x2+dt/2*k12,c,wn,z,par);
        [k31,k32]=discrete_sim_utils.act_deriv(x1+dt/2*k21,x2+dt/2*k22,c,wn,z,par);
        [k41,k42]=discrete_sim_utils.act_deriv(x1+dt*k31,x2+dt*k32,c,wn,z,par);
        act(ip)=x1+dt/6*(k11+2*k21+2*k31+k41);
        act(iv)=x2+dt/6*(k12+2*k22+2*k32+k42);
        act(ip)=max(-par.delta_max,min(par.delta_max,act(ip)));
        if isfield(par,'delta_rate_max')
            act(iv)=max(-par.delta_rate_max,min(par.delta_rate_max,act(iv))); end
    end
end

function [dx1,dx2] = act_deriv(x1,x2,cmd,wn,z,par)
    dx1 = x2; dx2 = wn^2*(cmd-x1)-2*z*wn*x2;
    if isfield(par,'delta_rate_max')
        dx1 = max(-par.delta_rate_max,min(par.delta_rate_max,dx1)); end
end

%% Plant RK4 step
function xn = rk4_plant_step(t, x, fin, T, m, par, dt, w_NED)
    k1 = discrete_sim_utils.plant_eom(t,      x,         fin,T,m,par, w_NED);
    k2 = discrete_sim_utils.plant_eom(t+dt/2, x+dt/2*k1, fin,T,m,par, w_NED);
    k3 = discrete_sim_utils.plant_eom(t+dt/2, x+dt/2*k2, fin,T,m,par, w_NED);
    k4 = discrete_sim_utils.plant_eom(t+dt,   x+dt*k3,   fin,T,m,par, w_NED);
    xn = x + dt/6*(k1+2*k2+2*k3+k4);
end

%% Plant EOM (13-state, with wind)
function xdot = plant_eom(t, x, fin, T, m, par, w_NED)
    xdot = zeros(13,1);
    pos=x(1:3); vel=x(4:6); quat=x(7:10); wb=x(11:13);
    p=wb(1); q=wb(2); r=wb(3);

    qn=norm(quat);
    if qn<1e-10, quat=[1;0;0;0]; else, quat=quat/qn; end
    q0=quat(1); q1=quat(2); q2=quat(3); q3=quat(4);

    R_BN=[1-2*(q2^2+q3^2), 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2);
          2*(q1*q2-q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3+q0*q1);
          2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), 1-2*(q1^2+q2^2)];

    alt = max(-pos(3),0);

    if nargin < 7 || isempty(w_NED)
        w_NED = discrete_sim_utils.get_wind(t, alt, par);
    end

    va = vel - w_NED;
    vb = R_BN*va;
    Va = max(norm(vb),1.0);
    alpha = atan2(vb(3), max(vb(1),0.1));
    beta  = asin(max(-1,min(1, vb(2)/Va)));

    rho = par.rho0*exp(-alt/par.h_scale);
    qbar = 0.5*rho*Va^2;

    de=(fin(1)-fin(3))/2; dr=(fin(2)-fin(4))/2;
    da=(fin(1)+fin(2)+fin(3)+fin(4))/4;

    Fx = -qbar*par.Sref*par.CA0;
    Fy =  qbar*par.Sref*(par.CYb*beta + par.CYdr*dr);
    Fz = -qbar*par.Sref*(par.CNa*alpha + par.CNd*de);
    Mp = qbar*par.Sref*par.d*(par.Cma*alpha + par.Cmd*de + par.Cmq*q*par.d/(2*Va));
    My = qbar*par.Sref*par.d*(par.Cnb*beta + par.Cndr*dr + par.Cnr*r*par.d/(2*Va));
    Mr = qbar*par.Sref*par.d*(par.Clp*p*par.d/(2*Va) + par.Clda*da);

    Fb = [Fx;Fy;Fz]+[T;0;0]+R_BN*[0;0;m*par.g];
    Mb = [Mr;Mp;My];

    xdot(1:3) = vel;
    xdot(4:6) = R_BN'*(Fb/m);
    I = [par.Ixx;par.Iyy;par.Izz];
    xdot(11:13) = (Mb - cross(wb,I.*wb))./I;
    Om = [0,-p,-q,-r; p,0,r,-q; q,-r,0,p; r,q,-p,0];
    xdot(7:10) = 0.5*Om*quat;
end

end % methods
end % classdef
