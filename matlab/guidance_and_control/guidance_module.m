function [nz_cmd, ny_cmd, xdot_guid] = guidance_module(t, nav_pos, nav_vel, R_BN, qbar, m, par, x_guid, wind_est_override)
%GUIDANCE_MODULE  Compute nz/ny acceleration commands.
%  par.guid_law = 'pp' | 'augmented_pp' (default) | 'pn_approx'
%  Wind Drift Lead: offsets LOS by -(t_go * wind_est) if wind_est ~= 0.

nz_cmd = 0;  ny_cmd = 0;
xdot_guid = zeros(3,1);

if qbar <= par.qbar_min_ctrl || t <= par.t_guide_on
    return;
end

los_NED = par.target_NED - nav_pos;
R_tgt   = norm(los_NED);
if R_tgt <= 2.0, return; end

% Wind Drift Lead
if nargin >= 9 && ~isempty(wind_est_override)
    wind_est = wind_est_override(:);
else
    wind_est = par.wind_est;
end

if norm(wind_est) > 0.01
    t_go = min(R_tgt / max(norm(nav_vel), 1.0), 10.0);
    wind_offset = wind_est * t_go;

    if norm(wind_offset) < 0.5 * R_tgt
        los_corrected = los_NED - wind_offset;
        R_corr = norm(los_corrected);
        if R_corr > 2.0
            los_NED = los_corrected;
            R_tgt   = R_corr;
        end
    end
end

V_mag = max(norm(nav_vel), 1.0);
u_V   = nav_vel / V_mag;
u_LOS = los_NED / R_tgt;

v_closing = dot(los_NED, nav_vel) / R_tgt;

if v_closing < 0 && t > par.t_guide_on + 1.0
    nz_cmd = 0;
    ny_cmd = 0;
    return;
else
    switch lower(par.guid_law)
        case 'pp'
            a_cmd = guid_pp(V_mag, u_V, u_LOS, R_tgt, par);
        case 'augmented pp'
            a_cmd = guid_augmented_pp(V_mag, u_V, u_LOS, R_tgt, los_NED, par);
        case 'pn_approx'
            [a_cmd, xdot_guid] = guid_pn(V_mag, u_V, u_LOS, los_NED, R_tgt, nav_vel, par, x_guid);
        otherwise
            a_cmd = guid_augmented_pp(V_mag, u_V, u_LOS, R_tgt, los_NED, par);
    end
end

% Gravity compensation -> body frame -> nz/ny
a_need = R_BN * (a_cmd - [0;0;par.g]);
nz_cmd = -a_need(3) / par.g;
ny_cmd =  a_need(2) / par.g;

% Command saturation (vector magnitude preserving direction)
if isfield(par, 'roll_alloc_max') && par.roll_alloc_max > 0
    delta_avail = max(par.delta_max - par.roll_alloc_max, 0.1*par.delta_max);
else
    delta_avail = par.delta_max;
end
nz_max = qbar * par.Sref * par.CNd * delta_avail / (m * par.g);
cmd_mag = sqrt(nz_cmd^2 + ny_cmd^2);
if cmd_mag > nz_max && cmd_mag > 1e-6
    scale = nz_max / cmd_mag;
    nz_cmd = nz_cmd * scale;
    ny_cmd = ny_cmd * scale;
end

% Ramp-up
dt = t - par.t_guide_on;
if dt < 0.5
    ramp = dt / 0.5;
    nz_cmd = nz_cmd * ramp;
    ny_cmd = ny_cmd * ramp;
end

end

%% Pure Pursuit
function a_cmd = guid_pp(V_mag, u_V, u_LOS, R_tgt, par)
    K = par.K_guid;
    if R_tgt < par.R_terminal_slant, K = K * 1.5; end
    a_cmd = K * V_mag * (u_LOS - u_V);
end

%% Augmented Pure Pursuit (gain scheduling + gravity lead + terminal boost)
function a_cmd = guid_augmented_pp(V_mag, u_V, u_LOS, R_tgt, los_NED, par)

    progress = max(0, min(1, 1 - R_tgt / par.R_gain_sched));
    K_guid = par.K_guid_far + (par.K_guid_near - par.K_guid_far) * progress;

    % Gravity lead
    t_go = R_tgt / max(V_mag, 1);
    gamma = asin(max(-1, min(1, -u_V(3))));
    grav_drop = [0; 0; 0.5 * par.g * t_go^2 * cos(gamma)];
    los_lead = los_NED + grav_drop;
    los_lead_n = norm(los_lead);

    if los_lead_n > 1
        u_LOS_lead = los_lead / los_lead_n;
        a_lead = K_guid * V_mag * (u_LOS_lead - u_V);
        lead_w = max(0, min(1, R_tgt / par.R_gain_sched));
        a_cmd = lead_w * a_lead + (1 - lead_w) * K_guid * V_mag * (u_LOS - u_V);
    else
        a_cmd = K_guid * V_mag * (u_LOS - u_V);
    end

    % Terminal boost
    if R_tgt < par.R_terminal_slant
        blend = 0.5 * (1 - cos(pi * (1 - R_tgt / par.R_terminal_slant)));
        a_terminal = K_guid * 1.5 * V_mag * (u_LOS - u_V);
        a_cmd = (1 - blend) * a_cmd + blend * a_terminal;
    end
end

%% Proportional Navigation (approximate)
function [a_cmd, xdot_guid] = guid_pn(~, ~, u_LOS, los_NED, R_tgt, nav_vel, par, x_guid)
    omega_LOS_raw = cross(los_NED, -nav_vel) / (R_tgt^2);
    omega_filt = x_guid(1:3);
    xdot_guid = (omega_LOS_raw - omega_filt) / par.pn_filter_tau;

    v_closing = dot(los_NED, nav_vel) / R_tgt;
    a_cmd = par.N_PN * max(v_closing, 1.0) * cross(omega_filt, u_LOS);
end
