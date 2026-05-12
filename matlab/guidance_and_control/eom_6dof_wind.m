function xdot = eom_6dof_wind(t, x, par)
%EOM_6DOF_WIND  6-DOF EOM wrapper with wind models.
%  par.wind_model: 'none' | 'constant' | 'powerlaw' | 'powerlaw_dryden'

% Backward compatibility
if ~isfield(par, 'wind_model') || isempty(par.wind_model)
    if isfield(par, 'wind_NED') && ~isempty(par.wind_NED) && norm(par.wind_NED) > 0
        par.wind_model = 'constant';
    else
        par.wind_model = 'none';
    end
end

% Wind vector computation
switch lower(par.wind_model)
    case 'none'
        wind_true = [0; 0; 0];

    case 'constant'
        if isfield(par, 'wind_NED') && ~isempty(par.wind_NED)
            wind_true = par.wind_NED(:);
        else
            wind_true = [0; 0; 0];
        end

    case 'powerlaw'
        h_eff = max(-x(3), par.wind_h_min);
        U_h   = par.wind_ref_speed * ...
                (h_eff / par.wind_ref_height)^par.wind_alpha;

        wind_true = -U_h * [cos(par.wind_direction);
                            sin(par.wind_direction);
                            0];

    case 'powerlaw_dryden'
        h_eff = max(-x(3), par.wind_h_min);
        U_h   = par.wind_ref_speed * ...
                (h_eff / par.wind_ref_height)^par.wind_alpha;
        wind_mean = -U_h * [cos(par.wind_direction);
                            sin(par.wind_direction);
                            0];

        t_clamped = max(par.gust_t(1), min(par.gust_t(end), t));
        gust = [par.gust_interp_N(t_clamped);
                par.gust_interp_E(t_clamped);
                par.gust_interp_D(t_clamped)];

        wind_true = wind_mean + gust;
    otherwise
        error('eom_6dof_wind: unknown wind_model "%s"', par.wind_model);
end

% Delegate to eom_6dof
if norm(wind_true) > 0
    vel_ground = x(4:6);

    x_air = x;
    x_air(4:6) = vel_ground - wind_true;

    if isfield(par, 'wind_est') && ~isempty(par.wind_est)
        wind_est = par.wind_est(:);
    else
        wind_est = wind_true;
    end

    xdot = eom_6dof(t, x_air, par, vel_ground, wind_est);
    xdot(1:3) = vel_ground;
else
    xdot = eom_6dof(t, x, par);
end

end
