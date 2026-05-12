function [t_gust, gust_NED, varargout] = generate_dryden_gust(par, t_end, dt, rng_seed)
%GENERATE_DRYDEN_GUST  MIL-HDBK-1797 Dryden turbulence time series.
%
%  Forming filters driven by white noise produce correlated NED gust components.
%  Low-altitude model (h < 1000 ft) with scale lengths and intensities
%  per MIL-F-8785C.

if nargin < 3 || isempty(dt),       dt = 0.01;  end
if nargin >= 4 && ~isempty(rng_seed), rng(rng_seed); end

W20 = par.dryden_W20;

if isfield(par, 'dryden_h_ref')
    h = par.dryden_h_ref;
else
    h = 500;
end

if isfield(par, 'dryden_V_ref')
    V = par.dryden_V_ref;
else
    V = 100;
end

h = max(h, 10);

% Scale lengths
if h < 304.8
    L_u = h / (0.177 + 0.000823*h)^1.2;
    L_v = h / 2;
    L_w = h / 2;
else
    L_u = 533.4;
    L_v = 533.4;
    L_w = 533.4;
end

% Intensities
sigma_w = 0.1 * W20;
sigma_u = sigma_w / (0.177 + 0.000823*h)^0.4;
sigma_v = sigma_u;

% Forming filters
K_u = sigma_u * sqrt(2*V / L_u);
a_u = V / L_u;
sys_u = tf(K_u, [1, a_u]);

K_v = sigma_v * sqrt(3*V / L_v);
a_v = V / L_v;
b_v = V / (sqrt(3) * L_v);
sys_v = tf(K_v * [1, b_v], conv([1, a_v], [1, a_v]));

K_w = sigma_w * sqrt(3*V / L_w);
a_w = V / L_w;
b_w = V / (sqrt(3) * L_w);
sys_w = tf(K_w * [1, b_w], conv([1, a_w], [1, a_w]));

% White noise and filter
t_gust = (0 : dt : t_end)';
N = length(t_gust);

noise_u = randn(N, 1) / sqrt(dt);
noise_v = randn(N, 1) / sqrt(dt);
noise_w = randn(N, 1) / sqrt(dt);

gust_u = lsim(sys_u, noise_u, t_gust);
gust_v = lsim(sys_v, noise_v, t_gust);
gust_w = lsim(sys_w, noise_w, t_gust);

% Rotate horizontal components by wind direction if specified
if isfield(par, 'wind_direction')
    dir = par.wind_direction;
    cos_d = cos(dir);  sin_d = sin(dir);
    gust_N = -gust_u * cos_d - gust_v * sin_d;
    gust_E = -gust_u * sin_d + gust_v * cos_d;
else
    gust_N = gust_u;
    gust_E = gust_v;
end
gust_D = gust_w;

gust_NED = [gust_N, gust_E, gust_D];

% Fast interpolants for ODE use
interp_N = griddedInterpolant(t_gust, gust_N, 'linear', 'nearest');
interp_E = griddedInterpolant(t_gust, gust_E, 'linear', 'nearest');
interp_D = griddedInterpolant(t_gust, gust_D, 'linear', 'nearest');

if nargout >= 3
    varargout{1} = struct('N', interp_N, 'E', interp_E, 'D', interp_D);
end

end
