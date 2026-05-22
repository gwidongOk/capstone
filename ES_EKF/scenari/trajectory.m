function [t, p_true, v_true, q_true, f_body_true, w_body_true, dt, info] = trajectory(case_name)
%TRAJECTORY  Compatibility wrapper for trajectory_base.

    if nargin < 1 || isempty(case_name)
        case_name = 'nominal';
    end

    truth = trajectory_base(case_name);
    t = truth.t;
    p_true = truth.p;
    v_true = truth.v;
    q_true = truth.q;
    f_body_true = truth.f_body;
    w_body_true = truth.w_body;
    dt = truth.dt;
    info = truth.info;
end
