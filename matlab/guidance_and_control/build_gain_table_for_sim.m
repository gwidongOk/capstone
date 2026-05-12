function par_out = build_gain_table_for_sim(par, method)
%BUILD_GAIN_TABLE_FOR_SIM  Design gains at each velocity breakpoint.

    V_bp  = par.sched_V(:)';
    N_pts = length(V_bp);
    mid   = ceil(N_pts/2);

    par_out = par;
    gt = struct();

    for i = 1:N_pts
        par_i = par;
        par_i.design_V = V_bp(i);
        par_i.gain_method = method;
        p = design_gains(par_i);

        gt.KR(i)      = p.KR;
        gt.wI(i)      = p.wI;
        gt.KA(i)      = p.KA;
        gt.KDC(i)     = p.KDC;
        gt.K_phi(i)   = p.K_phi;
        gt.Kp_roll(i) = p.Kp_roll;
        gt.Ki_roll(i) = p.Ki_roll;

        if i == mid
            par_out.design_info = p.design_info;
        end
    end

    par_out.gain_table.V_bp    = V_bp;
    par_out.gain_table.KR      = gt.KR;
    par_out.gain_table.wI      = gt.wI;
    par_out.gain_table.KA      = gt.KA;
    par_out.gain_table.KDC     = gt.KDC;
    par_out.gain_table.K_phi   = gt.K_phi;
    par_out.gain_table.Kp_roll = gt.Kp_roll;
    par_out.gain_table.Ki_roll = gt.Ki_roll;

    par_out.KR  = gt.KR(mid);   par_out.wI = gt.wI(mid); par_out.KA  = gt.KA(mid);   par_out.KDC = gt.KDC(mid);
    par_out.K_phi   = gt.K_phi(mid);
    par_out.Kp_roll = gt.Kp_roll(mid);
    par_out.Ki_roll = gt.Ki_roll(mid);
    par_out.gain_method = method;
end
