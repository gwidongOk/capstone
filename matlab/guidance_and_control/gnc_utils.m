classdef gnc_utils
%GNC_UTILS  Static utility functions for quaternion/DCM and flight events.

    methods (Static)

        function q = eul2quat_zyx(phi, theta, psi)
        %EUL2QUAT_ZYX  ZYX Euler -> quaternion [q0 q1 q2 q3].
            q_row = angle2quat(psi, theta, phi, 'ZYX');
            q = q_row(:);
        end

        function eul = quat2eul_zyx(q)
            [psi, theta, phi] = quat2angle(q(:)', 'ZYX');
            eul = [phi, theta, psi];
        end

        function R = quat2dcm_bn(q)
            R = quat2dcm(q(:)');
        end

        function [val, isterm, dir] = flight_events(t, x, par)
        %FLIGHT_EVENTS  ODE event functions: CPA, Apogee, Ground.
            alt = -x(3);
            vD  = x(6);

            if nargin < 3 || ~isfield(par, 'target_NED')
                if t < 3.0, val = [1; 1];
                else
                    val = [vD; alt];
                end

                isterm = [1; 1];
                dir    = [1; -1];
                return;
            end

            % Closing velocity
            pos = x(1:3);  vel = x(4:6);
            r_vec = pos - par.target_NED;
            R = norm(r_vec);
            if R > 0.1
                v_close = dot(r_vec, vel) / R;
            else
                v_close = 0;
            end

            if t < par.t_guide_on
                cpa_val = 1;
            else
                cpa_val = v_close;
            end

            % Disable CPA re-detection in Phase 2
            if isfield(par, 'phase') && par.phase == 2
                cpa_val = 1;
            end

            if t < 3.0
                apo_val = 1;
            else
                apo_val = vD;
            end

            val    = [cpa_val;   apo_val;  alt];
            isterm = [1;         1;        1];
            dir    = [1;         1;       -1];
        end

    end
end
