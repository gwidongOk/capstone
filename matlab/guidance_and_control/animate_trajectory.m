function animate_trajectory(t, X, par, playback_speed)
%ANIMATE_TRAJECTORY  3D trajectory animation with body axes.

if nargin < 3
    error('Usage: animate_trajectory(t, X, par [, playback_speed])');
end
if nargin < 4
    playback_speed = 1.0;
end

pos = X(:,1:3);
quat_h = X(:,7:10);
alt = -pos(:,3);
N = length(t);

arrow_len  = 15;
target_fps = 60;
frame_skip = max(1, round(N / (target_fps * (t(end)-t(1)) / playback_speed)));
view_az = -45; view_el = 30;

fig = figure('Name','Trajectory Animation','Position',[100 50 1000 750], ...
             'Color','w', 'Renderer','opengl');

pad = 20;
x_lim = [min(pos(:,1))-pad, max(pos(:,1))+pad];
y_lim = [min(pos(:,2))-pad, max(pos(:,2))+pad];
z_lim = [min(alt)-pad,      max(alt)+pad];

hold on; grid on;
plot3(par.target_NED(1), par.target_NED(2), -par.target_NED(3), ...
      'rp', 'MarkerSize', 18, 'MarkerFaceColor', 'r');
plot3(0, 0, 0, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');

h_trail = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 1.2);

h_qx = quiver3(0,0,0, 0,0,0, 'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
h_qy = quiver3(0,0,0, 0,0,0, 'g', 'LineWidth', 2.0, 'MaxHeadSize', 0.5);
h_qz = quiver3(0,0,0, 0,0,0, 'b', 'LineWidth', 2.0, 'MaxHeadSize', 0.5);

h_rocket = plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
title('Trajectory Animation');
legend('Target','Launch','Trajectory','x_{body}','y_{body}','z_{body}','Rocket', ...
       'Location','northeast');
xlim(x_lim); ylim(y_lim); zlim(z_lim);
view(view_az, view_el);
axis equal;

frame_indices = 1:frame_skip:N;
if frame_indices(end) ~= N
    frame_indices(end+1) = N;
end

fprintf('Animation: %.1fs flight, %d frames, %.1f× speed\n', ...
    t(end)-t(1), length(frame_indices), playback_speed);

t_wall_start = tic;
t_sim_start = t(1);

for fi = 1:length(frame_indices)
    if ~isvalid(fig), break; end

    k = frame_indices(fi);

    px = pos(k,1); py = pos(k,2); pz = alt(k);

    qq = quat_h(k,:)' / norm(quat_h(k,:));
    R_BN = gnc_utils.quat2dcm_bn(qq);
    R_NB = R_BN';

    x_body = R_NB(:,1);
    y_body = R_NB(:,2);
    z_body = R_NB(:,3);

    bx_plot = [x_body(1), x_body(2), -x_body(3)] * arrow_len;
    by_plot = [y_body(1), y_body(2), -y_body(3)] * arrow_len;
    bz_plot = [z_body(1), z_body(2), -z_body(3)] * arrow_len;

    trail_idx = 1:k;
    set(h_trail, 'XData', pos(trail_idx,1), ...
                 'YData', pos(trail_idx,2), ...
                 'ZData', alt(trail_idx));

    set(h_rocket, 'XData', px, 'YData', py, 'ZData', pz);

    set(h_qx, 'XData',px, 'YData',py, 'ZData',pz, ...
              'UData',bx_plot(1), 'VData',bx_plot(2), 'WData',bx_plot(3));
    set(h_qy, 'XData',px, 'YData',py, 'ZData',pz, ...
              'UData',by_plot(1), 'VData',by_plot(2), 'WData',by_plot(3));
    set(h_qz, 'XData',px, 'YData',py, 'ZData',pz, ...
              'UData',bz_plot(1), 'VData',bz_plot(2), 'WData',bz_plot(3));

    title(sprintf('t = %.2f s  |  V = %.0f m/s  |  Alt = %.0f m  [%.1f×]', ...
          t(k), norm(X(k,4:6)), pz, playback_speed));

    drawnow limitrate;

    % Wall-clock synchronization
    t_sim_elapsed = (t(k) - t_sim_start) / playback_speed;
    t_wall_elapsed = toc(t_wall_start);
    wait = t_sim_elapsed - t_wall_elapsed;
    if wait > 0.001
        pause(wait);
    end
end

slant = vecnorm(pos - par.target_NED', 2, 2);
[min_slant, cpa_idx] = min(slant);

title(sprintf('DONE: t = %.2f s  |  Min Miss = %.1f m (at t=%.2fs)', ...
      t(end), min_slant, t(cpa_idx)));
fprintf('Animation complete. Min miss: %.1f m.\n', min_slant);
end
