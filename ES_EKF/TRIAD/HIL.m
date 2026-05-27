clear; clc; close all;

port = "COM8";
baud = 115200;

if strlength(port) == 0
    ports = detectSerialPorts();
    if numel(ports) == 1
        port = string(ports{1});
    else
        if ~isempty(ports)
            fprintf("Available ports:\n");
            fprintf("  %s\n", ports{:});
        end
        port = string(input("Serial port (e.g. COM7): ", "s"));
        if strlength(port) == 0
            error("Serial port required.");
        end
    end
end

sp = serialport(port, baud, "Timeout", 0.1);
configureTerminator(sp, "LF");
flush(sp);
guard = onCleanup(@() stopStream(sp));

fig = figure("Name", "TRIAD HIL", "Color", "w");
ax = axes(fig);
hold(ax, "on");
grid(ax, "on");
axis(ax, "equal");
axis(ax, [-1.2 1.2 -1.2 1.2 -1.2 1.2]);
view(ax, 35, 25);
set(ax, "ZDir", "reverse");
xlabel(ax, "N");
ylabel(ax, "E");
zlabel(ax, "D");

quiver3(ax, 0,0,0, 1,0,0, 0, "Color", [0.2 0.2 0.2], "LineWidth", 1);
quiver3(ax, 0,0,0, 0,1,0, 0, "Color", [0.2 0.2 0.2], "LineWidth", 1);
quiver3(ax, 0,0,0, 0,0,1, 0, "Color", [0.2 0.2 0.2], "LineWidth", 1);
text(ax, 1.05, 0, 0, "N");
text(ax, 0, 1.05, 0, "E");
text(ax, 0, 0, 1.05, "D");
text(ax, -1.10, 0, 0, "S", "Color", [0.45 0.45 0.45]);
text(ax, 0, -1.10, 0, "W", "Color", [0.45 0.45 0.45]);
text(ax, 0, 0, -1.10, "U", "Color", [0.45 0.45 0.45]);

hBx = quiver3(ax, 0,0,0, 1,0,0, 0, "r", "LineWidth", 3);
hBy = quiver3(ax, 0,0,0, 0,1,0, 0, "g", "LineWidth", 3);
hBz = quiver3(ax, 0,0,0, 0,0,1, 0, "b", "LineWidth", 3);
tBx = text(ax, 1.08, 0, 0, "body X", "Color", "r", "FontWeight", "bold");
tBy = text(ax, 0, 1.08, 0, "body Y", "Color", "g", "FontWeight", "bold");
tBz = text(ax, 0, 0, 1.08, "body Z", "Color", "b", "FontWeight", "bold");
hInfo = annotation(fig, "textbox", [0.02 0.02 0.48 0.18], ...
    "String", "", "FitBoxToText", "off", "EdgeColor", [0.8 0.8 0.8], ...
    "BackgroundColor", "w", "FontName", "Consolas");

title(ax, "Waiting for TRIAD stream");
writeline(sp, "stream");
last_rx = tic;

while ishandle(fig)
    drawnow;

    if sp.NumBytesAvailable == 0
        if toc(last_rx) > 3
            title(ax, "Waiting for TRIAD stream");
        end
        pause(0.01);
        continue;
    end

    try
        line = strtrim(readline(sp));
    catch
        pause(0.01);
        continue;
    end

    if ~startsWith(line, "TRIAD,")
        continue;
    end
    last_rx = tic;

    parts = split(line, ",");
    if numel(parts) ~= 15
        continue;
    end

    v = str2double(parts(2:end));
    if any(isnan(v))
        continue;
    end

    t_ms = v(1);
    q = v(2:5);
    acc = v(6:8);
    mag = v(9:11);
    parallel = v(12);
    acc_norm_g = v(13);
    mag_norm = v(14);

    R_bn = quatToDcm(q);
    bx = R_bn(:,1);
    by = R_bn(:,2);
    bz = R_bn(:,3);

    set(hBx, "UData", bx(1), "VData", bx(2), "WData", bx(3));
    set(hBy, "UData", by(1), "VData", by(2), "WData", by(3));
    set(hBz, "UData", bz(1), "VData", bz(2), "WData", bz(3));

    set(tBx, "Position", 1.08*bx.');
    set(tBy, "Position", 1.08*by.');
    set(tBz, "Position", 1.08*bz.');

    tilt_deg = acosd(max(-1, min(1, -bx(3))));
    title(ax, sprintf("t=%.1fs  tilt=%.2f deg  acc=%.3fg  mag=%.3fG  parallel=%.3f", ...
        t_ms*1e-3, tilt_deg, acc_norm_g, mag_norm, parallel));
    infoText = sprintf(['Body axes in NED [N E D]\n' ...
        'X: [%+.3f %+.3f %+.3f]  %s\n' ...
        'Y: [%+.3f %+.3f %+.3f]  %s\n' ...
        'Z: [%+.3f %+.3f %+.3f]  %s'], ...
        bx(1), bx(2), bx(3), char(axisName(bx)), ...
        by(1), by(2), by(3), char(axisName(by)), ...
        bz(1), bz(2), bz(3), char(axisName(bz)));
    set(hInfo, "String", infoText);
    fprintf("X=%s [%+.3f %+.3f %+.3f]  Y=%s [%+.3f %+.3f %+.3f]  Z=%s [%+.3f %+.3f %+.3f]\n", ...
        char(axisName(bx)), bx(1), bx(2), bx(3), ...
        char(axisName(by)), by(1), by(2), by(3), ...
        char(axisName(bz)), bz(1), bz(2), bz(3));

    drawnow limitrate;
end

function R = quatToDcm(q)
q = q(:) / norm(q);
w = q(1); x = q(2); y = q(3); z = q(4);
R = [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y);
       2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x);
       2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)];
end

function stopStream(sp)
try
    writeline(sp, "stop");
catch
end
end

function name = axisName(v)
[~, idx] = max(abs(v));
names = {'N', 'E', 'D'; 'S', 'W', 'U'};
if v(idx) >= 0
    row = 1;
else
    row = 2;
end
name = names{row, idx};
end

function ports = detectSerialPorts()
ports = {};

try
    raw = serialportlist("available");
    ports = cellstr(raw);
    return;
catch
end

try
    raw = serialportlist;
    ports = cellstr(raw);
    return;
catch
end

try
    info = instrhwinfo('serial');
    ports = info.AvailableSerialPorts;
catch
end
end
