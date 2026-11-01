function report = run_demo(showFigure)
%RUN_DEMO Simulate, validate and export the DC motor speed-control example.
%   RUN_DEMO runs with a visible figure. RUN_DEMO(false) runs without a GUI.
if nargin == 0
    showFigure = true;
end
root = fileparts(mfilename('fullpath'));
oldFolder = pwd;
folderCleanup = onCleanup(@() cd(oldFolder));
cd(root);
p = motor_parameters();
model = 'dc_motor_speed_control';
in = Simulink.SimulationInput(model);
in = in.setVariable('p', p, 'Workspace', model);
in = in.setModelParameter('StopTime', num2str(p.stopTime), ...
    'Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-10', ...
    'MaxStep', '0.002', 'ReturnWorkspaceOutputs', 'on');
out = sim(in);
t = (0:p.sampleTime:p.stopTime).';
speed = interp1(out.speed.Time, out.speed.Data, t, 'linear');
current = interp1(out.current.Time, out.current.Data, t, 'linear');
voltage = interp1(out.voltage.Time, out.voltage.Data, t, 'linear');
reference = p.reference*(t >= p.referenceTime);
loadTorque = p.loadTorque*(t >= p.loadTime);
states = reference_solution(t, p);

% Same physical system without back-calculation: a directly comparable baseline.
pNoAw = p;
pNoAw.Kaw = 0;
in = in.setVariable('p', pNoAw, 'Workspace', model);
out = sim(in);
speedNoAw = interp1(out.speed.Time, out.speed.Data, t, 'linear');

report.matlab_release = version('-release');
report.ode45_max_speed_error_rad_s = max(abs(speed - states(:, 2)));
report.peak_speed_rad_s = max(speed);
report.overshoot_percent = max(0, (max(speed)/p.reference - 1)*100);
report.no_antiwindup_overshoot_percent = ...
    max(0, (max(speedNoAw)/p.reference - 1)*100);
report.final_speed_error_rad_s = abs(p.reference - speed(end));
report.final_current_A = current(end);
report.theoretical_steady_current_A = (p.b*p.reference + p.loadTorque)/p.Kt;
report.final_voltage_V = voltage(end);
report.theoretical_steady_voltage_V = ...
    p.R*report.theoretical_steady_current_A + p.Ke*p.reference;
report.max_abs_voltage_V = max(abs(voltage));
report.peak_abs_current_A = max(abs(current));
report.settling_band_percent = 2;
report.load_peak_speed_error_rad_s = ...
    max(abs(speed(t >= p.loadTime) - p.reference));
report.first_settling_time_s = settling_time(t, speed, p.reference, ...
    p.referenceTime, p.loadTime, 0.02);
report.load_recovery_time_s = settling_time(t, speed, p.reference, ...
    p.loadTime, p.stopTime, 0.02);

assert(all(isfinite([speed; current; voltage])), 'Nonfinite output.');
assert(report.max_abs_voltage_V <= p.voltageLimit + 1e-8, ...
    'Drive voltage exceeded its saturation bound.');
assert(report.ode45_max_speed_error_rad_s < 0.01, ...
    'Simulink and the independent ODE solution disagree.');
assert(report.final_speed_error_rad_s < 0.5, 'Speed did not converge.');
assert(abs(current(end) - report.theoretical_steady_current_A) < 0.05, ...
    'Steady-state torque balance failed.');
assert(abs(voltage(end) - report.theoretical_steady_voltage_V) < 0.1, ...
    'Steady-state electrical balance failed.');
assert(isfinite(report.first_settling_time_s) && ...
    isfinite(report.load_recovery_time_s), 'Response did not settle.');
report.verification = 'PASS';

results = fullfile(root, 'results');
if ~isfolder(results)
    mkdir(results);
end
data = table(t, reference, speed, speedNoAw, current, voltage, loadTorque, ...
    'VariableNames', {'time_s', 'reference_rad_s', 'speed_rad_s', ...
    'speed_without_antiwindup_rad_s', 'current_A', 'voltage_V', 'load_Nm'});
writetable(data, fullfile(results, 'response.csv'));
write_json(fullfile(results, 'metrics.json'), report);
write_json(fullfile(results, 'parameters.json'), p);

visibility = 'off';
if showFigure
    visibility = 'on';
end
fig = figure('Color', 'w', 'Visible', visibility, ...
    'Position', [100, 100, 1120, 760], 'Name', 'DC Motor Speed Control');
layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout);
plot(ax, t, reference, 'k:', 'LineWidth', 1.5);
hold(ax, 'on');
plot(ax, t, speed, 'Color', [0.05, 0.4, 0.7], 'LineWidth', 1.8);
plot(ax, t, speedNoAw, '--', 'Color', [0.85, 0.35, 0.1], 'LineWidth', 1.2);
ylabel(ax, 'Speed [rad/s]');
legend(ax, 'Request', 'PID + anti-windup', 'PID without anti-windup', ...
    'Location', 'best');
title(ax, 'Saturated 24 V drive; 0.12 N m load applied at 2 s');
grid(ax, 'on');
ax = nexttile(layout);
plot(ax, t, voltage, 'Color', [0.05, 0.4, 0.7], 'LineWidth', 1.6);
yline(ax, p.voltageLimit, 'k:', '+24 V');
yline(ax, -p.voltageLimit, 'k:', '-24 V');
ylabel(ax, 'Armature voltage [V]');
grid(ax, 'on');
ax = nexttile(layout);
yyaxis(ax, 'left');
plot(ax, t, current, 'LineWidth', 1.5);
ylabel(ax, 'Armature current [A]');
yyaxis(ax, 'right');
plot(ax, t, loadTorque, '--', 'LineWidth', 1.5);
ylabel(ax, 'Opposing load [N m]');
xlabel(ax, 'Time [s]');
grid(ax, 'on');
title(layout, 'DC Motor | Filtered PID Speed Control', 'FontWeight', 'bold');
exportgraphics(fig, fullfile(results, 'response.png'), 'Resolution', 160);
if ~showFigure
    close(fig);
end
disp(report);
end

function elapsed = settling_time(t, signal, target, startTime, endTime, band)
% Last departure from a target-relative band; excludes the next step time.
selected = find(t >= startTime & t < endTime);
outside = find(abs(signal(selected) - target) > band*abs(target), 1, 'last');
if isempty(outside)
    elapsed = 0;
elseif outside == numel(selected)
    elapsed = Inf;
else
    elapsed = t(selected(outside+1)) - startTime;
end
end

function write_json(path, value)
file = fopen(path, 'w');
assert(file ~= -1, 'Cannot write results file.');
fileCleanup = onCleanup(@() fclose(file));
fprintf(file, '%s\n', jsonencode(value, 'PrettyPrint', true));
end
