function report = run_demo(showFigures)
%RUN_DEMO Simulate, verify and export four focused DC motor figures.
%   RUN_DEMO(false) runs without opening figure windows.
if nargin == 0
    showFigures = true;
end
root = fileparts(mfilename('fullpath'));
oldFolder = pwd;
folderCleanup = onCleanup(@() cd(oldFolder));
cd(root);
p = motor_parameters();
windows = plan_figure_windows(p);
p.stopTime = windows.stopTime;

% Both controllers receive the same reference step AND load disturbance.
% An additional unloaded run isolates the effect of the shaft torque.
pNoLoad = p;
pNoLoad.loadTorque = 0;
nominal = simulate_motor(pNoLoad);
pNoAw = p;
pNoAw.Kaw = 0;
baseline = simulate_motor(pNoAw);
loaded = simulate_motor(p);
t = loaded.time_s;
reference = p.reference*(t >= p.referenceTime);
loadTorque = p.loadTorque*(t >= p.loadTime);
states = reference_solution(t, p);
loadDeviation = nominal.speed_rad_s - loaded.speed_rad_s;
peakDeviation = max(abs(loadDeviation(t >= p.loadTime)));

report.matlab_release = version('-release');
report.simulation_stop_time_s = p.stopTime;
report.ode45_max_speed_error_rad_s = max(abs(loaded.speed_rad_s - states(:, 2)));
report.overshoot_percent = max(0, (max(nominal.speed_rad_s)/p.reference - 1)*100);
report.no_antiwindup_overshoot_percent = ...
    max(0, (max(baseline.speed_rad_s)/p.reference - 1)*100);
report.settling_band_percent = 2;
report.first_settling_time_s = settling_time(t, ...
    loaded.speed_rad_s - p.reference, p.referenceTime, 0.02*p.reference, p.loadTime);
report.no_antiwindup_settling_time_s = settling_time(t, ...
    baseline.speed_rad_s - p.reference, p.referenceTime, 0.02*p.reference, p.loadTime);
report.load_time_s = p.loadTime;
report.load_torque_Nm = p.loadTorque;
report.load_peak_speed_deviation_rad_s = peakDeviation;
report.load_recovery_time_s = settling_time(t, ...
    loaded.speed_rad_s - p.reference, p.loadTime, 0.02*p.reference);
report.load_disturbance_settling_band_rad_s = ...
    0.02*max(abs(loaded.speed_rad_s(t >= p.loadTime) - p.reference));
report.load_disturbance_settling_time_s = settling_time(t, ...
    loaded.speed_rad_s - p.reference, p.loadTime, ...
    report.load_disturbance_settling_band_rad_s);
report.post_load_settling_with_antiwindup_s = ...
    windows.post_load_settling_with_antiwindup_s;
report.post_load_settling_without_antiwindup_s = ...
    windows.post_load_settling_without_antiwindup_s;
report.no_antiwindup_load_settling_time_s = settling_time(t, ...
    baseline.speed_rad_s - p.reference, p.loadTime, ...
    0.02*max(abs(baseline.speed_rad_s(t >= p.loadTime) - p.reference)));
report.final_speed_error_rad_s = abs(p.reference - loaded.speed_rad_s(end));
report.final_current_A = loaded.current_A(end);
report.theoretical_steady_current_A = (p.b*p.reference + p.loadTorque)/p.Kt;
report.final_voltage_V = loaded.voltage_V(end);
report.theoretical_steady_voltage_V = ...
    p.R*report.theoretical_steady_current_A + p.Ke*p.reference;
report.max_abs_voltage_V = max(abs(loaded.voltage_V));
report.peak_abs_current_A = max(abs(loaded.current_A));

assert(all(isfinite(loaded{:, :}), 'all'), 'Nonfinite simulation output.');
assert(report.max_abs_voltage_V <= p.voltageLimit + 1e-8, 'Voltage bound failed.');
assert(report.ode45_max_speed_error_rad_s < 0.01, 'ODE verification failed.');
assert(report.final_speed_error_rad_s < 0.5, 'Speed did not converge.');
assert(abs(report.final_current_A - report.theoretical_steady_current_A) ...
    < 0.05, 'Steady-state torque balance failed.');
assert(abs(report.final_voltage_V - report.theoretical_steady_voltage_V) ...
    < 0.1, 'Steady-state electrical balance failed.');
assert(all(isfinite([report.first_settling_time_s, ...
    report.no_antiwindup_settling_time_s, ...
    report.load_disturbance_settling_time_s])), 'A response did not settle.');
assert(windows.trackingEnd > p.referenceTime + ...
    report.no_antiwindup_settling_time_s, 'Tracking plot truncates settling.');
assert(windows.loadEnd > p.loadTime + ...
    report.load_disturbance_settling_time_s, 'Load plot truncates recovery.');
assert(windows.trackingEnd > p.loadTime + ...
    max(report.load_disturbance_settling_time_s, ...
    report.no_antiwindup_load_settling_time_s), ...
    'Main comparison truncates post-load settling.');
report.verification = 'PASS';

tracking = table(t, reference, loaded.speed_rad_s, baseline.speed_rad_s, ...
    loaded.current_A, loaded.voltage_V, baseline.current_A, ...
    baseline.voltage_V, loadTorque, 'VariableNames', ...
    {'time_s', 'reference_rad_s', 'speed_rad_s', ...
    'speed_without_antiwindup_rad_s', 'current_A', 'voltage_V', ...
    'current_without_antiwindup_A', 'voltage_without_antiwindup_V', 'load_Nm'});
loadData = table(t, loaded.speed_rad_s, nominal.speed_rad_s, ...
    loaded.current_A, nominal.current_A, loaded.voltage_V, ...
    nominal.voltage_V, loadTorque, 'VariableNames', ...
    {'time_s', 'speed_rad_s', 'unloaded_speed_rad_s', 'current_A', ...
    'unloaded_current_A', 'voltage_V', 'unloaded_voltage_V', 'load_Nm'});
results = fullfile(root, 'results');
if ~isfolder(results)
    mkdir(results);
end
writetable(tracking, fullfile(results, 'tracking.csv'));
writetable(loadData, fullfile(results, 'load_response.csv'));
write_json(fullfile(results, 'metrics.json'), report);
write_json(fullfile(results, 'parameters.json'), p);
write_json(fullfile(results, 'figure_windows.json'), windows);
plot_results(tracking, loadData, p, windows, report, results, showFigures);
disp(report);
end

function data = simulate_motor(p)
model = 'dc_motor_speed_control';
in = Simulink.SimulationInput(model);
in = in.setVariable('p', p, 'Workspace', model);
in = in.setModelParameter('StopTime', num2str(p.stopTime, 12), ...
    'Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-10', ...
    'MaxStep', '0.002', 'ReturnWorkspaceOutputs', 'on');
out = sim(in);
t = (0:p.sampleTime:p.stopTime).';
speed = interp1(out.speed.Time, out.speed.Data, t, 'linear');
current = interp1(out.current.Time, out.current.Data, t, 'linear');
voltage = interp1(out.voltage.Time, out.voltage.Data, t, 'linear');
data = table(t, speed, current, voltage, 'VariableNames', ...
    {'time_s', 'speed_rad_s', 'current_A', 'voltage_V'});
end

function elapsed = settling_time(t, deviation, startTime, tolerance, endTime)
if nargin < 5
    endTime = Inf;
end
selected = find(t >= startTime & t < endTime);
last = find(abs(deviation(selected)) > tolerance, 1, 'last');
if isempty(last)
    elapsed = 0;
elseif last == numel(selected)
    elapsed = Inf;
else
    elapsed = t(selected(last+1)) - startTime;
end
end

function write_json(path, value)
file = fopen(path, 'w');
assert(file ~= -1, 'Cannot write results file.');
fileCleanup = onCleanup(@() fclose(file));
fprintf(file, '%s\n', jsonencode(value, 'PrettyPrint', true));
end
