function plot_results(tracking, loaded, p, windows, report, folder, showFigures)
%PLOT_RESULTS Four focused figures exported as PNG previews and vector PDFs.
% Authoritative data: results/tracking.csv and results/load_response.csv.
blue = [0.00, 0.36, 0.60];
orange = [0.80, 0.32, 0.05];
gray = [0.38, 0.41, 0.45];
disturbanceColor = [0.45, 0.20, 0.55];

% Figure 1: complete test sequence, including settling AFTER the shaft load.
[fig, layout] = new_figure('Reference tracking', 1, showFigures);
ax = nexttile(layout);
hold(ax, 'on');
patch(ax, [p.referenceTime, windows.trackingEnd, ...
    windows.trackingEnd, p.referenceTime], ...
    p.reference*[0.98, 0.98, 1.02, 1.02], [0.86, 0.91, 0.94], ...
    'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(ax, tracking.time_s, tracking.reference_rad_s, ':', ...
    'Color', gray, 'LineWidth', 1.4, 'DisplayName', 'Reference');
plot(ax, tracking.time_s, tracking.speed_rad_s, ...
    'Color', blue, 'LineWidth', 2, 'DisplayName', 'With anti-windup');
plot(ax, tracking.time_s, tracking.speed_without_antiwindup_rad_s, '--', ...
    'Color', orange, 'LineWidth', 1.7, 'DisplayName', 'Without anti-windup');
xline(ax, p.referenceTime, ':', 'Color', gray, 'HandleVisibility', 'off');
xline(ax, p.loadTime, '--', 'Load applied', 'Color', disturbanceColor, ...
    'LabelOrientation', 'horizontal', 'FontSize', 10, 'HandleVisibility', 'off');
xlim(ax, [0, windows.trackingEnd]);
ylim(ax, [0, 1.12*max(tracking.speed_without_antiwindup_rad_s)]);
ylabel(ax, 'Angular speed, \omega [rad/s]');
xlabel(ax, 'Time [s]');
legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_axes(ax);
save_figure(fig, folder, '01_reference_tracking', showFigures);

% Figure 2: both controllers' complete voltage/current responses to both events.
[fig, layout] = new_figure('Control effort throughout the test', 2, showFigures);
selected = tracking.time_s <= windows.controlEnd;
t = tracking.time_s(selected);
ax = nexttile(layout);
hold(ax, 'on');
plot(ax, t, tracking.voltage_V(selected), 'Color', blue, 'LineWidth', 2, ...
    'DisplayName', 'With anti-windup');
plot(ax, t, tracking.voltage_without_antiwindup_V(selected), '--', ...
    'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'Without anti-windup');
yline(ax, p.voltageLimit, ':', 'Upper drive limit (24 V)', ...
    'Color', gray, 'LabelHorizontalAlignment', 'right', 'FontSize', 10, ...
    'HandleVisibility', 'off');
xline(ax, p.referenceTime, '--', 'Reference step', ...
    'Color', gray, 'LabelOrientation', 'horizontal', 'FontSize', 10, ...
    'HandleVisibility', 'off');
xline(ax, p.loadTime, '--', 'Load applied', 'Color', disturbanceColor, ...
    'LabelOrientation', 'horizontal', 'FontSize', 10, 'HandleVisibility', 'off');
ylabel(ax, 'Armature voltage, V [V]');
pad_limits(ax, [tracking.voltage_V(selected); ...
    tracking.voltage_without_antiwindup_V(selected); p.voltageLimit], true);
title(ax, '(a) Controller output', 'FontWeight', 'normal');
xlim(ax, [0, windows.controlEnd]);
legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_axes(ax);
ax = nexttile(layout);
hold(ax, 'on');
plot(ax, t, tracking.current_A(selected), 'Color', blue, 'LineWidth', 2);
plot(ax, t, tracking.current_without_antiwindup_A(selected), '--', ...
    'Color', orange, 'LineWidth', 1.5);
xline(ax, p.referenceTime, '--', 'Color', gray);
xline(ax, p.loadTime, '--', 'Color', disturbanceColor);
ylabel(ax, 'Armature current, i [A]');
xlabel(ax, 'Time [s]');
title(ax, '(b) Motor current', 'FontWeight', 'normal');
xlim(ax, [0, windows.controlEnd]);
pad_limits(ax, [tracking.current_A(selected); ...
    tracking.current_without_antiwindup_A(selected)], true);
style_axes(ax);
save_figure(fig, folder, '02_control_effort', showFigures);

% Figure 3: applied shaft torque and the resulting speed disturbance.
selected = loaded.time_s >= windows.loadStart;
t = loaded.time_s(selected);
[fig, layout] = new_figure('Load disturbance rejection', 2, showFigures);
ax = nexttile(layout);
hold(ax, 'on');
plot(ax, t, loaded.unloaded_speed_rad_s(selected), '--', ...
    'Color', gray, 'LineWidth', 1.4, 'DisplayName', 'No-load comparison');
plot(ax, t, loaded.speed_rad_s(selected), 'Color', blue, ...
    'LineWidth', 2, 'DisplayName', 'With load disturbance');
yline(ax, p.reference, ':', 'Color', gray, 'HandleVisibility', 'off');
xline(ax, p.loadTime, '--', 'Load applied', 'Color', disturbanceColor, ...
    'LabelOrientation', 'horizontal', 'FontSize', 10, 'HandleVisibility', 'off');
[minimumSpeed, minimumIndex] = min(loaded.speed_rad_s(selected));
plot(ax, t(minimumIndex), minimumSpeed, 'o', 'Color', blue, ...
    'MarkerFaceColor', 'w', 'LineWidth', 1.4, 'HandleVisibility', 'off');
text(ax, t(minimumIndex) + 0.035, minimumSpeed, ...
    sprintf('Load effect: %.2f rad/s', report.load_peak_speed_deviation_rad_s), ...
    'FontSize', 10, 'VerticalAlignment', 'bottom', 'Color', blue);
ylabel(ax, 'Angular speed, \omega [rad/s]');
title(ax, '(a) Speed response near the load step', 'FontWeight', 'normal');
xlim(ax, [windows.loadStart, windows.loadEnd]);
pad_limits(ax, [loaded.speed_rad_s(selected); ...
    loaded.unloaded_speed_rad_s(selected)], false);
legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_axes(ax);
ax = nexttile(layout);
stairs(ax, t, loaded.load_Nm(selected), 'Color', disturbanceColor, 'LineWidth', 2);
xline(ax, p.loadTime, '--', 'Color', disturbanceColor);
ylabel(ax, 'Opposing shaft torque, \tau_L [N m]');
xlabel(ax, 'Time [s]');
title(ax, '(b) Disturbance input at the motor shaft', 'FontWeight', 'normal');
xlim(ax, [windows.loadStart, windows.loadEnd]);
ylim(ax, [-0.01, 1.2*p.loadTorque]);
yticks(ax, [0, p.loadTorque/2, p.loadTorque]);
style_axes(ax);
save_figure(fig, folder, '03_load_rejection', showFigures);

% Figure 4: causal load onset is marked on both controller and current plots.
[fig, layout] = new_figure('Controller response to load', 2, showFigures);
ax = nexttile(layout);
hold(ax, 'on');
plot(ax, t, loaded.unloaded_voltage_V(selected), '--', ...
    'Color', gray, 'LineWidth', 1.4, 'DisplayName', 'No-load comparison');
plot(ax, t, loaded.voltage_V(selected), 'Color', blue, ...
    'LineWidth', 2, 'DisplayName', 'With load disturbance');
xline(ax, p.loadTime, '--', 'Load applied', 'Color', disturbanceColor, ...
    'LabelOrientation', 'horizontal', 'FontSize', 10, 'HandleVisibility', 'off');
ylabel(ax, 'Armature voltage, V [V]');
title(ax, '(a) Controller increases drive voltage', 'FontWeight', 'normal');
xlim(ax, [windows.loadStart, windows.loadEnd]);
pad_limits(ax, [loaded.voltage_V(selected); loaded.unloaded_voltage_V(selected)], false);
legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_axes(ax);
ax = nexttile(layout);
hold(ax, 'on');
plot(ax, t, loaded.unloaded_current_A(selected), '--', ...
    'Color', gray, 'LineWidth', 1.4);
plot(ax, t, loaded.current_A(selected), 'Color', blue, 'LineWidth', 2);
xline(ax, p.loadTime, '--', 'Color', disturbanceColor);
ylabel(ax, 'Armature current, i [A]');
xlabel(ax, 'Time [s]');
title(ax, '(b) Additional current produces the required shaft torque', 'FontWeight', 'normal');
xlim(ax, [windows.loadStart, windows.loadEnd]);
pad_limits(ax, [loaded.current_A(selected); loaded.unloaded_current_A(selected)], false);
style_axes(ax);
save_figure(fig, folder, '04_load_control_effort', showFigures);
end

function [fig, layout] = new_figure(name, panels, visible)
visibility = 'off';
if visible
    visibility = 'on';
end
height = 460 + 150*(panels - 1);
fig = figure('Name', name, 'Color', 'w', 'Visible', visibility, ...
    'Position', [100, 80, 980, height]);
layout = tiledlayout(fig, panels, 1, 'TileSpacing', 'compact', 'Padding', 'loose');
end

function style_axes(ax)
set(ax, 'FontName', 'Arial', 'FontSize', 12, 'LineWidth', 0.8, ...
    'TickDir', 'out', 'Box', 'off', 'YGrid', 'on', 'XGrid', 'off', ...
    'GridAlpha', 0.12, 'Layer', 'top');
end

function pad_limits(ax, values, includeZero)
lower = min(values);
upper = max(values);
if includeZero
    lower = min(lower, 0);
end
padding = max(0.12*(upper - lower), 0.05);
lowerLimit = lower - padding;
if includeZero && lower >= 0
    lowerLimit = 0;
end
ylim(ax, [lowerLimit, upper + padding]);
end

function save_figure(fig, folder, name, visible)
exportgraphics(fig, fullfile(folder, [name, '.png']), 'Resolution', 200, ...
    'Padding', 8, 'Units', 'points');
exportgraphics(fig, fullfile(folder, [name, '.pdf']), 'ContentType', 'vector', ...
    'Padding', 8, 'Units', 'points');
if ~visible
    close(fig);
end
end
