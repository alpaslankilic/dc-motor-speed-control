function windows = plan_figure_windows(p)
%PLAN_FIGURE_WINDOWS Include startup AND post-load settling in every full plot.
% Equation-level sizing precedes final Simulink runs; 20% headroom is added.
probeEnd = max(p.loadTime + 1.5, p.referenceTime + 2);
pNoLoad = p;
pNoLoad.loadTorque = 0;
pNoAw = p;
pNoAw.Kaw = 0;
currentUnloaded = p.b*p.reference/p.Kt;
voltageUnloaded = p.R*currentUnloaded + p.Ke*p.reference;
currentLoaded = (p.b*p.reference + p.loadTorque)/p.Kt;
voltageLoaded = p.R*currentLoaded + p.Ke*p.reference;
for attempt = 1:5
    t = (0:p.sampleTime:probeEnd).';
    nominal = reference_solution(t, pNoLoad);
    loaded = reference_solution(t, p);
    baseline = reference_solution(t, pNoAw);
    trackingAw = settled_duration(t, loaded(:, 2) - p.reference, ...
        p.referenceTime, 0.02*abs(p.reference), p.loadTime);
    trackingNoAw = settled_duration(t, baseline(:, 2) - p.reference, ...
        p.referenceTime, 0.02*abs(p.reference), p.loadTime);
    nominalVoltage = controller_voltage(t, nominal, pNoLoad);
    currentSettling = settled_duration(t, nominal(:, 1) - currentUnloaded, ...
        p.referenceTime, 0.02*abs(currentUnloaded), Inf);
    voltageSettling = settled_duration(t, nominalVoltage - voltageUnloaded, ...
        p.referenceTime, 0.02*abs(voltageUnloaded), Inf);
    responses = {loaded, baseline};
    parameters = {p, pNoAw};
    recovery = zeros(2, 3);
    for controller = 1:2
        states = responses{controller};
        speedExcursion = max(abs(states(t >= p.loadTime, 2) - p.reference));
        recovery(controller, 1) = settled_duration(t, states(:, 2) - p.reference, ...
            p.loadTime, 0.02*speedExcursion, Inf);
        recovery(controller, 2) = settled_duration(t, states(:, 1) - currentLoaded, ...
            p.loadTime, 0.02*abs(currentLoaded - currentUnloaded), Inf);
        voltage = controller_voltage(t, states, parameters{controller});
        recovery(controller, 3) = settled_duration(t, voltage - voltageLoaded, ...
            p.loadTime, 0.02*abs(voltageLoaded - voltageUnloaded), Inf);
    end
    startup = max([trackingAw, trackingNoAw, currentSettling, voltageSettling]);
    loadSettling = max(recovery, [], 'all');
    lastEvent = max(p.referenceTime + startup, p.loadTime + loadSettling);
    if isfinite(lastEvent) && probeEnd - lastEvent >= 0.15
        break
    end
    probeEnd = 2*probeEnd;
end
assert(isfinite(lastEvent), 'Cannot identify settled figure windows.');
roundUp = @(time) ceil(time/p.sampleTime)*p.sampleTime;
windows.stopTime = roundUp(max(p.referenceTime + 1.2*startup, ...
    p.loadTime + 1.2*loadSettling));
windows.trackingEnd = windows.stopTime;
windows.controlEnd = windows.stopTime;
windows.loadStart = max(0, p.loadTime - 0.1);
windows.loadEnd = windows.stopTime;
windows.headroom_percent = 20;
windows.tracking_band_percent = 2;
windows.post_load_speed_band_percent_of_excursion = 2;
windows.post_load_current_voltage_band_percent_of_step = 2;
windows.limiting_post_load_settling_time_s = loadSettling;
windows.post_load_settling_with_antiwindup_s = struct( ...
    'speed', recovery(1, 1), 'current', recovery(1, 2), 'voltage', recovery(1, 3));
windows.post_load_settling_without_antiwindup_s = struct( ...
    'speed', recovery(2, 1), 'current', recovery(2, 2), 'voltage', recovery(2, 3));
end

function voltage = controller_voltage(t, states, p)
request = p.reference*(t >= p.referenceTime);
voltage = p.Kp*(request - states(:, 2)) + states(:, 3) ...
    - p.Kd*p.N*(states(:, 2) - states(:, 4));
voltage = min(p.voltageLimit, max(-p.voltageLimit, voltage));
end

function duration = settled_duration(t, deviation, eventTime, tolerance, endTime)
indices = find(t >= eventTime & t < endTime);
last = find(abs(deviation(indices)) > tolerance, 1, 'last');
if isempty(last)
    duration = 0;
elseif last == numel(indices)
    duration = Inf;
else
    duration = t(indices(last + 1)) - eventTime;
end
end
