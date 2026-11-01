function [windows,p] = plan_figure_windows(p)
%PLAN_FIGURE_WINDOWS Settle both startup responses before applying the load.
probeEnd = max(p.loadTime+1.5,p.referenceTime+2);
currentUnloaded = p.b*p.reference/p.Kt;
voltageUnloaded = p.R*currentUnloaded+p.Ke*p.reference;
currentLoaded = (p.b*p.reference+p.loadTorque)/p.Kt;
voltageLoaded = p.R*currentLoaded+p.Ke*p.reference;
loadSettling = Inf;
for attempt = 1:5
    t = (0:p.sampleTime:probeEnd).';
    pNoLoad = p;
    pNoLoad.loadTorque = 0;
    pNoAwNoLoad = pNoLoad;
    pNoAwNoLoad.Kaw = 0;
    nominal = reference_solution(t,pNoLoad);
    nominalNoAw = reference_solution(t,pNoAwNoLoad);
    startupResponses = {nominal,nominalNoAw};
    startupParameters = {pNoLoad,pNoAwNoLoad};
    startup = zeros(2,3);
    for controller = 1:2
        states = startupResponses{controller};
        startup(controller,1) = settled_duration(t,states(:,2)-p.reference, ...
            p.referenceTime,0.02*abs(p.reference));
        startup(controller,2) = settled_duration(t,states(:,1)-currentUnloaded, ...
            p.referenceTime,0.02*abs(currentUnloaded));
        voltage = controller_voltage(t,states,startupParameters{controller});
        startup(controller,3) = settled_duration(t,voltage-voltageUnloaded, ...
            p.referenceTime,0.02*abs(voltageUnloaded));
    end
    if ~all(isfinite(startup),'all')
        probeEnd = 2*probeEnd;
        continue
    end
    p.loadTime = p.referenceTime+max(startup,[],'all')+p.sampleTime;
    pNoAw = p;
    pNoAw.Kaw = 0;
    responses = {reference_solution(t,p),reference_solution(t,pNoAw)};
    parameters = {p,pNoAw};
    recovery = zeros(2,3);
    for controller = 1:2
        states = responses{controller};
        speedExcursion = max(abs(states(t>=p.loadTime,2)-p.reference));
        recovery(controller,1) = settled_duration(t,states(:,2)-p.reference, ...
            p.loadTime,0.02*speedExcursion);
        recovery(controller,2) = settled_duration(t,states(:,1)-currentLoaded, ...
            p.loadTime,0.02*abs(currentLoaded-currentUnloaded));
        voltage = controller_voltage(t,states,parameters{controller});
        recovery(controller,3) = settled_duration(t,voltage-voltageLoaded, ...
            p.loadTime,0.02*abs(voltageLoaded-voltageUnloaded));
    end
    loadSettling = max(recovery,[],'all');
    if isfinite(loadSettling) && probeEnd-p.loadTime-loadSettling>=0.1
        break
    end
    probeEnd = 2*probeEnd;
end
assert(isfinite(loadSettling),'Cannot identify settled figure windows.');
windows.stopTime = ceil((p.loadTime+1.1*loadSettling)/p.sampleTime)*p.sampleTime;
windows.trackingEnd = windows.stopTime;
windows.controlEnd = windows.stopTime;
windows.loadStart = max(0,p.loadTime-0.1);
windows.loadEnd = windows.stopTime;
windows.headroom_percent = 10;
windows.initial_settled_at_s = p.referenceTime+startup;
windows.load_time_s = p.loadTime;
windows.signal_order = {'speed_rad_s','current_A','voltage_V'};
windows.final_settled_at_s = p.loadTime+recovery;
windows.tracking_band_percent = 2;
windows.post_load_speed_band_percent_of_excursion = 2;
windows.post_load_current_voltage_band_percent_of_step = 2;
windows.limiting_post_load_settling_time_s = loadSettling;
windows.post_load_settling_with_antiwindup_s = struct( ...
    'speed',recovery(1,1),'current',recovery(1,2),'voltage',recovery(1,3));
windows.post_load_settling_without_antiwindup_s = struct( ...
    'speed',recovery(2,1),'current',recovery(2,2),'voltage',recovery(2,3));
assert(p.loadTime>max(windows.initial_settled_at_s,[],'all'), ...
    'Load precedes initial settling.');
end

function voltage = controller_voltage(t,states,p)
request = p.reference*(t>=p.referenceTime);
voltage = p.Kp*(request-states(:,2))+states(:,3) ...
    -p.Kd*p.N*(states(:,2)-states(:,4));
voltage = min(p.voltageLimit,max(-p.voltageLimit,voltage));
end

function duration = settled_duration(t,deviation,eventTime,tolerance)
indices = find(t>=eventTime);
if isempty(indices)
    duration = Inf;
    return
end
last = find(abs(deviation(indices))>tolerance,1,'last');
if isempty(last)
    duration = 0;
elseif last==numel(indices)
    duration = Inf;
else
    duration = t(indices(last+1))-eventTime;
end
end
