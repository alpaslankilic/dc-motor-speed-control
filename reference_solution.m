function states = reference_solution(t, p)
%REFERENCE_SOLUTION Independent ODE45 solution of the closed-loop equations.
% State order: armature current, angular speed, integral voltage, filter state.
states = zeros(numel(t), 4);
edges = unique([t(1), p.referenceTime, p.loadTime, t(end)]);
edges = edges(edges >= t(1) & edges <= t(end));
x0 = zeros(4, 1);
options = odeset('RelTol', 1e-10, 'AbsTol', 1e-11, 'MaxStep', 0.002);
for k = 1:numel(edges)-1
    left = edges(k);
    right = edges(k+1);
    % Hold the step values over each segment to avoid integrating a jump.
    midpoint = (left + right)/2;
    request = p.reference * (midpoint >= p.referenceTime);
    load = p.loadTorque * (midpoint >= p.loadTime);
    solution = ode45(@(~, x) rhs(x, request, load, p), ...
        [left, right], x0, options);
    selected = t >= left & t <= right;
    states(selected, :) = deval(solution, t(selected)).';
    x0 = solution.y(:, end);
end
end

function dx = rhs(x, request, load, p)
error = request - x(2);
derivative = p.Kd*p.N*(x(2) - x(4));
rawVoltage = p.Kp*error + x(3) - derivative;
voltage = min(p.voltageLimit, max(-p.voltageLimit, rawVoltage));
dx = [(voltage - p.R*x(1) - p.Ke*x(2))/p.L; ...
      (p.Kt*x(1) - p.b*x(2) - load)/p.J; ...
      p.Ki*error + p.Kaw*(voltage - rawVoltage); ...
      p.N*(x(2) - x(4))];
end
