function x = motioncore_reference(mc, t)
% MOTIONCORE_REFERENCE Exact piecewise-constant-input state trajectory.
% MATLAB only: no Control System Toolbox or Simulink required.
% Output columns: [current_A omega_rad_s]. Valid for t >= 0.
t = t(:);
assert(all(isfinite(t)) && all(t >= 0));
A = mc.math.A; B = mc.math.B;
x0 = [mc.initial.current_A; mc.initial.omega_rad_s];
ts = mc.input.step_time_s;
pre = -A\(B*[mc.input.voltage_before_V; mc.input.load_torque_Nm]);
post = -A\(B*[mc.input.voltage_after_V; mc.input.load_torque_Nm]);
xSwitch = pre + expm(A*ts)*(x0-pre);
x = zeros(numel(t), 2);
for n = 1:numel(t)
    if t(n) < ts
        state = pre + expm(A*t(n))*(x0-pre);
    else
        state = post + expm(A*(t(n)-ts))*(xSwitch-post);
    end
    x(n,:) = state.';
end
end
