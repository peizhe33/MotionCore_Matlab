function d = motioncore_phase3_reference(mc,p3)
% Exact ZOH plant propagation plus explicitly ordered digital PID recurrence.
% MATLAB only; no Simulink or Control System Toolbox. Samples are POST-output,
% PRE-state-update at t=k*Ts. The voltage is held until the next sample.
N = round(p3.stop_time_s/p3.Ts);
assert(abs(N*p3.Ts-p3.stop_time_s)<1e-9);
time_s = (0:N).'*p3.Ts;
ref = p3.initial_reference_rpm + ...
    (p3.reference_rpm-p3.initial_reference_rpm)*(time_s>=p3.step_time_s-1e-12) + ...
    p3.second_step_delta_rpm*(time_s>=p3.second_step_s-1e-12);
E = expm([p3.A p3.B; zeros(2,4)]*p3.Ts);
Ad = E(1:2,1:2); Bd = E(1:2,3:4);
x = [mc.initial.current_A; mc.initial.omega_rad_s];
iv = p3.integrator_initial_V; prevY = x(2)*mc.units.rad_s_to_rpm; prevD = 0;
v = zeros(N+1,11);
for k = 1:N+1
    y = x(2)*mc.units.rad_s_to_rpm;
    e = ref(k)-y;
    deriv = p3.derivative_a*prevD+p3.derivative_b*(y-prevY);
    raw = p3.Kp*e+iv-p3.Kd*deriv;
    u = min(max(raw,p3.voltage_min_V),p3.voltage_max_V);
    aw = u-raw;
    v(k,:) = [y x(1) x(2) e raw u iv deriv aw mc.motor.Kt*x(1) mc.motor.Ke*x(2)];
    % Forward Euler I state: present output uses iv[k], not iv[k+1].
    iv = iv+p3.Ts*(p3.Ki*e+p3.Kb*aw);
    prevD = deriv; prevY = y;
    x = Ad*x+Bd*[u;p3.load_Nm];
end
d = array2table(v,'VariableNames',{'rpm','current_A','omega_rad_s', ...
    'error_rpm','raw_voltage_V','voltage_V','integrator_V', ...
    'derivative_rpm_s','aw_error_V','torque_Nm','back_emf_V'});
d.time_s = time_s; d.reference_rpm = ref; d.measured_rpm = d.rpm;
d.load_Nm = p3.load_Nm+zeros(N+1,1);
end
