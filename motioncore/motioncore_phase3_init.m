function p3 = motioncore_phase3_init(mc)
% PHASE 3 ONLY. mc is read from the validated MotionCore model workspace.
% Units: controller error is RPM; voltage output and integral state are V.
p3.model = 'MotionCore_Phase3';
p3.Ts = 1e-3;                      % s, 1 kHz controller and feedback sampling
p3.bandwidth_rad_s = 15;           % conservative nominal design target
p3.candidate_bandwidths = [8 15 25];
p3.test_speeds_rpm = [500 1000 1500 2000];
p3.step_time_s = 0.05;
p3.stop_time_s = 1.0;
p3.reference_rpm = 1000;
p3.initial_reference_rpm = 0;
p3.second_step_s = 0.65;
p3.second_step_delta_rpm = 0;       % zero for nominal single-step tests
p3.voltage_min_V = 0;
p3.voltage_max_V = mc.input.voltage_after_V; % validated 12 V supply
p3.Kd = 0;                        % V*s/RPM: start in PI mode intentionally
p3.derivative_filter_s = 0.005;    % s; used if Kd is enabled later
p3.antiwindup_time_s = 0.05;
p3.Kb = 1/p3.antiwindup_time_s;     % 1/s, back-calculation gain
p3.integrator_initial_V = 0;
p3.load_Nm = 0;                    % constant; no load-step disturbance yet
p3.check.steady_error_rpm = 1;
p3.check.settling_band_fraction = 0.02;
p3.check.settling_limit_s = 0.5;
p3.check.hold_time_s = 0.05;
p3.check.overshoot_limit_pct = 5;
p3.check.reference_relative_tol = 2e-4;
p3.check.identity_relative_tol = 1e-7;
p3.check.energy_relative_tol = mc.check.energy_relative_tol;

% Recompute matrices from golden motor PARAMETERS without modifying mc.
m = mc.motor;
p3.A = [-m.Ra/m.La -m.Ke/m.La; m.Kt/m.J -m.b/m.J];
p3.B = [1/m.La 0; 0 -1/m.J];
poles = eig(p3.A);
assert(all(real(poles)<0) && max(abs(imag(poles)))<1e-8, ...
    'MotionCore:PlantChanged','This initial tuning assumes two real stable motor poles.');
p3.slow_pole_rad_s = min(-real(poles));
p3.fast_pole_rad_s = max(-real(poles));
p3.dc_gain_rpm_V = mc.units.rad_s_to_rpm*m.Kt/(m.Ra*m.b+m.Kt*m.Ke);
% G(s) ~= G0*a/(s+a); PI zero at -a gives L(s) ~= bandwidth/s.
p3.Kp = p3.bandwidth_rad_s/(p3.dc_gain_rpm_V*p3.slow_pole_rad_s);
p3.Ki = p3.slow_pole_rad_s*p3.Kp;
p3.derivative_a = p3.derivative_filter_s/(p3.derivative_filter_s+p3.Ts);
p3.derivative_b = 1/(p3.derivative_filter_s+p3.Ts);
assert(p3.Ts>0 && p3.Ts*p3.Kb<1 && p3.voltage_max_V>p3.voltage_min_V);
assert(p3.voltage_min_V==0 && p3.voltage_max_V>0, ...
    'Phase 3 checks currently assume a positive supply and 0 V lower limit.');
assert(mc.initial.current_A==0 && mc.initial.omega_rad_s==0, ...
    'Nominal startup tests assume the validated zero initial state.');
assert(abs(mc.motor.Kt-mc.motor.Ke)<1e-12, ...
    'Phase 3 energy validation assumes reciprocal motor constants in SI.');
assert(norm(p3.A-mc.math.A,inf)<1e-10 && norm(p3.B-mc.math.B,inf)<1e-10, ...
    'Golden mc math matrices disagree with its motor parameters.');
end
