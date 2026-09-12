% MOTIONCORE_INIT Phase 1-2 parameters, all in SI units except RPM.
% Illustrative permanent-magnet brushed DC motor, NOT a fitted real motor.
% Re-run run_motioncore after editing. No workspace clear or path changes.
mc = struct();
mc.model = 'MotionCore';
mc.motor.Ra = 2.0;       % ohm
mc.motor.La = 2.0e-3;    % H
mc.motor.Ke = 0.05;      % V/(rad/s)
mc.motor.Kt = 0.05;      % N*m/A; numerically Ke in consistent SI units
mc.motor.J  = 1.0e-4;    % kg*m^2, total shaft-referred rotor + load inertia
mc.motor.b  = 1.0e-4;    % N*m/(rad/s), viscous friction only
mc.initial.current_A = 0;
mc.initial.omega_rad_s = 0;
mc.input.step_time_s = 0.05;
mc.input.voltage_before_V = 0;
mc.input.voltage_after_V = 12;
mc.input.load_torque_Nm = 0; % Constant signed opposing torque; no pulse yet
mc.units.rad_s_to_rpm = 60/(2*pi);
mc.sim.stop_time_s = 0.8;
mc.sim.solver = 'ode45';
mc.sim.max_step_s = 5e-5; % 20 maximum steps/electrical time constant
mc.sim.rel_tol = 1e-7;
mc.sim.abs_tol = 1e-9;
mc.check.state_relative_tol = 1e-4;
mc.check.steady_relative_tol = 2e-3;
mc.check.energy_relative_tol = 5e-3;
% PID, PWM, encoder and fixed-point parameters belong to later phases.

assert(all(isfinite([mc.motor.Ra mc.motor.La mc.motor.Ke ...
    mc.motor.Kt mc.motor.J mc.motor.b])));
assert(all([mc.motor.Ra mc.motor.La mc.motor.Ke mc.motor.Kt mc.motor.J] > 0));
assert(mc.motor.b >= 0);
assert(mc.input.step_time_s > 0 && mc.sim.stop_time_s > mc.input.step_time_s);
assert(mc.sim.max_step_s > 0 && mc.sim.rel_tol > 0 && mc.sim.abs_tol > 0);
assert(all(isfinite([mc.initial.current_A mc.initial.omega_rad_s ...
    mc.input.voltage_before_V mc.input.voltage_after_V mc.input.load_torque_Nm])));

% x = [current_A; omega_rad_s], u = [armature_V; load_Nm].
mc.math.A = [-mc.motor.Ra/mc.motor.La, -mc.motor.Ke/mc.motor.La; ...
             mc.motor.Kt/mc.motor.J,  -mc.motor.b/mc.motor.J];
mc.math.B = [1/mc.motor.La, 0; 0, -1/mc.motor.J];
mc.math.C = eye(2);
mc.math.D = zeros(2);
% Speed / voltage TF, descending powers of s; zero load and initial states.
mc.math.speed_tf_num = mc.motor.Kt;
mc.math.speed_tf_den = [mc.motor.La*mc.motor.J, ...
    mc.motor.La*mc.motor.b + mc.motor.Ra*mc.motor.J, ...
    mc.motor.Ra*mc.motor.b + mc.motor.Kt*mc.motor.Ke];
mc.math.poles = eig(mc.math.A);
mc.math.slow_tau_s = max(-1./real(mc.math.poles));
mc.math.electrical_tau_s = mc.motor.La/mc.motor.Ra;
mc.math.final_state = -mc.math.A\(mc.math.B * ...
    [mc.input.voltage_after_V; mc.input.load_torque_Nm]);
if mc.sim.max_step_s > mc.math.electrical_tau_s/10
    warning('MotionCore:Resolution', 'MaxStep exceeds electrical tau/10.');
end
