function p5 = motioncore_phase5_init(p3,p4)
% PHASE 5A: encoder parameters only; p3/p4 remain the frozen runtime structs.
p5.model = 'MotionCore_Phase5';
p5.ENCODER_PPR = 1024;       % cycles/revolution on ONE conceptual A channel
p5.decode_multiplier = 4;   % x4 interpretation; no A/B edge model yet
p5.ENCODER_CPR = p5.decode_multiplier*p5.ENCODER_PPR;
p5.Tenc = p3.Ts;            % s, fixed count-difference window
p5.theta_initial_rad = 0;
p5.rpm_per_count = 60/(p5.ENCODER_CPR*p5.Tenc);
p5.test_speeds_rpm = [500 1000 1500 2000];
p5.low_speeds_rpm = [50 100 250 500];
p5.resolution_sweep_ppr = [128 256 1024 2048];
p5.run_optional_experiments = false; % runtime overrides only, no saved edits
p5.tail_window_s = 0.2;     % time-weighted metrics over a uniform sample grid
p5.check.mean_speed_error_rpm = 1;
p5.check.max_tail_speed_error_rpm = 3;
p5.check.overshoot_pct = 5;
p5.check.settling_limit_s = 0.5;
p5.check.identity_tol = 1e-7;
p5.check.transition_tol = p3.check.reference_relative_tol;
p5.check.copy_relative_tol = 2e-6;
p5.check.copy_absolute_tol = 1e-7;
p5.check.reference_rpm_tol = 2*p5.rpm_per_count;
assert(abs(p3.Ts-0.001)<1e-12 && abs(p3.Kp-0.00620355671)<1e-10 && ...
    abs(p3.Ki-0.0848230016)<1e-10 && p3.Kd==0 && p3.Kb==20, ...
    'MotionCore:Phase5Baseline','Unexpected frozen controller coefficients.');
assert(p4.Vdc_V==12 && p4.fpwm_Hz==20000 && p3.voltage_min_V==0 && ...
    p3.voltage_max_V==p4.Vdc_V,'Expected the validated 12 V averaged driver.');
end
