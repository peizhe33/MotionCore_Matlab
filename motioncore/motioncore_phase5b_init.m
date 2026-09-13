function b = motioncore_phase5b_init(mc,p3,p4,p5)
% Phase 5B only. Retain every controller/PWM/estimator setting from Phase 5A.
assert(p3.Ts==0.001 && abs(p3.Kp-0.00620355671)<1e-10 && ...
    abs(p3.Ki-0.0848230016)<1e-10 && p3.Kd==0 && p3.Kb==20);
assert(p4.Vdc_V==12 && p4.fpwm_Hz==20000);
assert(p5.ENCODER_PPR==1024 && p5.decode_multiplier==4 && ...
    p5.ENCODER_CPR==4096 && p5.Tenc==0.001);
b.model='MotionCore_Phase5B';
b.Tedge=1e-6;                 % A/B observation + decoder update, 1 MHz
b.CPR=p5.ENCODER_CPR;
b.design_max_rpm=3000;        % guard envelope, above reachable no-load speed
b.no_load_rpm=p4.Vdc_V*mc.motor.Kt/(mc.motor.Ra*mc.motor.b+mc.motor.Kt*mc.motor.Ke)*mc.units.rad_s_to_rpm;
b.transition_rate_2000_Hz=2000*b.CPR/60;
b.transition_rate_no_load_Hz=b.no_load_rpm*b.CPR/60;
b.transition_rate_design_Hz=b.design_max_rpm*b.CPR/60;
assert(b.no_load_rpm<b.design_max_rpm && b.transition_rate_design_Hz*b.Tedge<0.25, ...
    'Choose at least four observations per encoder transition at the design speed.');
assert(abs(p3.Ts/b.Tedge-round(p3.Ts/b.Tedge))<1e-9 && ...
    abs(p5.Tenc/b.Tedge-round(p5.Tenc/b.Tedge))<1e-9);
% Natural binary state = 2*A+B. Forward Gray cycle: 0 -> 1 -> 3 -> 2 -> 0.
b.A_table=logical([0 0 1 1]); b.B_table=logical([0 1 1 0]);
b.delta_table=int8([0 1 -1 0, -1 0 0 1, 1 0 0 -1, 0 -1 1 0]);
b.invalid_table=logical([0 0 0 1, 0 0 1 0, 0 1 0 0, 1 0 0 0]);
% Index is 4*previous_state+current_state (zero based).
b.initial_count=int64(floor(p5.theta_initial_rad*b.CPR/(2*pi)));
phase=mod(double(b.initial_count),4); gray=uint8([0 1 3 2]);
b.initial_state=gray(phase+1);
b.test_speeds_rpm=[500 1000 1500 2000];
b.zoom_start_s=0.8; b.zoom_duration_s=100e-6;
b.compare.copy_atol=1e-7; b.compare.copy_rtol=2e-6;
b.compare.max_true_rpm=1;      % 0.2% of the lowest main target
b.compare.max_encoder_rpm=2*p5.rpm_per_count; % endpoint differences of one count
b.compare.max_current_A=0.1; b.compare.max_voltage_V=0.2;
b.compare.max_duty=0.2/p4.Vdc_V; b.compare.max_count=1;
% Local decoded-vs-ideal count must be EXACT on the shared edge grid. Only
% cross-run floor-boundary differences get the +/-1-count comparison allowance.
end
