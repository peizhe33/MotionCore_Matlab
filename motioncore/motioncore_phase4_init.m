function p4 = motioncore_phase4_init(mc,p3)
% PHASE 4 ONLY: averaged PWM parameters derived from validated Phase 3.
% The PI coefficients are copied, not retuned. Encoder, switching PWM and
% fixed-point parameters are intentionally absent from this phase.
assert(isstruct(mc) && isstruct(p3),'MotionCore:BadParameters', ...
    'Pass the validated mc and p3 parameter structures.');
p4.model = 'MotionCore_Phase4';
p4.phase3_model = p3.model;
p4.controller_Ts = p3.Ts;
p4.Ts = p3.Ts;
p4.Kp = p3.Kp;
p4.Ki = p3.Ki;
p4.Kd = p3.Kd;
p4.Kb = p3.Kb;
p4.fpwm_Hz = 20e3;
p4.Tpwm_s = 1/p4.fpwm_Hz;
p4.controller_frequency_Hz = 1/p4.Ts;
p4.Vdc_V = p3.voltage_max_V;
p4.PWM_BITS = 12;                 % Future FPGA resolution; not applied here
p4.quantized_duty_experiment = false;
p4.test_speeds_rpm = p3.test_speeds_rpm;
p4.reference_rpm = p3.reference_rpm;
p4.initial_reference_rpm = p3.initial_reference_rpm;
p4.step_time_s = p3.step_time_s;
p4.stop_time_s = p3.stop_time_s;
p4.load_Nm = p3.load_Nm;
p4.second_step_s = p3.second_step_s;
p4.second_step_delta_rpm = p3.second_step_delta_rpm;
p4.voltage_min_V = p3.voltage_min_V;
p4.voltage_max_V = p3.voltage_max_V;
p4.antiwindup_time_s = p3.antiwindup_time_s;
p4.check = p3.check;
p4.check.phase3_regression_relative_tol = 2e-6;
p4.check.pwm_identity_relative_tol = 2e-8;
p4.check.duty_tolerance = 2e-8;
p4.check.max_phase3_regression_abs_rpm = 1e-3;
p4.check.max_phase3_regression_abs_current_A = 1e-6;

assert(abs(p4.Vdc_V-p3.voltage_max_V)<eps(max(1,abs(p4.Vdc_V))), ...
    'MotionCore:SupplyMismatch','PWM bus must equal the Phase 3 upper voltage limit initially.');
assert(p4.Vdc_V>0 && p4.voltage_min_V==0 && p4.voltage_max_V==p4.Vdc_V);
assert(p4.fpwm_Hz>0 && p4.Tpwm_s>0 && p4.controller_frequency_Hz>0);
assert(abs(p4.fpwm_Hz/p4.controller_frequency_Hz-round(p4.fpwm_Hz/p4.controller_frequency_Hz))<1e-12, ...
    'MotionCore:FrequencyRatio','Choose PWM and controller rates with an integer ratio initially.');
assert(p4.fpwm_Hz/p4.controller_frequency_Hz>=10, ...
    'MotionCore:PWMRate','PWM rate should be much higher than speed-loop rate.');
assert(isequal(p4.Kp,p3.Kp) && isequal(p4.Ki,p3.Ki) && ...
    isequal(p4.Kd,p3.Kd) && isequal(p4.Kb,p3.Kb), ...
    'MotionCore:ControllerChanged','Phase 4 must initially preserve the Phase 3 PI controller.');
assert(abs(p4.Ts-1e-3)<1e-12 && abs(p4.Kp-0.00620355671)<1e-10 && ...
    abs(p4.Ki-0.0848230016)<1e-10 && p4.Kd==0 && abs(p4.Kb-20)<1e-12, ...
    'MotionCore:UnexpectedPhase3Gains', ...
    'The checked Phase 3 controller gains differ from the frozen baseline.');
assert(p4.PWM_BITS>=1 && p4.PWM_BITS<=24 && fix(p4.PWM_BITS)==p4.PWM_BITS);
assert(mc.motor.Ra>0 && mc.motor.La>0 && mc.motor.J>0);
end
