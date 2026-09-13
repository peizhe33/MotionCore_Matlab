function s = motioncore_phase5_reference(mc,p3,p4,p5)
% Exact augmented ZOH plant [i; omega; theta] + count-only sampled PI feedback.
% At an estimator hit: sample theta -> floor -> difference -> PI output ->
% update PI registers -> propagate plant. First delta is zero at rest.
p5=motioncore_phase5_parameters(p5,p3);
N=round(p3.stop_time_s/p3.Ts); M=round(p5.Tenc/p3.Ts);
assert(abs(N*p3.Ts-p3.stop_time_s)<1e-10);
A=[p3.A zeros(2,1);0 1 0]; B=[p3.B;0 0];
E=expm([A B;zeros(2,5)]*p3.Ts);
x=[mc.initial.current_A;mc.initial.omega_rad_s;p5.theta_initial_rad];
iv=p3.integrator_initial_V; oldY=mc.initial.omega_rad_s*mc.units.rad_s_to_rpm;
oldDeriv=0; count=floor(x(3)*p5.ENCODER_CPR/(2*pi)); prevCount=count;
delta=0; measured=0;
names={'rpm','current_A','omega_rad_s','shaft_angle_rad','encoder_count', ...
    'encoder_delta_count','encoder_rpm','measured_rpm','error_rpm','raw_voltage_V', ...
    'voltage_V','integrator_V','derivative_rpm_s','aw_error_V','torque_Nm', ...
    'back_emf_V','reference_rpm','load_Nm','duty_cycle','voltage_avg_V'};
values=zeros(N+1,numel(names)); t=(0:N).'*p3.Ts;
for k=1:N+1
    if mod(k-1,M)==0
        count=floor(x(3)*p5.ENCODER_CPR/(2*pi));
        delta=count-prevCount; prevCount=count; measured=delta*p5.rpm_per_count;
    end
    ref=p3.initial_reference_rpm+(p3.reference_rpm-p3.initial_reference_rpm)* ...
        (t(k)>=p3.step_time_s-1e-12)+p3.second_step_delta_rpm*(t(k)>=p3.second_step_s-1e-12);
    rpm=x(2)*mc.units.rad_s_to_rpm; err=ref-measured;
    deriv=p3.derivative_a*oldDeriv+p3.derivative_b*(measured-oldY);
    raw=p3.Kp*err+iv-p3.Kd*deriv;
    u=min(max(raw,p3.voltage_min_V),p3.voltage_max_V);
    duty=min(max(u/p4.Vdc_V,0),1); va=duty*p4.Vdc_V; aw=u-raw;
    values(k,:)=[rpm x(1) x(2) x(3) count delta measured measured err raw u iv ...
        deriv aw mc.motor.Kt*x(1) mc.motor.Ke*x(2) ref p3.load_Nm duty va];
    iv=iv+p3.Ts*(p3.Ki*err+p3.Kb*aw); oldY=measured; oldDeriv=deriv;
    x=E(1:3,1:3)*x+E(1:3,4:5)*[va;p3.load_Nm];
end
s=array2table(values,'VariableNames',names); s.time_s=t;
s.voltage_command_sat_V=s.voltage_V; s.pwm_error_V=s.voltage_avg_V-s.voltage_V;
s.encoder_rpm_error=s.encoder_rpm-s.rpm; s.true_rpm=s.rpm; s.feedback_rpm=s.measured_rpm;
end
