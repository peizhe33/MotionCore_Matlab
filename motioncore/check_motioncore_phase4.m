function [checks,metrics] = check_motioncore_phase4(mc,p3,p4,d,s,isNominal)
% CHECK_MOTIONCORE_PHASE4  Averaged-PWM and golden-plant checks.
if nargin<6,isNominal=true;end
checks=struct('name',{},'status',{},'detail',{});
tol=p4.check.pwm_identity_relative_tol;
ref=motioncore_phase4_reference(mc,p3,p4,false);
dv=d{:,:}; sv=s{:,:};
add('All signals finite',all(isfinite(dv(:))) && all(isfinite(sv(:))), ...
    'Dense and sampled logs');

% The copied digital controller must still match the frozen Phase 3 sampled
% reference. PWM is an identity map at Vdc=12 V, so these are also a direct
% regression of the original controller algebra.
for n={'rpm','current_A','voltage_V','integrator_V','derivative_rpm_s'}
    name=n{1}; err=max(abs(s.(name)-ref.(name)))/max(1,max(abs(ref.(name))));
    add(['Exact sampled reference: ' name],err<p3.check.reference_relative_tol, ...
        sprintf('Normalized max error %.3g',err));
end

add('Torque = Kt*i',near(d.torque_Nm,mc.motor.Kt*d.current_A,tol),'Plant identity');
add('Back EMF = Ke*omega',near(d.back_emf_V,mc.motor.Ke*d.omega_rad_s,tol),'Plant identity');
add('RPM conversion',near(d.rpm,mc.units.rad_s_to_rpm*d.omega_rad_s,tol),'Plant identity');
add('Ideal sampled feedback/reference',near(s.measured_rpm,s.rpm,tol) && ...
    near(s.reference_rpm,ref.reference_rpm,tol),'No encoder quantisation in Phase 4');

expectedDuty=min(max(d.voltage_command_sat_V/p4.Vdc_V,0),1);
add('Duty-cycle equation',near(d.duty_cycle,expectedDuty,tol) && ...
    min(d.duty_cycle)>=-p4.check.duty_tolerance && max(d.duty_cycle)<=1+p4.check.duty_tolerance, ...
    'D = sat(Vcmd/Vdc), 0 <= D <= 1');
add('Average-voltage reconstruction',near(d.voltage_avg_V,d.duty_cycle*p4.Vdc_V,tol) && ...
    near(d.voltage_avg_V,d.voltage_command_sat_V,tol), ...
    'Va_avg = D*Vdc and equals Phase 3 command');
add('PWM error is zero',max(abs(d.pwm_error_V))<=p4.check.max_phase3_regression_abs_current_A, ...
    sprintf('Max |Va_avg-Vcmd_sat| %.3g V',max(abs(d.pwm_error_V))));
add('Applied voltage within supply',min(d.voltage_avg_V)>=p4.voltage_min_V-1e-8 && ...
    max(d.voltage_avg_V)<=p4.voltage_max_V+1e-8,'Supply limits');

% Explicitly verify the sampled plant recurrence using the applied averaged
% voltage observed in the log, independent of controller implementation.
E=expm([p3.A p3.B;zeros(2,4)]*p3.Ts);
x=[s.current_A s.omega_rad_s];
pred=x(1:end-1,:)*E(1:2,1:2).' + ...
    [s.voltage_avg_V(1:end-1) s.load_Nm(1:end-1)]*E(1:2,3:4).';
err=max(max(abs(x(2:end,:)-pred)./max(max(abs(x),[],1),[1 1])));
add('Golden plant exact ZOH transition',err<p3.check.reference_relative_tol,sprintf('Error %.3g',err));

% Energy uses left-held applied voltage on each controller interval.
dt=diff(d.time_s); i=d.current_A; w=d.omega_rad_s;
Ein=sum(dt.*d.voltage_avg_V(1:end-1).*(i(1:end-1)+i(2:end))/2);
loss=mc.motor.Ra*i.^2+mc.motor.b*w.^2+d.load_Nm.*w;
energy=0.5*mc.motor.La*i.^2+0.5*mc.motor.J*w.^2;
residual=Ein-trapz(d.time_s,loss)-(energy(end)-energy(1));
scale=max([abs(Ein),abs(energy(end)-energy(1)),1e-6]);
add('Energy conservation',abs(residual)/scale<p3.check.energy_relative_tol, ...
    sprintf('Relative residual %.3g',abs(residual)/scale));
add('Current physically bounded',max(abs(i))<=1.02*p4.voltage_max_V/mc.motor.Ra, ...
    sprintf('Peak absolute current %.4f A',max(abs(i))));

% Controller recurrence and anti-windup are unchanged from Phase 3.
expectedI=s.integrator_V(1:end-1)+p3.Ts*(p3.Ki*s.error_rpm(1:end-1)+p3.Kb*s.aw_error_V(1:end-1));
add('Forward Euler integral and anti-windup',near(s.integrator_V(2:end),expectedI,tol), ...
    'One update per controller sample');
add('Controller voltage algebra',near(s.raw_voltage_V,p3.Kp*s.error_rpm+s.integrator_V-p3.Kd*s.derivative_rpm_s,tol) && ...
    near(s.voltage_command_sat_V,min(max(s.raw_voltage_V,p3.voltage_min_V),p3.voltage_max_V),tol), ...
    'Unsaturated PI -> supply saturation');

if isNominal
    metrics=motioncore_phase4_metrics(d,p4);
    add('Steady speed tracking',metrics.tail_max_error_rpm<=p3.check.steady_error_rpm, ...
        sprintf('Tail max error %.4g RPM',metrics.tail_max_error_rpm));
    add('Rise and settling times',isfinite(metrics.rise_time_s) && isfinite(metrics.settling_time_s) && ...
        metrics.settling_time_s<=p3.check.settling_limit_s, ...
        sprintf('Rise %.4f s, settling %.4f s',metrics.rise_time_s,metrics.settling_time_s));
    add('Overshoot',metrics.overshoot_pct<=p3.check.overshoot_limit_pct, ...
        sprintf('%.4g percent',metrics.overshoot_pct));
else
    metrics=motioncore_phase4_metrics(d,p4,p4.second_step_s,p4.reference_rpm, ...
        p4.reference_rpm+p4.second_step_delta_rpm);
end
for k=1:numel(checks),fprintf('[%s] %s: %s\n',checks(k).status,checks(k).name,checks(k).detail);end

    function add(name,pass,detail)
        status='FAIL';if pass,status='PASS';end
        checks(end+1)=struct('name',name,'status',status,'detail',detail); %#ok<AGROW>
    end
end
function tf=near(a,b,t)
tf=all(abs(a-b)<=t*max(1,max(abs(b))));
end
