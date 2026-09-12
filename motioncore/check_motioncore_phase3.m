function [checks,metrics] = check_motioncore_phase3(mc,p3,d,s,isNominal)
% Original check_motioncore remains unchanged: its fixed-input reference
% applies to the GOLDEN run only. Here use exact held-voltage plant updates.
if nargin<5,isNominal=true;end
checks=struct('name',{},'status',{},'detail',{});
ref=motioncore_phase3_reference(mc,p3);
values=d{:,:}; sampleValues=s{:,:};
add('All signals finite',all(isfinite(values(:))) && all(isfinite(sampleValues(:))),'Dense and sampled logs');
compare={'rpm','current_A','voltage_V','integrator_V','derivative_rpm_s'};
for k=1:numel(compare)
    n=compare{k}; normErr=max(abs(s.(n)-ref.(n)))/max(1,max(abs(ref.(n))));
    add(['Exact sampled reference: ' n],normErr<p3.check.reference_relative_tol, ...
        sprintf('Normalized max error %.3g',normErr));
end
tol=p3.check.identity_relative_tol;
add('Torque = Kt*i',near(d.torque_Nm,mc.motor.Kt*d.current_A,tol),'Golden identity');
add('Back EMF = Ke*omega',near(d.back_emf_V,mc.motor.Ke*d.omega_rad_s,tol),'Golden identity');
add('RPM conversion',near(d.rpm,mc.units.rad_s_to_rpm*d.omega_rad_s,tol),'Golden identity');
add('Sampled feedback and reference',near(s.measured_rpm,s.rpm,tol) && ...
    near(s.reference_rpm,ref.reference_rpm,tol),'Ideal speed sampling; no encoder');
add('Error and PID algebra',near(s.error_rpm,s.reference_rpm-s.measured_rpm,tol) && ...
    near(s.raw_voltage_V,p3.Kp*s.error_rpm+s.integrator_V-p3.Kd*s.derivative_rpm_s,tol), ...
    'Explicit discrete parallel PID');
expectedU=min(max(s.raw_voltage_V,p3.voltage_min_V),p3.voltage_max_V);
add('Voltage saturation and AW tracking',near(s.voltage_V,expectedU,tol) && ...
    near(s.aw_error_V,s.voltage_V-s.raw_voltage_V,tol) && ...
    min(d.voltage_V)>=p3.voltage_min_V-1e-8 && max(d.voltage_V)<=p3.voltage_max_V+1e-8, ...
    '0 to supply limit');
expectedI=s.integrator_V(1:end-1)+p3.Ts*(p3.Ki*s.error_rpm(1:end-1)+p3.Kb*s.aw_error_V(1:end-1));
add('Forward Euler integral and anti-windup recurrence',near(s.integrator_V(2:end),expectedI,tol),'One update per Ts');
expectedD=p3.derivative_a*s.derivative_rpm_s(1:end-1)+ ...
    p3.derivative_b*diff(s.measured_rpm);
add('Filtered derivative recurrence',near(s.derivative_rpm_s(2:end),expectedD,tol),'Backward Euler, measurement only');

% Exact plant transition using the OBSERVED voltage, independently of PID.
E=expm([p3.A p3.B;zeros(2,4)]*p3.Ts);
x=[s.current_A s.omega_rad_s];
pred=x(1:end-1,:)*E(1:2,1:2).'+ ...
    [s.voltage_V(1:end-1) s.load_Nm(1:end-1)]*E(1:2,3:4).';
normErr=max(max(abs(x(2:end,:)-pred)./max(max(abs(x),[],1),[1 1])));
add('Golden plant exact ZOH transition',normErr<p3.check.reference_relative_tol,sprintf('Error %.3g',normErr));

% Energy integral uses LEFT-held voltage on each continuous solver interval.
% This avoids trapezoidal averaging across a controller voltage discontinuity.
dt=diff(d.time_s); i=d.current_A; w=d.omega_rad_s;
Ein=sum(dt.*d.voltage_V(1:end-1).*(i(1:end-1)+i(2:end))/2);
loss=mc.motor.Ra*i.^2+mc.motor.b*w.^2+d.load_Nm.*w;
Eloss=trapz(d.time_s,loss);
energy=0.5*mc.motor.La*i.^2+0.5*mc.motor.J*w.^2;
residual=Ein-Eloss-(energy(end)-energy(1));
scale=max([abs(Ein),abs(energy(end)-energy(1)),1e-6]);
add('Energy conservation',abs(residual)/scale<p3.check.energy_relative_tol, ...
    sprintf('Relative residual %.3g',abs(residual)/scale));
add('Current physically bounded',max(abs(i))<=1.02*p3.voltage_max_V/mc.motor.Ra, ...
    sprintf('Peak absolute current %.4f A (no current limiter)',max(abs(i))));
if isNominal
    metrics=motioncore_phase3_metrics(d,p3);
    add('Steady speed tracking',metrics.tail_max_error_rpm<=p3.check.steady_error_rpm, ...
        sprintf('Mean signed error %.4g RPM; tail max %.4g RPM',metrics.steady_error_rpm,metrics.tail_max_error_rpm));
    add('Rise and settling times',isfinite(metrics.rise_time_s) && ...
        isfinite(metrics.settling_time_s) && metrics.settling_time_s<=p3.check.settling_limit_s, ...
        sprintf('Rise %.4f s, settling %.4f s',metrics.rise_time_s,metrics.settling_time_s));
    add('Overshoot',metrics.overshoot_pct<=p3.check.overshoot_limit_pct, ...
        sprintf('%.4g percent',metrics.overshoot_pct));
    tail=d.time_s>=d.time_s(end)-p3.check.hold_time_s;
    omega=p3.reference_rpm/mc.units.rad_s_to_rpm;
    ieq=(mc.motor.b*omega+p3.load_Nm)/mc.motor.Kt;
    veq=mc.motor.Ra*ieq+mc.motor.Ke*omega;
    add('Reachable equilibrium and final current/voltage',veq>=p3.voltage_min_V && veq<=p3.voltage_max_V && ...
        abs(mean(i(tail))-ieq)<max(0.005,0.01*abs(ieq)) && ...
        abs(mean(d.voltage_V(tail))-veq)<0.02, ...
        sprintf('Expected %.4f A, %.4f V',ieq,veq));
else
    metrics=motioncore_phase3_metrics(d,p3,p3.second_step_s,p3.reference_rpm, ...
        p3.reference_rpm+p3.second_step_delta_rpm);
end
for k=1:numel(checks)
    fprintf('[%s] %s: %s\n',checks(k).status,checks(k).name,checks(k).detail);
end

    function add(name,pass,detail)
        status='FAIL';if pass,status='PASS';end
        checks(end+1)=struct('name',name,'status',status,'detail',detail);
    end
end
function yes=near(a,b,tol)
yes=all(abs(a-b)<=tol*max(1,max(abs(b))));
end
