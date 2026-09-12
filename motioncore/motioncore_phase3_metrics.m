function m = motioncore_phase3_metrics(d,p3,eventTime,startRPM,targetRPM)
% Metrics use actual shaft RPM. Rise time is 10-90% of commanded change.
% Settling requires remaining within 2% of the change through end of record,
% with at least hold_time_s of observed in-band data. Missing times are NaN.
if nargin<3
    eventTime=p3.step_time_s; startRPM=p3.initial_reference_rpm; targetRPM=p3.reference_rpm;
end
take=d.time_s>=eventTime-1e-10; t=d.time_s(take); y=d.rpm(take);
assert(~isempty(t));
delta=targetRPM-startRPM;
assert(abs(delta)>0,'Metrics need a nonzero commanded speed change.');
progress=(y-startRPM)/delta;
t10=crossing(t,progress,0.1); t90=crossing(t,progress,0.9);
m.target_rpm=targetRPM;
m.rise_time_s=t90-t10;
m.overshoot_pct=max(0,100*(max(progress)-1));
band=max(1,p3.check.settling_band_fraction*abs(delta));
outside=find(abs(y-targetRPM)>band);
if isempty(outside), settled=1; else, settled=outside(end)+1; end
m.settling_time_s=NaN;
if settled<=numel(t) && t(end)-t(settled)>=p3.check.hold_time_s
    m.settling_time_s=t(settled)-eventTime;
end
tail=d.time_s>=d.time_s(end)-p3.check.hold_time_s;
m.steady_error_rpm=targetRPM-mean(d.rpm(tail));
m.tail_max_error_rpm=max(abs(targetRPM-d.rpm(tail)));
m.max_current_A=max(d.current_A); m.min_current_A=min(d.current_A);
m.max_abs_current_A=max(abs(d.current_A));
m.max_voltage_V=max(d.voltage_V); m.min_voltage_V=min(d.voltage_V);
m.max_abs_integrator_V=max(abs(d.integrator_V));
sat=abs(d.raw_voltage_V-d.voltage_V)>1e-8;
m.saturation_occurred=any(sat);
m.saturation_duration_s=sum(diff(d.time_s).*double(sat(1:end-1)));
values=d{:,:}; m.all_signals_finite=all(isfinite(values(:)));
end

function tc=crossing(t,p,level)
k=find(p>=level,1); tc=NaN;
if isempty(k), return; end
if k==1, tc=t(1); return; end
tc=t(k-1)+(level-p(k-1))*(t(k)-t(k-1))/(p(k)-p(k-1));
end
