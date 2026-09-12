function m = motioncore_phase4_metrics(d,p4,eventTime,startRPM,targetRPM)
% Phase 4 performance metrics. Rise uses interpolated 10--90% crossings.
if nargin<3
    eventTime=p4.step_time_s; startRPM=p4.initial_reference_rpm; targetRPM=p4.reference_rpm;
end
take=d.time_s>=eventTime-1e-10; t=d.time_s(take); y=d.rpm(take);
delta=targetRPM-startRPM; assert(abs(delta)>0);
progress=(y-startRPM)/delta;
t10=crossing(t,progress,.1); t90=crossing(t,progress,.9);
m.target_rpm=targetRPM;
m.rise_time_s=t90-t10;
m.overshoot_pct=max(0,100*(max(progress)-1));
band=max(1,p4.check.settling_band_fraction*abs(delta)); outside=find(abs(y-targetRPM)>band);
if isempty(outside),k=1;else,k=outside(end)+1;end
m.settling_time_s=NaN;
if k<=numel(t) && t(end)-t(k)>=p4.check.hold_time_s,m.settling_time_s=t(k)-eventTime;end
tail=d.time_s>=d.time_s(end)-p4.check.hold_time_s;
m.steady_error_rpm=targetRPM-mean(d.rpm(tail));
m.tail_max_error_rpm=max(abs(targetRPM-d.rpm(tail)));
m.max_current_A=max(d.current_A); m.min_current_A=min(d.current_A);
m.max_abs_current_A=max(abs(d.current_A));
m.max_voltage_command_V=max(d.voltage_command_sat_V);
m.min_voltage_command_V=min(d.voltage_command_sat_V);
m.max_average_voltage_V=max(d.voltage_avg_V);
m.min_average_voltage_V=min(d.voltage_avg_V);
m.max_duty=max(d.duty_cycle); m.min_duty=min(d.duty_cycle);
sat=abs(d.raw_voltage_V-d.voltage_command_sat_V)>1e-8;
m.saturation_occurred=any(sat);
m.saturation_duration_s=sum(diff(d.time_s).*double(sat(1:end-1)));
m.saturation_percentage=100*m.saturation_duration_s/max(d.time_s(end)-d.time_s(1),eps);
values=d{:,:}; m.all_signals_finite=all(isfinite(values(:)));
end
function tc=crossing(t,p,level)
k=find(p>=level,1);tc=NaN;
if isempty(k),return;end
if k==1,tc=t(1);return;end
tc=t(k-1)+(level-p(k-1))*(t(k)-t(k-1))/(p(k)-p(k-1));
end
