function [d,s,e] = motioncore_phase5_logs(simOut,p3run,p5,stage)
% Required logs fail loudly. Preserve event-side semantics by snapping sample
% hits and retaining the last value at duplicate event times.
if nargin<4,stage='encoder';end
names={'rpm','current_A','torque_Nm','back_emf_V','omega_rad_s','voltage_V','load_Nm'};
continuous={'rpm','current_A','torque_Nm','back_emf_V','omega_rad_s','shaft_angle_rad','encoder_rpm_error'};
if ~strcmp(stage,'golden')
    names=[names {'reference_rpm','measured_rpm','error_rpm','raw_voltage_V', ...
        'integrator_V','derivative_rpm_s','aw_error_V'}];
end
if any(strcmp(stage,{'phase4','encoder'}))
    names=[names {'voltage_command_sat_V','duty_cycle','voltage_avg_V','pwm_error_V'}];
end
if strcmp(stage,'encoder')
    names=[names {'shaft_angle_rad','encoder_count','encoder_delta_count','encoder_rpm','encoder_rpm_error'}];
end
sig=required(simOut,'current_A'); t=unique(snap(double(sig.Time(:)),p3run.Ts));
d=table(t,'VariableNames',{'time_s'});
s=table((0:round(p3run.stop_time_s/p3run.Ts)).'*p3run.Ts,'VariableNames',{'time_s'});
e=table((0:floor((p3run.stop_time_s+1e-10)/p5.Tenc)).'*p5.Tenc,'VariableNames',{'time_s'});
for k=1:numel(names)
    n=names{k}; sig=required(simOut,n);
    [st,idx]=unique(snap(double(sig.Time(:)),p3run.Ts),'last'); v=double(sig.Data(:)); v=v(idx);
    assert(numel(st)>1 || strcmp(n,'load_Nm'), ...
        'MotionCore:ShortLog','Required log %s has fewer than two samples.',n);
    assert(all(isfinite(v)) && all(isfinite(st)),'Nonfinite required log: %s',n);
    if numel(st)==1
        d.(n)=repmat(v,height(d),1); s.(n)=repmat(v,height(s),1); e.(n)=repmat(v,height(e),1);
    else
        expectedEnd=p3run.stop_time_s;
        encoderHeld=any(strcmp(n,{'encoder_count','encoder_delta_count','encoder_rpm'}));
        if encoderHeld,expectedEnd=floor((p3run.stop_time_s+1e-10)/p5.Tenc)*p5.Tenc;end
        assert(st(1)<=1e-9 && st(end)>=expectedEnd-1e-9,'Incomplete log: %s',n);
        method='previous'; if any(strcmp(n,continuous)),method='linear';end
        if encoderHeld
            % The last estimator value legitimately holds to StopTime when
            % StopTime is not an integer multiple of Tenc; no samples invented.
            d.(n)=interp1(st,v,d.time_s,method,'extrap');
            s.(n)=interp1(st,v,s.time_s,method,'extrap');
            e.(n)=interp1(st,v,e.time_s,method,'extrap');
        else
            d.(n)=interp1(st,v,d.time_s,method); s.(n)=interp1(st,v,s.time_s,method);
            e.(n)=interp1(st,v,e.time_s,method);
        end
    end
end
if ~strcmp(stage,'golden')
    assert(max(abs(d.reference_rpm))>0 && max(abs(d.rpm))>0 && max(abs(d.current_A))>0, ...
        'MotionCore:DisconnectedLogging','Nonzero test has empty/zero reference, speed or current logs.');
    d.true_rpm=d.rpm; d.feedback_rpm=d.measured_rpm;
    s.true_rpm=s.rpm; s.feedback_rpm=s.measured_rpm;
    e.true_rpm=e.rpm; e.feedback_rpm=e.measured_rpm;
end
end
function sig=required(o,name)
available=who(o);
assert(any(strcmp(available,name)),'MotionCore:MissingLog','Missing required Simulink log: %s',name);
sig=o.get(name);
assert(isa(sig,'timeseries') && ~isempty(sig.Time) && numel(sig.Data)==numel(sig.Time), ...
    'MotionCore:BadLog','Log %s must be a nonempty scalar timeseries.',name);
end
function t=snap(t,Ts)
nearest=round(t/Ts)*Ts; hit=abs(t-nearest)<1e-9*max(1,Ts); t(hit)=nearest(hit);
end
