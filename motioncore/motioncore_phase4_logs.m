function [d,s] = motioncore_phase4_logs(simOut,p3,p4)
% Reuse the validated Phase 3 logger for legacy signals, then add PWM logs.
[d,s] = motioncore_phase3_logs(simOut,p3);
names={'voltage_command_sat_V','duty_cycle','voltage_avg_V','pwm_error_V'};
for k=1:numel(names)
    sig=simOut.get(names{k});
    [st,idx]=unique(double(sig.Time(:)),'last'); values=double(sig.Data(:)); values=values(idx);
    if numel(st)==1
        d.(names{k})=repmat(values,height(d),1); s.(names{k})=repmat(values,height(s),1);
    else
        d.(names{k})=interp1(st,values,d.time_s,'previous','extrap');
        s.(names{k})=interp1(st,values,s.time_s,'previous','extrap');
    end
end
end
