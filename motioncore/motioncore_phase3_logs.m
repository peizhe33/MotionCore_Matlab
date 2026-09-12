function [d,s] = motioncore_phase3_logs(simOut,p3)
% Dense plant grid for peaks/energy, separate controller grid for recurrences.
names={'rpm','current_A','torque_Nm','back_emf_V','omega_rad_s', ...
    'voltage_V','load_Nm','reference_rpm','measured_rpm','error_rpm', ...
    'raw_voltage_V','integrator_V','derivative_rpm_s','aw_error_V'};
continuous={'rpm','current_A','torque_Nm','back_emf_V','omega_rad_s'};
sig=simOut.get('current_A'); denseTime=unique(snap(double(sig.Time(:)),p3.Ts));
sampleTime=(0:round(p3.stop_time_s/p3.Ts)).'*p3.Ts;
d=table(denseTime,'VariableNames',{'time_s'});
s=table(sampleTime,'VariableNames',{'time_s'});
for k=1:numel(names)
    sig=simOut.get(names{k});
    [st,idx]=unique(snap(double(sig.Time(:)),p3.Ts),'last');
    values=double(sig.Data(:)); values=values(idx);
    assert(numel(st)==1 || (st(1)<=1e-9 && st(end)>=p3.stop_time_s-1e-9), ...
        'Incomplete log: %s',names{k});
    if numel(st)==1
        d.(names{k})=repmat(values,height(d),1); s.(names{k})=repmat(values,height(s),1);
    else
        method='previous'; if any(strcmp(names{k},continuous)),method='linear';end
        d.(names{k})=interp1(st,values,denseTime,method,'extrap');
        s.(names{k})=interp1(st,values,sampleTime,method,'extrap');
    end
end
end

function t=snap(t,Ts)
nearest=round(t/Ts)*Ts;
near=abs(t-nearest)<1e-9*max(1,Ts);
t(near)=nearest(near);
end
