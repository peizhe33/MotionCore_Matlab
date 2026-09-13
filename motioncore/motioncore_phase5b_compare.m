function r = motioncore_phase5b_compare(a,z,b,mode)
% 'basic' and 'copy' are numerical identity gates; 'quadrature' permits a
% one-count cross-run phase difference and its consequent estimator ripple.
if nargin<4,mode='quadrature';end
assert(isequal(a.time_s,z.time_s),'Controller sample grids differ.');
if strcmp(mode,'basic')
    names={'rpm','current_A','torque_Nm','back_emf_V','voltage_V','raw_voltage_V'};
elseif strcmp(mode,'copy')
    names=a.Properties.VariableNames; names=setdiff(names,{'time_s'},'stable');
else
    names={'rpm','encoder_rpm','current_A','voltage_command_sat_V','raw_voltage_V', ...
        'duty_cycle','encoder_count','integrator_V'};
end
r=struct('passed',true);
for k=1:numel(names)
    n=names{k}; diff=z.(n)-a.(n); err=max(abs(diff));
    if ~strcmp(mode,'quadrature')
        limit=b.compare.copy_atol+b.compare.copy_rtol*max(abs(a.(n)));
    else
        switch n
            case 'rpm',limit=b.compare.max_true_rpm;
            case 'encoder_rpm',limit=b.compare.max_encoder_rpm;
            case 'current_A',limit=b.compare.max_current_A;
            case {'voltage_command_sat_V','raw_voltage_V'},limit=b.compare.max_voltage_V;
            case 'duty_cycle',limit=b.compare.max_duty;
            case 'encoder_count',limit=b.compare.max_count;
            case 'integrator_V',limit=0.02;
        end
    end
    r.(['max_abs_' n])=err; r.(['rms_' n])=sqrt(mean(diff.^2));
    r.(['final_difference_' n])=diff(end); r.(['limit_' n])=limit;
    r.passed=r.passed && err<=limit;
end
end
