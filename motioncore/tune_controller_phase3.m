function tuning = tune_controller_phase3(mc,p3)
% Deterministic bounded PI bandwidth comparison; no toolbox autotuner.
% The 15 rad/s candidate is selected in init for moderate speed/current.
assert(p3.Kd==0,'MotionCore:TuningScope', ...
    'This initial tuning/stability comparison is for PI (Kd=0). Retune before enabling derivative action.');
rows=struct([]);
for wc=p3.candidate_bandwidths
    c=p3;
    c.Kp=wc/(c.dc_gain_rpm_V*c.slow_pole_rad_s); c.Ki=c.slow_pole_rad_s*c.Kp;
    worstOS=0; worstSettle=0; maxCurrent=0;
    for target=c.test_speeds_rpm
        c.reference_rpm=target;
        d=motioncore_phase3_reference(mc,c); m=motioncore_phase3_metrics(d,c);
        worstOS=max(worstOS,m.overshoot_pct);
        if isnan(m.settling_time_s), worstSettle=Inf;
        else, worstSettle=max(worstSettle,m.settling_time_s); end
        maxCurrent=max(maxCurrent,m.max_abs_current_A);
    end
    % Unsaturated augmented sampled [i omega I] closed-loop dynamics.
    E=expm([c.A c.B;zeros(2,4)]*c.Ts); Ad=E(1:2,1:2); Bu=E(1:2,3);
    C=[0 mc.units.rad_s_to_rpm];
    F=[Ad-Bu*c.Kp*C Bu;-c.Ts*c.Ki*C 1];
    row=struct('bandwidth_rad_s',wc,'Kp',c.Kp,'Ki',c.Ki, ...
        'max_pole_magnitude',max(abs(eig(F))), ...
        'worst_settling_s',worstSettle,'worst_overshoot_pct',worstOS, ...
        'sampled_peak_current_A',maxCurrent);
    if isempty(rows)
        rows = row;
    else
        rows(end+1) = row; %#ok<AGROW>
    end
end
tuning=struct2table(rows); disp(tuning);
assert(all(tuning.max_pole_magnitude<1),'A sampled PI candidate is unstable.');
end
