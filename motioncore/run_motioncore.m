% RUN_MOTIONCORE Build, simulate, plot and validate Phase 1-2.
% Set MATLAB Current Folder to this directory, then run: run_motioncore
% Rebuilds from source; existing saved SLX is copied into backups first.
root = fileparts(mfilename('fullpath'));
run(fullfile(root,'motioncore_init.m'));
modelFile = build_motioncore(mc);
simOut = sim(mc.model, 'ReturnWorkspaceOutputs', 'on');
omegaLog = simOut.get('omega_rad_s');
% Use actual logged times, not a presumed fixed-step output grid.
time_s = unique(double(omegaLog.Time(:)), 'last');
names = {'rpm','current_A','torque_Nm','back_emf_V', ...
    'omega_rad_s','voltage_V','load_Nm'};
d = table(time_s);
for k = 1:numel(names)
    sig = simOut.get(names{k});
    [st, si] = unique(double(sig.Time(:)), 'last');
    values = double(sig.Data(:));
    values = values(si);
    if isequal(st,time_s)
        d.(names{k}) = values;
    elseif numel(st) == 1
        d.(names{k}) = repmat(values,numel(time_s),1);
    else
        method = 'linear';
        if any(strcmp(names{k},{'voltage_V','load_Nm'})), method = 'previous'; end
        d.(names{k}) = interp1(st,values,time_s,method,'extrap');
    end
end
xRef = motioncore_reference(mc,time_s);
checks = check_motioncore(mc,d);
[peakCurrent, peakIdx] = max(d.current_A);
fprintf('\nFinal speed: %.3f RPM; expected equilibrium: %.3f RPM\n', ...
    d.rpm(end),mc.math.final_state(2)*mc.units.rad_s_to_rpm);
fprintf('Final current: %.6f A; expected equilibrium: %.6f A\n', ...
    d.current_A(end),mc.math.final_state(1));
fprintf('Peak current: %.4f A at t = %.6f s\n',peakCurrent,time_s(peakIdx));
fprintf('Poles: %.6g, %.6g 1/s; slow time constant: %.6g s\n', ...
    mc.math.poles(1),mc.math.poles(2),mc.math.slow_tau_s);

fig = figure('Name','MotionCore Phase 1-2: open-loop plant','Color','w', ...
    'Position',[80 80 1150 780]);
subplot(3,2,1);
plot(time_s,d.rpm,'b',time_s,xRef(:,2)*mc.units.rad_s_to_rpm,'r--');
grid on; ylabel('Speed (RPM)'); legend('Simulink','Exact reference','Location','best');
title('Open-loop speed');
subplot(3,2,2); plot(time_s,d.current_A,'b'); grid on;
ylabel('Current (A)'); title('Armature current');
subplot(3,2,3); plot(time_s,d.torque_Nm,'b',time_s,mc.motor.b*d.omega_rad_s,'r--');
grid on; ylabel('Torque (N m)'); legend('Electromagnetic','Viscous friction','Location','best');
subplot(3,2,4); plot(time_s,d.voltage_V,'k',time_s,d.back_emf_V,'b');
grid on; ylabel('Voltage (V)'); legend('Armature voltage','Back EMF','Location','best');
subplot(3,2,5); plot(time_s,d.omega_rad_s,'b'); grid on;
ylabel('Speed (rad/s)'); xlabel('Time (s)');
subplot(3,2,6); plot(time_s,d.load_Nm,'b'); grid on;
ylabel('Load torque (N m)'); xlabel('Time (s)');
for k = 1:6, subplot(3,2,k); xlabel('Time (s)'); end
outDir = fullfile(root,'results');
if ~exist(outDir,'dir'), mkdir(outDir); end
writetable(d,fullfile(outDir,'open_loop_signals.csv'));
save(fullfile(outDir,'open_loop_results.mat'),'mc','simOut','d','checks');
savefig(fig,fullfile(outDir,'open_loop_waveforms.fig'));
print(fig,fullfile(outDir,'open_loop_waveforms.png'),'-dpng','-r150');
fprintf('Results saved in: %s\n',outDir);
if any(strcmp({checks.status},'FAIL'))
    error('MotionCore:ValidationFailed', ...
        'Plant sanity checks failed. Inspect checks and saved plots before Phase 3.');
end
if any(strcmp({checks.status},'SKIP'))
    warning('MotionCore:IncompleteValidation', ...
        'Some checks were skipped. Review them before Phase 3.');
end
