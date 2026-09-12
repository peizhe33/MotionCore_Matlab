function summary = run_motioncore_phase4(phase3File)
% RUN_MOTIONCORE_PHASE4  Golden regression, averaged-PWM build, and sweep.
% The runner never rebuilds MotionCore.slx or MotionCore_Phase3.slx.
if nargin<1 || isempty(phase3File)
    phase3File=fullfile(fileparts(mfilename('fullpath')),'MotionCore_Phase3.slx');
end
assert(isfile(phase3File),'MotionCore_Phase3.slx is required.');
[ok,a]=fileattrib(phase3File); assert(ok); phase3File=a.Name;
root=fileparts(phase3File); addpath(root);
phase4File=fullfile(root,'MotionCore_Phase4.slx'); goldenFile=fullfile(root,'MotionCore.slx');
assert(isfile(goldenFile),'The validated MotionCore.slx baseline is required.');
baselineBytes=readBytes(goldenFile); phase3Bytes=readBytes(phase3File);
[~,base]=fileparts(goldenFile); [~,p3name]=fileparts(phase3File);
load_system(goldenFile); load_system(phase3File);
assert(strcmp(get_param(base,'Dirty'),'off') && strcmp(get_param(p3name,'Dirty'),'off'), ...
    'Save the golden and Phase 3 models before running Phase 4.');
mw3=get_param(p3name,'ModelWorkspace'); mc=getVariable(mw3,'mc'); p3=getVariable(mw3,'p3');
goldenSignature=motioncore_plant_signature([base '/DC_Motor_Plant']);
assert(isequaln(goldenSignature,motioncore_plant_signature([p3name '/DC_Motor_Plant'])), ...
    'The Phase 3 plant no longer matches the golden plant. Stop before Phase 4.');
p4=motioncore_phase4_init(mc,p3);
outDir=fullfile(root,'results_phase4'); if ~isfolder(outDir),mkdir(outDir);end

fprintf('\n=== MotionCore Phase 4: preflight golden regression ===\n');
goldenOut=sim(base,'ReturnWorkspaceOutputs','on'); goldenData=collectGolden(goldenOut);
goldenChecks=check_motioncore(mc,goldenData);
assert(all(strcmp({goldenChecks.status},'PASS')),'Golden open-loop regression failed.');
save(fullfile(outDir,'golden_regression.mat'),'mc','goldenData','goldenChecks');

fprintf('\n=== MotionCore Phase 4: frozen Phase 3 regression ===\n');
for n=1:numel(p3.test_speeds_rpm)
    c3=p3; c3.reference_rpm=p3.test_speeds_rpm(n);
    [d3,s3]=runCase(p3name,c3,p3);
    [c3checks,~]=check_motioncore_phase3(mc,c3,d3,s3,true);
    assert(all(strcmp({c3checks.status},'PASS')), ...
        'Phase 3 regression failed at %g RPM.',c3.reference_rpm);
end
% Re-run the established Phase 3 anti-windup stress case before Phase 4.
stress=p3; stress.reference_rpm=3000; stress.second_step_delta_rpm=-2000; stress.stop_time_s=1.6;
[awD,awS]=runCase(p3name,stress,p3);
[awChecks,awMetrics]=check_motioncore_phase3(mc,stress,awD,awS,false);
assert(all(strcmp({awChecks.status},'PASS')) && awMetrics.saturation_occurred, ...
    'Phase 3 anti-windup stress regression failed.');

% Build a separate root model by copying Phase 3 subsystems. The source
% model's bytes and in-memory plant signature are checked again at the end.
modelFile=build_motioncore_phase4(phase3File,mc,p3,p4);
load_system(modelFile); assert(bdIsLoaded(p4.model));
assert(isequaln(goldenSignature,motioncore_plant_signature([p4.model '/DC_Motor_Plant'])), ...
    'Phase 4 plant copy differs from the validated golden plant.');

rows=struct([]); regressions=struct([]); allPassed=true; runs=cell(1,numel(p4.test_speeds_rpm));
fig=figure('Name','MotionCore Phase 4 averaged PWM','Color','w','Position',[80 60 1250 900]);
for n=1:numel(p4.test_speeds_rpm)
    target=p4.test_speeds_rpm(n); c3=p3; c3.reference_rpm=target;
    c4=p4; c4.reference_rpm=target; p3c=p3; p3c.reference_rpm=target;
    fprintf('\n--- %g RPM reference: Phase 3 vs Phase 4 ---\n',target);
    [d3,s3]=runCase(p3name,c3,p3);
    [d4,s4]=runCase(p4.model,c4,p3);
    [checks,m]=check_motioncore_phase4(mc,p3c,c4,d4,s4,true);
    reg=comparePhase3Phase4(d3,d4,c4,s3,s4);
    m.phase3_regression_pass=reg.all_passed;
    allPassed=allPassed && all(strcmp({checks.status},'PASS')) && reg.all_passed;
    if isempty(rows),rows=m;else,rows(end+1)=m;end %#ok<AGROW>
    if isempty(regressions),regressions=reg;else,regressions(end+1)=reg;end %#ok<AGROW>
    runs{n}=d4;
    stem=sprintf('speed_%g_rpm',target);
    writetable(d4,fullfile(outDir,[stem '.csv']));
    save(fullfile(outDir,[stem '.mat']),'mc','p3','p4','c3','c4','d3','s3','d4','s4','checks','m','reg');
    subplot(4,2,2*n-1); plot(d4.time_s,d4.reference_rpm,'k--',d4.time_s,d4.rpm,'b');
    grid on; ylabel('RPM'); xlabel('Time (s)'); title(sprintf('%g RPM tracking',target));
    legend('Reference','Phase 4','Location','southeast');
    subplot(4,2,2*n); yyaxis left; plot(d4.time_s,d4.voltage_avg_V,'b'); ylabel('Va avg (V)');
    yyaxis right; plot(d4.time_s,d4.duty_cycle,'r'); ylabel('Duty'); grid on; xlabel('Time (s)');
    title(sprintf('PWM duty and average voltage (max |diff| %.2g V)',reg.max_abs_voltage_avg_error_V));
end
summary=struct2table(rows); regressionTable=struct2table(regressions);
disp(summary); disp(regressionTable);
writetable(summary,fullfile(outDir,'phase4_metrics.csv'));
writetable(regressionTable,fullfile(outDir,'phase3_phase4_regression.csv'));
savefig(fig,fullfile(outDir,'speed_sweep.fig')); print(fig,fullfile(outDir,'speed_sweep.png'),'-dpng','-r150');

% Separate, non-primary experiment showing the future PWM quantisation error.
qd=motioncore_phase4_reference(mc,p3,p4,true);
quantized=table(qd.time_s,qd.duty_ideal,qd.duty_cycle,qd.voltage_avg_V, ...
    'VariableNames',{'time_s','duty_ideal','duty_quantized','voltage_quantized_V'});
writetable(quantized,fullfile(outDir,'quantized_duty_experiment.csv'));
fprintf('\nOptional %d-bit duty experiment: max duty error %.6g, max voltage error %.6g V.\n', ...
    p4.PWM_BITS,max(abs(qd.duty_cycle-qd.duty_ideal)),max(abs(qd.voltage_avg_V-qd.voltage_command_sat_V)));

assert(isequal(baselineBytes,readBytes(goldenFile)),'MotionCore:BaselineChanged', ...
    'The golden MotionCore.slx bytes changed during Phase 4.');
assert(isequal(phase3Bytes,readBytes(phase3File)),'MotionCore:Phase3Changed', ...
    'MotionCore_Phase3.slx bytes changed during Phase 4.');
assert(isequaln(goldenSignature,motioncore_plant_signature([base '/DC_Motor_Plant'])), ...
    'The golden plant changed in memory.');
save(fullfile(outDir,'phase4_validation.mat'),'mc','p3','p4','summary','regressionTable','allPassed','modelFile');
assert(allPassed,'MotionCore:Phase4ValidationFailed', ...
    'Phase 4 averaged-PWM checks failed; inspect results_phase4.');
fprintf('\nPHASE 4 PASS. Ts=%.6g s (%g Hz), fpwm=%g Hz, Vdc=%.3g V.\n', ...
    p4.Ts,p4.controller_frequency_Hz,p4.fpwm_Hz,p4.Vdc_V);
fprintf('D = sat(Vcmd/Vdc), Va_avg = D*Vdc. Phase 3/4 regression passed for all test speeds.\n');
end

function [d,s]=runCase(modelName,c,p3base)
input=Simulink.SimulationInput(modelName);
% All copied Phase 3 blocks reference p3. For Phase 4, retain every
% validated p3 field and override only the test reference/stop time.
if isfield(c,'model') && strcmp(c.model,p3base.model)
    p3run=c; % Phase 3 stress/nominal case: preserve every tested field.
else
    p3run=p3base; p3run.reference_rpm=c.reference_rpm; p3run.stop_time_s=c.stop_time_s;
end
input=input.setVariable('p3',p3run,'Workspace',modelName);
if strcmp(modelName,'MotionCore_Phase4'),input=input.setVariable('p4',c,'Workspace',modelName);end
input=input.setModelParameter('StopTime',num2str(c.stop_time_s,17));
result=sim(input);
if strcmp(modelName,'MotionCore_Phase4')
    [d,s] = motioncore_phase4_logs(result,p3run,c);
else
    [d,s] = motioncore_phase3_logs(result,p3run);
end
end

function r=comparePhase3Phase4(d3,d4,p4,s3,s4)
names={'rpm','current_A','torque_Nm','back_emf_V','voltage_V','raw_voltage_V'};
r=struct('target_rpm',d4.reference_rpm(end),'all_passed',true);
for k=1:numel(names)
    n=names{k};
    % Compare on the common controller grid. This avoids interpolation across
    % a held-voltage discontinuity while still checking every logged signal.
    v3=s3.(n); if strcmp(n,'voltage_V'),v4=s4.voltage_avg_V;else,v4=s4.(n);end
    e=max(abs(v4-v3)); rel=e/max(1,max(abs(v3)));
    r.(['max_abs_' n])=e; r.(['relative_' n])=rel;
    pass=(rel<=p4.check.phase3_regression_relative_tol) || ...
        (e<=p4.check.max_phase3_regression_abs_rpm && strcmp(n,'rpm')) || ...
        (e<=p4.check.max_phase3_regression_abs_current_A && ~strcmp(n,'rpm'));
    r.all_passed=r.all_passed && pass;
end
r.max_abs_voltage_avg_error_V=max(abs(d4.voltage_avg_V-d4.voltage_command_sat_V));
r.max_abs_duty_identity=max(abs(d4.duty_cycle-min(max(d4.voltage_command_sat_V/p4.Vdc_V,0),1)));
r.all_passed=r.all_passed && r.max_abs_voltage_avg_error_V<=p4.check.max_phase3_regression_abs_current_A && ...
    r.max_abs_duty_identity<=p4.check.duty_tolerance;
end

function d=collectGolden(o)
sig=o.get('omega_rad_s'); t=unique(double(sig.Time(:)),'stable'); d=table(t,'VariableNames',{'time_s'});
names={'rpm','current_A','torque_Nm','back_emf_V','omega_rad_s','voltage_V','load_Nm'};
for k=1:numel(names)
    sig=o.get(names{k}); [st,idx]=unique(double(sig.Time(:)),'last'); v=double(sig.Data(:)); v=v(idx);
    if numel(st)==1,d.(names{k})=repmat(v,numel(t),1);
    else,method='linear';if any(strcmp(names{k},{'voltage_V','load_Nm'})),method='previous';end
        d.(names{k})=interp1(st,v,t,method,'extrap');end
end
end
function b=readBytes(file)
f=fopen(file,'rb');assert(f~=-1); c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
end
