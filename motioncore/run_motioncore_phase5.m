function summary = run_motioncore_phase5(root,runOptional)
% Run only against the user's already validated, saved Phase 1--4 baseline.
% No previous builder/runner is invoked; original check functions are reused.
if nargin<1 || isempty(root),root=fileparts(mfilename('fullpath'));end
if nargin<2,runOptional=false;end
[ok,a]=fileattrib(root); assert(ok && a.directory); root=a.Name;
oldPath=path; guard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(root); names={'MotionCore','MotionCore_Phase3','MotionCore_Phase4'};
files=cellfun(@(n)fullfile(root,[n '.slx']),names,'UniformOutput',false);
for k=1:numel(files)
    assert(isfile(files{k}),'MotionCore:MissingBaseline', ...
        'Missing validated baseline: %s. Supply the corrected SLX; Phase 5 will not rebuild it.',files{k});
    load_system(files{k});
    assert(strcmpi(get_param(names{k},'FileName'),files{k}) && strcmp(get_param(names{k},'Dirty'),'off'), ...
        'Save/close conflicting or unsaved baseline: %s',names{k});
end
frozenFiles=files; code=dir(fullfile(root,'*.m'));
for k=1:numel(code)
    if ~contains(code(k).name,'phase5'),frozenFiles{end+1}=fullfile(root,code(k).name);end %#ok<AGROW>
end
before=cellfun(@readBytes,frozenFiles,'UniformOutput',false);
preserveGuard=onCleanup(@()verifyFiles(frozenFiles,before)); %#ok<NASGU>
mw=get_param(names{1},'ModelWorkspace'); mc0=getVariable(mw,'mc');
mw=get_param(names{2},'ModelWorkspace'); p3gold=getVariable(mw,'p3');
mw=get_param(names{3},'ModelWorkspace'); mc=getVariable(mw,'mc');
p3=getVariable(mw,'p3'); p4=getVariable(mw,'p4');
for key={'Ts','Kp','Ki','Kd','Kb','voltage_min_V','voltage_max_V','A','B'}
    assert(isequaln(p3.(key{1}),p3gold.(key{1})),'Phase 3/4 parameter mismatch: %s',key{1});
end
assert(isequaln(mc.motor,mc0.motor) && isequaln(mc.initial,mc0.initial));
p5=motioncore_phase5_parameters(motioncore_phase5_init(p3,p4),p3);
p5.run_optional_experiments=logical(runOptional);
% Each invocation gets its own directory, preserving previous Phase 5 reports.
parent=fullfile(root,'results_phase5'); if ~isfolder(parent),mkdir(parent);end
outDir=tempname(parent); mkdir(outDir);
fprintf('\nPhase 5A results: %s\n',outDir);
goldSig=motioncore_plant_signature([names{1} '/DC_Motor_Plant']);
for k=2:3
    assert(isequaln(goldSig,motioncore_plant_signature([names{k} '/DC_Motor_Plant'])),'Golden plant differs.');
end
baseStructure=motioncore_phase5_structure(names{3});

% Fail closed at every earlier validation gate, before copying/modifying.
fprintf('\n--- Frozen golden open-loop regression ---\n');
o=sim(names{1},'ReturnWorkspaceOutputs','on'); gp=p3; gp.stop_time_s=mc0.sim.stop_time_s;
[d,~,~]=motioncore_phase5_logs(o,gp,p5,'golden');
goldenChecks=check_motioncore(mc0,d);
save(fullfile(outDir,'golden_regression.mat'),'goldenChecks','d'); requirePass(goldenChecks);
baselines=cell(1,numel(p5.test_speeds_rpm));
for k=1:numel(p5.test_speeds_rpm)
    c=p3; c.reference_rpm=p5.test_speeds_rpm(k); q=runtime4(p4,c);
    fprintf('\n--- Frozen Phase 3/4: %g RPM ---\n',c.reference_rpm);
    [d3,s3]=simulate(names{2},c,q,p5,'phase3');
    [checks3,~]=check_motioncore_phase3(mc,c,d3,s3,true); requirePass(checks3);
    [d4,s4]=simulate(names{3},c,q,p5,'phase4');
    [checks4,~]=check_motioncore_phase4(mc,c,q,d4,s4,true); requirePass(checks4);
    reg34=compare(s3,s4,p5,true); assert(reg34.pass,'Frozen Phase 3/4 numerical regression failed.');
    baselines{k}=struct('d',d4,'s',s4,'p3',c,'p4',q);
    save(fullfile(outDir,sprintf('frozen_%g.mat',c.reference_rpm)), ...
        'c','q','d3','s3','d4','s4','checks3','checks4','reg34');
end
% Preserve the established unreachable-reference AW on/off experiment.
stress=p3; stress.reference_rpm=3000; stress.second_step_delta_rpm=-2000; stress.stop_time_s=1.6;
for k=1:2
    c=stress; if k==2,c.Kb=0;end
    [sd,ss]=simulate(names{2},c,runtime4(p4,c),p5,'phase3');
    [checks,met]=check_motioncore_phase3(mc,c,sd,ss,false); requirePass(checks);
    if k==1,on=met;onRecovery=recovery(sd,c);else,off=met;offRecovery=recovery(sd,c);end
    save(fullfile(outDir,sprintf('frozen_aw_%d.mat',k)),'c','sd','ss','checks','met');
end
offTime=offRecovery; if isnan(offTime),offTime=Inf;end
assert(on.saturation_occurred && isfinite(onRecovery) && onRecovery<0.5 && ...
    onRecovery<offTime && on.max_abs_integrator_V<off.max_abs_integrator_V && ...
    on.tail_max_error_rpm<=p3.check.steady_error_rpm, ...
    'Frozen anti-windup effectiveness regression failed.');
[sd,ss]=simulate(names{3},stress,runtime4(p4,stress),p5,'phase4');
[checks,~]=check_motioncore_phase4(mc,stress,runtime4(p4,stress),sd,ss,false); requirePass(checks);

fprintf('\n--- Copy Phase 4 whole model; validate before enabling encoder ---\n');
modelFile=build_motioncore_phase5(files{3},p5,'copy');
motioncore_phase5_structure(p5.model,baseStructure,false);
copyRows=struct([]);
for k=1:numel(baselines)
    b=baselines{k}; [dc,sc]=simulate(p5.model,b.p3,b.p4,p5,'phase4');
    reg=compare(b.s,sc,p5,false); reg.target_rpm=b.p3.reference_rpm;
    copyRows=append(copyRows,reg);
    assert(reg.pass,'Untouched Phase 4/5 copy regression failed at %g RPM.',b.p3.reference_rpm);
end
writetable(struct2table(copyRows),fullfile(outDir,'copy_regression.csv'));
build_motioncore_phase5(files{3},p5,'encoder');
motioncore_phase5_structure(p5.model,baseStructure,true);

rows=struct([]); changes=struct([]);
for k=1:numel(baselines)
    b=baselines{k}; c=b.p3; q=b.p4;
    fprintf('\n--- Encoder feedback: %g RPM ---\n',c.reference_rpm);
    [d,s,e]=simulate(p5.model,c,q,p5,'encoder');
    [checks,m]=check_motioncore_phase5(mc,c,q,p5,d,s,e,true);
    delta=compare(b.s,s,p5,false); delta.target_rpm=c.reference_rpm;
    % Deliberate quantisation differences are reported, NOT identity-gated.
    delta=rmfield(delta,'pass'); changes=append(changes,delta);
    stem=fullfile(outDir,sprintf('speed_%g',c.reference_rpm));
    save([stem '.mat'],'mc','c','q','p5','d','s','e','checks','m','delta');
    writetable(d,[stem '.csv']); writetable(e,[stem '_encoder.csv']);
    requirePass(checks);
    assert(m.max_abs_current_A<=max(abs(b.d.current_A))+0.25, ...
        'Unexpected >0.25 A peak-current growth over Phase 4.');
    rows=append(rows,m); plotCase(d,s,b.d,stem);
end
fprintf('\n--- Phase 5 encoder-feedback saturation/recovery ---\n');
[d,s,e]=simulate(p5.model,stress,runtime4(p4,stress),p5,'encoder');
[checks,awMetrics]=check_motioncore_phase5(mc,stress,runtime4(p4,stress),p5,d,s,e,false);
save(fullfile(outDir,'encoder_antiwindup.mat'),'stress','d','s','e','checks','awMetrics'); requirePass(checks);
summary=struct2table(rows); differences=struct2table(changes);
writetable(summary,fullfile(outDir,'phase5_metrics.csv'));
writetable(differences,fullfile(outDir,'phase4_vs_phase5.csv'));
if runOptional,optionalExperiments(mc,p3,p4,p5,outDir);end
motioncore_phase5_structure(p5.model,baseStructure,true); verifyFiles(frozenFiles,before);
allPassed=true; %#ok<NASGU>
save(fullfile(outDir,'phase5_validation.mat'),'mc','p3','p4','p5','summary','differences','copyRows','allPassed','modelFile');
disp(summary(:,{'target_rpm','rise_time_s','settling_time_s','steady_error_rpm','tail_max_error_rpm', ...
    'max_abs_current_A','encoder_tail_ripple_rpm','encoder_rms_error_rpm'}));
fprintf('\nPHASE 5A PASS: prior regressions, copy identity, topology, physics, encoder and tracking passed.\n');
fprintf('%g PPR x%g = %g CPR; Tenc=%g s; resolution=%.7g RPM/count. PI unchanged.\n', ...
    p5.ENCODER_PPR,p5.decode_multiplier,p5.ENCODER_CPR,p5.Tenc,p5.rpm_per_count);
fprintf('Quantisation differences: %s\n',fullfile(outDir,'phase4_vs_phase5.csv'));
end

function [d,s,e]=simulate(mdl,c,q,p5,stage)
in=Simulink.SimulationInput(mdl);
in=in.setVariable('p3',c,'Workspace',mdl);
if ~strcmp(stage,'phase3'),in=in.setVariable('p4',q,'Workspace',mdl);end
if strcmp(stage,'encoder'),in=in.setVariable('p5',p5,'Workspace',mdl);end
in=in.setModelParameter('StopTime',num2str(c.stop_time_s,17),'ReturnWorkspaceOutputs','on');
result=sim(in); [d,s,e]=motioncore_phase5_logs(result,c,p5,stage);
end
function q=runtime4(q,c)
keys=intersect(fieldnames(q),fieldnames(c));
for k=1:numel(keys)
    n=keys{k}; if ~any(strcmp(n,{'model','check'})),q.(n)=c.(n);end
end
end
function r=compare(a,b,p5,is34)
assert(isequal(a.time_s,b.time_s),'Regression grids differ.');
names={'rpm','current_A','torque_Nm','back_emf_V','raw_voltage_V','voltage_V','integrator_V','measured_rpm'};
if ~is34,names=[names {'voltage_avg_V','voltage_command_sat_V','duty_cycle','reference_rpm'}];end
r=struct('pass',true);
for k=1:numel(names)
    n=names{k}; x=a.(n); y=b.(n); err=max(abs(x-y));
    r.(['max_abs_' n])=err; r.(['rms_' n])=sqrt(mean((x-y).^2));
    r.pass=r.pass && err<=p5.check.copy_absolute_tol+p5.check.copy_relative_tol*max(abs(x));
end
end
function rows=append(rows,row)
if isempty(rows),rows=row;else,rows(end+1)=row;end
end
function requirePass(c)
assert(~isempty(c) && all(strcmp({c.status},'PASS')), ...
    'MotionCore:Phase5Validation','Validation failed or skipped. Stop and inspect the printed checks.');
end
function b=readBytes(file)
f=fopen(file,'rb'); assert(f~=-1); g=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
end
function verifyFiles(files,before)
for k=1:numel(files)
    assert(isequal(before{k},readBytes(files{k})),'Frozen file changed: %s',files{k});
end
end
function plotCase(d,s,b,stem)
f=figure('Color','w','Name','Phase 5A encoder','Position',[60 60 1250 880]);
subplot(3,2,1); plot(d.time_s,d.reference_rpm,'k--',b.time_s,b.rpm,'g--',d.time_s,d.rpm,'b');
hold on; stairs(s.time_s,s.encoder_rpm,'Color',[0.8 0.3 0]); grid on; ylabel('RPM');
legend('Reference','Phase 4 true','Phase 5 true','Encoder','Location','best');
subplot(3,2,2); stairs(s.time_s,s.encoder_rpm_error); grid on; ylabel('Encoder - true (RPM)');
subplot(3,2,3); stairs(s.time_s,s.encoder_count); grid on; ylabel('Accumulated count');
subplot(3,2,4); stairs(s.time_s,s.encoder_delta_count); grid on; ylabel('Counts/window');
subplot(3,2,5); yyaxis left; stairs(s.time_s,s.voltage_command_sat_V); ylabel('Command (V)');
yyaxis right; stairs(s.time_s,s.duty_cycle); ylabel('Duty'); grid on; xlabel('Time (s)');
subplot(3,2,6); plot(d.time_s,d.current_A); grid on; ylabel('Current (A)'); xlabel('Time (s)');
savefig(f,[stem '.fig']); print(f,[stem '.png'],'-dpng','-r150'); %close(f);
end
function optionalExperiments(mc,p3,p4,p5,outDir)
% Simulate parameter overrides; never save them into the primary model.
rows=struct([]);
for ppr=p5.resolution_sweep_ppr
    q5=p5; q5.ENCODER_PPR=ppr; q5=motioncore_phase5_parameters(q5,p3);
    for target=p5.low_speeds_rpm
        c=p3; c.reference_rpm=target;
        [d,s,e]=simulate(p5.model,c,runtime4(p4,c),q5,'encoder');
        % Analysis only: apply the baseline tracking limits as a REPORTED
        % outcome; low-PPR experiments do not redefine the primary gate.
        [checks,m]=check_motioncore_phase5(mc,c,runtime4(p4,c),q5,d,s,e,true);
        m.baseline_limits_passed=all(strcmp({checks.status},'PASS')); rows=append(rows,m);
        stem=fullfile(outDir,sprintf('optional_%gPPR_%gRPM',ppr,target));
        save([stem '.mat'],'c','q5','d','s','e','checks','m'); writetable(s,[stem '.csv']);
    end
end
writetable(struct2table(rows),fullfile(outDir,'optional_resolution_low_speed.csv'));
% One figure for the combined experiment, without smoothing measured values.
tab=struct2table(rows); f=figure('Color','w','Position',[80 80 1050 480]);
for k=1:numel(p5.low_speeds_rpm)
    target=p5.low_speeds_rpm(k); select=tab.target_rpm==target;
    subplot(1,2,1); plot(tab.encoder_ppr(select),tab.encoder_tail_rms_error_rpm(select),'-o', ...
        'DisplayName',sprintf('%g RPM',target)); hold on;
    subplot(1,2,2); plot(tab.encoder_ppr(select),tab.steady_error_rpm(select),'-o', ...
        'DisplayName',sprintf('%g RPM',target)); hold on;
end
subplot(1,2,1); grid on; xlabel('PPR (x4 interpretation)'); ylabel('Encoder tail RMS error (RPM)'); legend('show');
subplot(1,2,2); grid on; xlabel('PPR (x4 interpretation)'); ylabel('Mean true-speed error (RPM)'); legend('show');
savefig(f,fullfile(outDir,'optional_resolution.fig'));
print(f,fullfile(outDir,'optional_resolution.png'),'-dpng','-r150'); close(f);
end
function r=recovery(d,p)
take=d.time_s>=p.second_step_s-1e-10; t=d.time_s(take); y=d.rpm(take);
target=p.reference_rpm+p.second_step_delta_rpm;
bad=find(abs(y-target)>0.02*abs(target));
if isempty(bad),k=1;else,k=bad(end)+1;end
r=NaN;
if k<=numel(t) && t(end)-t(k)>=p.check.hold_time_s,r=t(k)-p.second_step_s;end
end
