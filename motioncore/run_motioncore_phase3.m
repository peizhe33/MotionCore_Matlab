function summary=run_motioncore_phase3(baselineFile)
% Place all Phase 3 .m files beside the EXISTING validated MotionCore.slx,
% set that folder as MATLAB Current Folder, then run run_motioncore_phase3.
% Alternatively pass an absolute path to the golden model as the argument.
% Golden sources and model are never rebuilt or saved by this runner.
if nargin<1,baselineFile=fullfile(fileparts(mfilename('fullpath')),'MotionCore.slx');end
assert(exist(baselineFile,'file')==4,'MotionCore:MissingGoldenModel', ...
    'Place Phase 3 scripts beside your existing validated MotionCore.slx.');
[ok,attrs]=fileattrib(baselineFile); assert(ok); baselineFile=attrs.Name;
root=fileparts(baselineFile); [~,base]=fileparts(baselineFile);
oldPath=path; pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(root);
assert(exist('check_motioncore','file')==2 && exist('motioncore_reference','file')==2, ...
    'Keep your original Phase 1-2 validation scripts on the MATLAB path.');
baselineBytes=readBytes(baselineFile);
load_system(baselineFile);
assert(strcmpi(get_param(base,'FileName'),baselineFile),'Another baseline model is loaded.');
assert(strcmp(get_param(base,'Dirty'),'off'),'Save your golden model before running Phase 3.');
mw=get_param(base,'ModelWorkspace'); mc=getVariable(mw,'mc');
goldenSignature=motioncore_plant_signature([base '/DC_Motor_Plant']);
outDir=fullfile(root,'results_phase3'); if ~exist(outDir,'dir'),mkdir(outDir);end

fprintf('\n--- Golden open-loop regression (original checker, unchanged) ---\n');
goldenOut=sim(base,'ReturnWorkspaceOutputs','on');
goldenData=collectGolden(goldenOut);
goldenChecks=check_motioncore(mc,goldenData);
save(fullfile(outDir,'golden_regression.mat'),'mc','goldenData','goldenChecks');
assert(all(strcmp({goldenChecks.status},'PASS')), ...
    'MotionCore:GoldenRegression','Golden baseline has FAIL/SKIP checks. Stop here.');

p3=motioncore_phase3_init(mc);
fprintf('\nTs=%.6g s, Kp=%.9g, Ki=%.9g, Kd=%.9g, Kb=%.6g 1/s\n', ...
    p3.Ts,p3.Kp,p3.Ki,p3.Kd,p3.Kb);
fprintf('\n--- Sampled tuning comparison (exact reference, Kd=0) ---\n');
tuning=tune_controller_phase3(mc,p3);
writetable(tuning,fullfile(outDir,'tuning_comparison.csv'));
modelFile=build_motioncore_phase3(baselineFile,mc,p3);
assert(isequaln(goldenSignature,motioncore_plant_signature([p3.model '/DC_Motor_Plant'])), ...
    'MotionCore:PlantCopy','Copied plant structure or parameters differ from golden.');

rows=struct([]); allPassed=true; runs=cell(1,numel(p3.test_speeds_rpm));
fig=figure('Name','MotionCore Phase 3 speed sweep','Color','w','Position',[80 60 1200 850]);
for k=1:numel(p3.test_speeds_rpm)
    c=p3; c.reference_rpm=p3.test_speeds_rpm(k);
    fprintf('\n--- %g RPM reference ---\n',c.reference_rpm);
    [d,s]=runCase(c); [checks,m]=check_motioncore_phase3(mc,c,d,s,true);
    allPassed=allPassed && all(strcmp({checks.status},'PASS'));
    if isempty(rows)
        rows = m;
    else
        rows(end+1) = m; %#ok<AGROW>
    end

    runs{k} = d;

    stem=sprintf('speed_%g_rpm',c.reference_rpm);
    writetable(d,fullfile(outDir,[stem '.csv']));
    save(fullfile(outDir,[stem '.mat']),'mc','c','d','s','checks','m');
    subplot(4,2,2*k-1); plot(d.time_s,d.reference_rpm,'k--',d.time_s,d.rpm,'b');
    grid on; ylabel('RPM'); title(sprintf('%g RPM: speed tracking',c.reference_rpm));
    xlabel('Time (s)'); legend('Reference','Actual','Location','southeast');
    subplot(4,2,2*k); yyaxis left; plot(d.time_s,d.voltage_V,'b'); ylabel('Voltage (V)');
    yyaxis right; plot(d.time_s,d.current_A,'r'); ylabel('Current (A)'); grid on;
    xlabel('Time (s)'); title('Applied voltage and armature current');
end
summary=struct2table(rows); disp(summary);
writetable(summary,fullfile(outDir,'phase3_metrics.csv'));
savefig(fig,fullfile(outDir,'speed_sweep.fig'));
print(fig,fullfile(outDir,'speed_sweep.png'),'-dpng','-r150');

% A reference saturation test, not a load disturbance or Phase 4 feature.
stress=p3; stress.reference_rpm=3000; stress.second_step_delta_rpm=-2000;
stress.stop_time_s=1.6;
fprintf('\n--- Unreachable 3000 RPM then 1000 RPM: anti-windup ON ---\n');
[awD,awS]=runCase(stress);
[awChecks,awMetrics]=check_motioncore_phase3(mc,stress,awD,awS,false);
noaw=stress; noaw.Kb=0;
fprintf('\n--- Same test: anti-windup OFF (intentional comparison) ---\n');
[offD,offS]=runCase(noaw);
[offChecks,offMetrics]=check_motioncore_phase3(mc,noaw,offD,offS,false);
% Compare recovery into +/-2% of the final 1000 RPM (same band for both).
awRecovery=recovery(awD,stress); offRecovery=recovery(offD,stress);
offComparison=offRecovery; if isnan(offComparison),offComparison=Inf;end
awPass=awMetrics.saturation_occurred && isfinite(awRecovery) && ...
    awRecovery<0.5 && awRecovery<offComparison && ...
    awMetrics.max_abs_integrator_V<offMetrics.max_abs_integrator_V && ...
    awMetrics.tail_max_error_rpm<=p3.check.steady_error_rpm;
fprintf('AW recovery %.4f s; no-AW recovery %.4f s. Peak |I state| %.3f vs %.3f V.\n', ...
    awRecovery,offRecovery,awMetrics.max_abs_integrator_V,offMetrics.max_abs_integrator_V);
fprintf('Anti-windup recovery check: %s\n',passText(awPass));
allPassed=allPassed && all(strcmp({awChecks.status},'PASS')) && ...
    all(strcmp({offChecks.status},'PASS')) && awPass;
save(fullfile(outDir,'antiwindup_comparison.mat'),'stress','noaw','awD','offD', ...
    'awChecks','offChecks','awMetrics','offMetrics','awRecovery','offRecovery','awPass');
writetable(awD,fullfile(outDir,'antiwindup_on.csv'));
writetable(offD,fullfile(outDir,'antiwindup_off.csv'));
af=figure('Name','MotionCore Phase 3 anti-windup','Color','w','Position',[90 80 1050 750]);
subplot(3,1,1);plot(awD.time_s,awD.reference_rpm,'k--',awD.time_s,awD.rpm,'b',offD.time_s,offD.rpm,'r');
grid on;ylabel('RPM');legend('Reference','AW on','AW off','Location','best');
subplot(3,1,2);plot(awD.time_s,awD.integrator_V,'b',offD.time_s,offD.integrator_V,'r');
grid on;ylabel('Integral state (V)');
subplot(3,1,3);plot(awD.time_s,awD.voltage_V,'b',offD.time_s,offD.voltage_V,'r');
grid on;ylabel('Applied V');xlabel('Time (s)');
savefig(af,fullfile(outDir,'antiwindup_comparison.fig'));
print(af,fullfile(outDir,'antiwindup_comparison.png'),'-dpng','-r150');

assert(isequal(baselineBytes,readBytes(baselineFile)), ...
    'MotionCore:BaselineChanged','The golden SLX bytes changed during this run.');
assert(isequaln(goldenSignature,motioncore_plant_signature([base '/DC_Motor_Plant'])), ...
    'The original golden plant changed in memory.');
save(fullfile(outDir,'phase3_validation.mat'),'p3','mc','summary','tuning','allPassed','modelFile');
assert(allPassed,'MotionCore:Phase3ValidationFailed', ...
    'Phase 3 checks failed. Results are saved; inspect them before proceeding.');
fprintf('\nPHASE 3 PASS. Golden model unchanged. Results: %s\n',outDir);

    function [d,s]=runCase(c)
        % Temporary SimulationInput override: saved default model is unchanged.
        input=Simulink.SimulationInput(p3.model);
        input=input.setVariable('p3',c,'Workspace',p3.model);
        input=input.setModelParameter('StopTime',num2str(c.stop_time_s,17));
        result=sim(input); [d,s]=motioncore_phase3_logs(result,c);
    end
end

function d=collectGolden(o)
sig=o.get('omega_rad_s'); [t,~]=unique(double(sig.Time(:)),'last');
d=table(t,'VariableNames',{'time_s'});
names={'rpm','current_A','torque_Nm','back_emf_V','omega_rad_s','voltage_V','load_Nm'};
for k=1:numel(names)
    sig=o.get(names{k});[st,idx]=unique(double(sig.Time(:)),'last');v=double(sig.Data(:));v=v(idx);
    if numel(st)==1,d.(names{k})=repmat(v,numel(t),1);
    elseif isequal(st,t),d.(names{k})=v;
    else
        method='linear';if any(strcmp(names{k},{'voltage_V','load_Nm'})),method='previous';end
        d.(names{k})=interp1(st,v,t,method,'extrap');
    end
end
end
function b=readBytes(file)
f=fopen(file,'rb');assert(f~=-1);guard=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
end
function r=recovery(d,p)
% Report time from downward reference step to remaining within +/-20 RPM.
take=d.time_s>=p.second_step_s-1e-10;t=d.time_s(take);y=d.rpm(take);
target=p.reference_rpm+p.second_step_delta_rpm;
bad=find(abs(y-target)>0.02*abs(target));
if isempty(bad),k=1;else,k=bad(end)+1;end
r=NaN;if k<=numel(t) && t(end)-t(k)>=p.check.hold_time_s,r=t(k)-p.second_step_s;end
end
function t=passText(pass)
t='FAIL';if pass,t='PASS';end
end
