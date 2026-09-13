function summary = run_motioncore_phase5b(root)
% Final behavioral phase only. Build and validate a separate Phase 5B model.
if nargin<1 || isempty(root),root=fileparts(mfilename('fullpath'));end
[ok,a]=fileattrib(root); assert(ok && a.directory); root=a.Name;
oldPath=path; pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(root); names={'MotionCore','MotionCore_Phase3','MotionCore_Phase4','MotionCore_Phase5'};
files=cellfun(@(n)fullfile(root,[n '.slx']),names,'UniformOutput',false);
for k=1:numel(files)
    assert(isfile(files{k}),'MotionCore:MissingBaseline','Missing validated model: %s',files{k});
    load_system(files{k});
    assert(strcmpi(get_param(names{k},'FileName'),files{k}) && strcmp(get_param(names{k},'Dirty'),'off'), ...
        'Save/close conflicting or unsaved baseline: %s',names{k});
end
frozenFiles=files; scripts=dir(fullfile(root,'*.m'));
for k=1:numel(scripts)
    if ~contains(lower(scripts(k).name),'phase5b'),frozenFiles{end+1}=fullfile(root,scripts(k).name);end %#ok<AGROW>
end
bytes=cellfun(@readBytes,frozenFiles,'UniformOutput',false);
freezeGuard=onCleanup(@()verifyFrozen(frozenFiles,bytes)); %#ok<NASGU>
mw=get_param('MotionCore_Phase5','ModelWorkspace');
mc=getVariable(mw,'mc'); p3=getVariable(mw,'p3'); p4=getVariable(mw,'p4'); p5=getVariable(mw,'p5');
for mdl={'MotionCore_Phase3','MotionCore_Phase4'}
    saved=getVariable(get_param(mdl{1},'ModelWorkspace'),'p3');
    for key={'Ts','Kp','Ki','Kd','Kb','A','B','integrator_initial_V','voltage_min_V','voltage_max_V'}
        assert(isequaln(saved.(key{1}),p3.(key{1})),'Frozen controller parameter mismatch: %s',key{1});
    end
end
b=motioncore_phase5b_init(mc,p3,p4,p5);
parent=fullfile(root,'results_phase5b'); if ~isfolder(parent),mkdir(parent);end
outDir=tempname(parent); mkdir(outDir);
fprintf('\nPhase 5B results: %s\n',outDir);
fprintf('Edges: %.3f Hz at 2000 RPM; %.3f Hz at no-load %.3f RPM.\n', ...
    b.transition_rate_2000_Hz,b.transition_rate_no_load_Hz,b.no_load_rpm);
fprintf('Tedge=%g s; Tenc=%g s; PI Ts=%g s. No extra snapshot pipeline delay.\n',b.Tedge,p5.Tenc,p3.Ts);

% STOP immediately if any prior golden gate fails. Nothing is rebuilt there.
bases=motioncore_phase5b_preflight(mc,p3,p4,p5,b,outDir);
goldStructure=motioncore_phase5b_structure('MotionCore_Phase5',[],false);
modelFile=build_motioncore_phase5b(files{4},b,'copy');
motioncore_phase5b_structure(b.model,goldStructure,false);
copyRows=struct([]);
for k=1:numel(bases)
    base=bases{k};
    [~,s]=motioncore_phase5b_simulate(b.model,base.p3,base.p4,p5,b,'encoder');
    r=motioncore_phase5b_compare(base.s,s,b,'copy'); r.target_rpm=base.p3.reference_rpm;
    copyRows=append(copyRows,r); assert(r.passed,'Untouched Phase 5A/5B copy identity failed.');
end
writetable(struct2table(copyRows),fullfile(outDir,'copy_identity.csv'));
[decoderTests,prototypes]=test_motioncore_phase5b_decoder(b,p5,outDir);
assert(all(decoderTests.passed));
build_motioncore_phase5b(files{4},b,'quadrature');
verifyStructure();
rows=struct([]); comparisons=struct([]); representative='';
for k=1:numel(bases)
    base=bases{k}; c=base.p3; q=base.p4;
    fprintf('\n--- Phase 5B quadrature feedback: %g RPM ---\n',c.reference_rpm);
    [d,s,e,f]=motioncore_phase5b_simulate(b.model,c,q,p5,b,'quadrature');
    [checks,m]=check_motioncore_phase5b(mc,c,q,p5,b,d,s,e,f,true);
    compare=motioncore_phase5b_compare(base.s,s,b,'quadrature'); compare.target_rpm=c.reference_rpm;
    stem=fullfile(outDir,sprintf('speed_%g',c.reference_rpm));
    save([stem '.mat'],'mc','c','q','p5','b','d','s','e','f','checks','m','compare','-v7.3');
    writetable(s,[stem '_controller.csv']); writetable(e,[stem '_estimator.csv']);
    % Every A/B tick is retained in MAT; CSV zoom is convenient for inspection.
    zoom=f.time_s>=b.zoom_start_s & f.time_s<=b.zoom_start_s+b.zoom_duration_s;
    writetable(f(zoom,:),[stem '_AB_zoom.csv']);
    mustPass(checks); assert(compare.passed,'Phase 5A/5B response comparison exceeded its limits.');
    assert(m.max_abs_integrator_V<max(20,2*base.metrics.max_abs_integrator_V), ...
        'Unexpected integrator growth compared with Phase 5A.');
    rows=append(rows,m); comparisons=append(comparisons,compare);
    plotCase(d,s,f,base.s,b,stem);
    if c.reference_rpm==1000,representative=[stem '.fig'];end
    clear d s e f
end
stress=p3; stress.reference_rpm=3000; stress.second_step_delta_rpm=-2000; stress.stop_time_s=1.6;
fprintf('\n--- Quadrature-feedback anti-windup stress: 3000 -> 1000 RPM ---\n');
[d,s,e,f]=motioncore_phase5b_simulate(b.model,stress,p4,p5,b,'quadrature');
[checks,awMetrics]=check_motioncore_phase5b(mc,stress,p4,p5,b,d,s,e,f,false);
[~,awBase]=motioncore_phase5b_simulate('MotionCore_Phase5',stress,p4,p5,b,'encoder');
awCompare=motioncore_phase5b_compare(awBase,s,b,'quadrature');
save(fullfile(outDir,'antiwindup_quadrature.mat'),'stress','d','s','e','f','checks','awMetrics','awCompare','-v7.3');
mustPass(checks); assert(awCompare.passed);
assert(awMetrics.max_abs_integrator_V<max(20,2*max(abs(awBase.integrator_V))), ...
    'Quadrature feedback caused unexpected integrator growth in the saturation test.');
clear d s e f
verifyStructure(); motioncore_phase5b_structure('MotionCore_Phase5',goldStructure,false);
verifyFrozen(frozenFiles,bytes);
summary=struct2table(rows); comparison=struct2table(comparisons);
writetable(summary,fullfile(outDir,'phase5b_metrics.csv'));
writetable(comparison,fullfile(outDir,'phase5a_vs_phase5b.csv'));
allPassed=true; %#ok<NASGU>
save(fullfile(outDir,'phase5b_validation.mat'),'mc','p3','p4','p5','b','summary','comparison', ...
    'decoderTests','copyRows','awMetrics','awCompare','allPassed','modelFile');
disp(summary(:,{'target_rpm','rise_time_s','settling_time_s','steady_error_rpm', ...
    'tail_max_error_rpm','overshoot_pct','max_abs_current_A','encoder_rms_error_rpm', ...
    'encoder_tail_ripple_rpm','max_local_count_difference','invalid_transition_count'}));
fprintf('\nPHASE 5B PASS: earlier regressions, x4 decoder, topology, physics, tracking and anti-windup passed.\n');
fprintf('Behavioral feature development stops here. Next: RTL unit verification and integration.\n');
if ~isempty(representative),openfig(representative,'new','visible');end
    function verifyStructure()
        motioncore_phase5b_structure(b.model,goldStructure,true);
        assert(isequaln(motioncore_plant_signature([b.model '/Quadrature_Decoder']),prototypes.decoder), ...
            'Closed-loop decoder differs from standalone tested decoder.');
        assert(isequaln(motioncore_plant_signature([b.model '/Quadrature_AB']),prototypes.generator), ...
            'Closed-loop A/B generator differs from standalone tested generator.');
    end
end
function rows=append(rows,row)
if isempty(rows),rows=row;else,rows(end+1)=row;end
end
function mustPass(checks)
assert(~isempty(checks) && all(strcmp({checks.status},'PASS')), ...
    'MotionCore:Phase5BValidation','Phase 5B check failed. Results retained; inspect the first FAIL.');
end
function bytes=readBytes(file)
f=fopen(file,'rb'); assert(f~=-1); guard=onCleanup(@()fclose(f)); %#ok<NASGU>
bytes=fread(f,Inf,'*uint8');
end
function verifyFrozen(files,bytes)
for k=1:numel(files),assert(isequal(bytes{k},readBytes(files{k})),'Frozen file changed: %s',files{k});end
end
function plotCase(d,s,f,base,b,stem)
% Render plots from a compact sample selection; checks use ALL logged data.
fig=figure('Color','w','Visible','off','Position',[60 60 1250 920]);
subplot(3,2,1); plot(s.time_s,s.reference_rpm,'k--',s.time_s,s.rpm,'b'); hold on;
stairs(base.time_s,base.encoder_rpm,'g:'); stairs(s.time_s,s.encoder_rpm,'Color',[0.8 0.3 0]);
grid on; ylabel('RPM'); legend('Reference','True','5A encoder','5B encoder','Location','best');
subplot(3,2,2); take=f.time_s>=b.zoom_start_s & f.time_s<=b.zoom_start_s+b.zoom_duration_s;
x=(f.time_s(take)-b.zoom_start_s)*1e6;
stairs(x,f.encoder_A(take)+2,'b'); hold on; stairs(x,f.encoder_B(take),'r');
yticks([0 1 2 3]); yticklabels({'B=0','B=1','A=0','A=1'}); ylim([-.2 3.2]); grid on;
xlabel(sprintf('Microseconds after %.3f s',b.zoom_start_s)); title('A/B zoom; forward B leads A');
subplot(3,2,3); stairs(s.time_s,s.decoded_encoder_count,'b'); hold on;
stairs(s.time_s,s.ideal_encoder_count_phase5a,'r--'); grid on; ylabel('Count (1 ms snapshots)');
legend('Decoded','Same-trajectory ideal','Location','best');
subplot(3,2,4); stairs(s.time_s,s.count_difference_vs_phase5a); grid on; ylabel('Decoded - ideal count');
subplot(3,2,5); stairs(s.time_s,s.encoder_rpm_error); grid on; ylabel('Encoder - true (RPM)'); xlabel('Time (s)');
subplot(3,2,6); yyaxis left; stairs(s.time_s,s.voltage_command_sat_V); ylabel('Voltage (V)');
yyaxis right; pick=unique(round(linspace(1,height(d),min(20000,height(d)))));
plot(d.time_s(pick),d.current_A(pick)); ylabel('Current (A)'); grid on; xlabel('Time (s)');
title(sprintf('Duty range %.3f to %.3f',min(s.duty_cycle),max(s.duty_cycle)));

drawnow;

if isgraphics(fig)
    savefig(fig,[stem '.fig']);

    try
        exportgraphics(fig,[stem '.png'],'Resolution',150);
    catch ME
        warning('MotionCore:PlotExport', ...
            'PNG export failed: %s',ME.message);
    end
end

if isgraphics(fig)
    close(fig);
end
end
