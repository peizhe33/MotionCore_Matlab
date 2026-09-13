function bases = motioncore_phase5b_preflight(mc,p3,p4,p5,b,outDir)
% Invoke frozen CHECKERS on frozen MODELS. Never invoke previous BUILDERS or
% RUNNERS because those can regenerate golden SLX files or prior results.
gold='MotionCore'; mw=get_param(gold,'ModelWorkspace'); mc0=getVariable(mw,'mc');
assert(isequaln(mc.motor,mc0.motor) && isequaln(mc.initial,mc0.initial));
signature=motioncore_plant_signature([gold '/DC_Motor_Plant']);
for mdl={'MotionCore_Phase3','MotionCore_Phase4','MotionCore_Phase5'}
    assert(isequaln(signature,motioncore_plant_signature([mdl{1} '/DC_Motor_Plant'])));
end
motioncore_phase5_structure('MotionCore_Phase4',[],false);
motioncore_phase5_structure('MotionCore_Phase5',[],true);
out=sim(gold,'ReturnWorkspaceOutputs','on'); c=p3; c.stop_time_s=mc0.sim.stop_time_s;
[d,~,~]=motioncore_phase5_logs(out,c,p5,'golden'); checks=check_motioncore(mc0,d);
save(fullfile(outDir,'frozen_open_loop.mat'),'d','checks'); mustPass(checks);
bases=cell(1,numel(b.test_speeds_rpm));
for k=1:numel(b.test_speeds_rpm)
    c=p3; c.reference_rpm=b.test_speeds_rpm(k); q=runtime4(p4,c);
    fprintf('\n--- Frozen Phase 3/4/5A: %g RPM ---\n',c.reference_rpm);
    [d3,s3]=motioncore_phase5b_simulate('MotionCore_Phase3',c,q,p5,b,'phase3');
    [checks3,~]=check_motioncore_phase3(mc,c,d3,s3,true); mustPass(checks3);
    [d4,s4]=motioncore_phase5b_simulate('MotionCore_Phase4',c,q,p5,b,'phase4');
    [checks4,~]=check_motioncore_phase4(mc,c,q,d4,s4,true); mustPass(checks4);
    reg34=motioncore_phase5b_compare(s3,s4,b,'basic'); assert(reg34.passed);
    [d5,s5,e5]=motioncore_phase5b_simulate('MotionCore_Phase5',c,q,p5,b,'encoder');
    [checks5,m5]=check_motioncore_phase5(mc,c,q,p5,d5,s5,e5,true); mustPass(checks5);
    bases{k}=struct('d',d5,'s',s5,'e',e5,'p3',c,'p4',q,'metrics',m5);
    save(fullfile(outDir,sprintf('frozen_%g.mat',c.reference_rpm)), ...
        'c','q','d3','s3','d4','s4','d5','s5','e5','checks3','checks4','checks5','reg34','m5');
end
stress=p3; stress.reference_rpm=3000; stress.second_step_delta_rpm=-2000; stress.stop_time_s=1.6;
for k=1:2
    c=stress; if k==2,c.Kb=0;end
    [d,s]=motioncore_phase5b_simulate('MotionCore_Phase3',c,runtime4(p4,c),p5,b,'phase3');
    [checks,m]=check_motioncore_phase3(mc,c,d,s,false); mustPass(checks);
    t=recovery(d,c);
    if k==1,on=m;onTime=t;else,off=m;offTime=t;end
end
if isnan(offTime),offTime=Inf;end
assert(on.saturation_occurred && isfinite(onTime) && onTime<0.5 && onTime<offTime && ...
    on.max_abs_integrator_V<off.max_abs_integrator_V && on.tail_max_error_rpm<=p3.check.steady_error_rpm);
q=runtime4(p4,stress);
[d,s]=motioncore_phase5b_simulate('MotionCore_Phase4',stress,q,p5,b,'phase4');
[checks,~]=check_motioncore_phase4(mc,stress,q,d,s,false); mustPass(checks);
[d,s,e]=motioncore_phase5b_simulate('MotionCore_Phase5',stress,q,p5,b,'encoder');
[checks,m]=check_motioncore_phase5(mc,stress,q,p5,d,s,e,false); mustPass(checks);
save(fullfile(outDir,'frozen_antiwindup.mat'),'stress','d','s','e','checks','m','on','off','onTime','offTime');
end
function q=runtime4(q,c)
keys=intersect(fieldnames(q),fieldnames(c));
for k=1:numel(keys),n=keys{k};if ~any(strcmp(n,{'model','check'})),q.(n)=c.(n);end,end
end
function mustPass(c)
assert(~isempty(c) && all(strcmp({c.status},'PASS')),'MotionCore:FrozenRegression', ...
    'A frozen regression failed or skipped. STOP; do not repair a previous phase automatically.');
end
function r=recovery(d,p)
take=d.time_s>=p.second_step_s-1e-10; t=d.time_s(take); y=d.rpm(take);
target=p.reference_rpm+p.second_step_delta_rpm; bad=find(abs(y-target)>0.02*abs(target));
if isempty(bad),k=1;else,k=bad(end)+1;end
r=NaN; if k<=numel(t) && t(end)-t(k)>=p.check.hold_time_s,r=t(k)-p.second_step_s;end
end
