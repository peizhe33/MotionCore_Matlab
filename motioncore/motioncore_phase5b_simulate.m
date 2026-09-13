function [d,s,e,f] = motioncore_phase5b_simulate(mdl,c,p4,p5,b,stage)
% Per-test controller/reference struct c is always passed explicitly.
q=p4; keys=intersect(fieldnames(q),fieldnames(c));
for k=1:numel(keys)
    n=keys{k}; if ~any(strcmp(n,{'model','check'})),q.(n)=c.(n);end
end
input=Simulink.SimulationInput(mdl); input=input.setVariable('p3',c,'Workspace',mdl);
if ~strcmp(stage,'phase3'),input=input.setVariable('p4',q,'Workspace',mdl);end
if any(strcmp(stage,{'encoder','quadrature'})),input=input.setVariable('p5',p5,'Workspace',mdl);end
if strcmp(stage,'quadrature'),input=input.setVariable('b',b,'Workspace',mdl);end
input=input.setModelParameter('StopTime',num2str(c.stop_time_s,17),'ReturnWorkspaceOutputs','on');
out=sim(input); f=[];
if strcmp(stage,'quadrature'),[d,s,e,f]=motioncore_phase5b_logs(out,c,p5,b);
else,[d,s,e]=motioncore_phase5_logs(out,c,p5,stage);end
end
