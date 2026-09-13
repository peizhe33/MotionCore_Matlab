function [summary,signatures] = test_motioncore_phase5b_decoder(b,p5,outDir)
% Exercise the ACTUAL block factories, including all 16 transition pairs,
% both directions, holds, invalid transitions/resynchronization and one turn.
[~,suffix]=fileparts(tempname); mdl=['MC5B_Test_' suffix];
load_system('simulink'); new_system(mdl); guard=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','b.Tedge','ReturnWorkspaceOutputs','on','SaveOutput','off','SignalLogging','off');
mw=get_param(mdl,'ModelWorkspace'); assignin(mw,'b',b); assignin(mw,'p5',p5);
motioncore_phase5b_decoder([mdl '/Decoder']);
signatures.decoder=motioncore_plant_signature([mdl '/Decoder']);
add_block('simulink/Sources/From Workspace',[mdl '/Input_A'], ...
    'VariableName','test_A', ...
    'Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value', ...
    'SampleTime','b.Tedge');

add_block('simulink/Sources/From Workspace',[mdl '/Input_B'], ...
    'VariableName','test_B', ...
    'Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value', ...
    'SampleTime','b.Tedge');
add_line(mdl,'Input_A/1','Decoder/1'); add_line(mdl,'Input_B/1','Decoder/2');
lognames={'test_count','test_state','test_previous','test_direction','test_invalid'};
for k=1:numel(lognames),logSignal(mdl,['Decoder/' num2str(k)],lognames{k});end
sequences={[0 1 3 2 0],[0 2 3 1 0],[0 0 0 0],[0 3 2 0],[0 1 3 2 0 2 3 1 0]};
labels={'forward','reverse','hold','invalid_then_resync','direction_reversal'};
rows=struct([]);
for k=1:numel(sequences)
    row=runSequence(sequences{k},labels{k}); rows=append(rows,row);
end
for prev=0:3
    for curr=0:3
        row=runSequence([prev curr],sprintf('transition_%d_%d',prev,curr)); rows=append(rows,row);
    end
end

% Replace input stimulus with the actual A/B generator; validate one full
% mechanical revolution in each direction, 8 observations per quarter-cycle.
delete_line(mdl,'Input_A/1','Decoder/1'); delete_line(mdl,'Input_B/1','Decoder/2');
delete_block([mdl '/Input_A']); delete_block([mdl '/Input_B']);
motioncore_phase5b_generator([mdl '/Generator']);
signatures.generator=motioncore_plant_signature([mdl '/Generator']);
add_block('simulink/Sources/From Workspace',[mdl '/Input_Angle'], ...
    'VariableName','test_angle', ...
    'Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value', ...
    'SampleTime','b.Tedge');
add_line(mdl,'Input_Angle/1','Generator/1');
add_line(mdl,'Generator/1','Decoder/1'); add_line(mdl,'Generator/2','Decoder/2');
logSignal(mdl,'Generator/1','test_generated_A'); logSignal(mdl,'Generator/2','test_generated_B');
logSignal(mdl,'Generator/3','test_ideal');
N=b.CPR*8; t=(0:N).'*b.Tedge;
for direction=[1 -1]
    theta=(0.3+direction*(0:N).'/8)*2*pi/b.CPR;
    c=b; c.initial_state=uint8(0); c.initial_count=int64(0);
    input=Simulink.SimulationInput(mdl);
    input=input.setVariable('b',c,'Workspace',mdl);
    input=input.setVariable('test_angle',timeseries(theta,t),'Workspace',mdl);
    input=input.setModelParameter('StopTime',num2str(t(end),17)); out=sim(input);
    count=read(out,'test_count',t); ideal=read(out,'test_ideal',t);
    A=read(out,'test_generated_A',t); B=read(out,'test_generated_B',t);
    invalid=read(out,'test_invalid',t);
    assert(isequal(count,ideal) && count(end)-count(1)==direction*b.CPR && all(invalid==0));
    assert(all(ismember(A,[0 1])) && all(ismember(B,[0 1])));
    assert(mean(A(1:end-1))==0.5 && mean(B(1:end-1))==0.5,'A/B duty cycle is not 50 percent.');
    assert(isequal(B(1:end-1),circshift(A(1:end-1),-direction*8)), ...
        'A/B is not one quarter electrical cycle apart.');
    row=struct('name',sprintf('one_turn_direction_%d',direction),'samples',numel(t), ...
        'final_count',count(end),'invalid_count',invalid(end),'passed',true);
    rows=append(rows,row);
end
summary=struct2table(rows); writetable(summary,fullfile(outDir,'decoder_unit_tests.csv'));
save(fullfile(outDir,'decoder_unit_tests.mat'),'summary','signatures');
disp(summary);
    function row=runSequence(states,label)
        t=(0:numel(states)-1).'*b.Tedge; c=b;
        c.initial_state=uint8(states(1)); c.initial_count=int64(0);
        input=Simulink.SimulationInput(mdl);
        input=input.setVariable('b',c,'Workspace',mdl);
        input=input.setVariable('test_A',timeseries(logical(states(:)>=2),t),'Workspace',mdl);
        input=input.setVariable('test_B',timeseries(logical(mod(states(:),2)),t),'Workspace',mdl);
        input=input.setModelParameter('StopTime',num2str(t(end),17)); out=sim(input);
        count=read(out,'test_count',t); current=read(out,'test_state',t);
        previous=read(out,'test_previous',t); step=read(out,'test_direction',t);
        invalid=read(out,'test_invalid',t);
        [wanted,wstep,wbad,wprev]=motioncore_phase5b_decode_reference(states,c.initial_state,0);
        assert(isequal(current,states(:)) && isequal(previous,wprev) && isequal(count,wanted) && ...
            isequal(step,wstep) && isequal(invalid,cumsum(double(wbad))),'Decoder test failed: %s',label);
        row=struct('name',label,'samples',numel(t),'final_count',count(end),'invalid_count',invalid(end),'passed',true);
    end
end
function v=read(out,name,t)
assert(any(strcmp(who(out),name)),'Missing test log %s',name); sig=out.get(name);
[st,idx]=unique(double(sig.Time(:)),'last'); data=double(sig.Data(:));
assert(numel(st)==numel(t) && max(abs(st-t))<1e-12); v=data(idx);
end
function logSignal(mdl,src,name)
add_block('simulink/Sinks/To Workspace',[mdl '/' name],'VariableName',name, ...
    'SaveFormat','Timeseries','MaxDataPoints','inf','SampleTime','-1');
add_line(mdl,src,[name '/1'],'autorouting','on');
end
function rows=append(rows,row)
if isempty(rows),rows=row;else,rows(end+1)=row;end
end
