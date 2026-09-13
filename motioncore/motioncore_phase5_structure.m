function snap = motioncore_phase5_structure(mdl,expected,encoderEnabled)
% Structural regression and explicit root-level logging/feedback connectivity.
if nargin<3,encoderEnabled=false;end
protected={'DC_Motor_Plant','Digital_PID','PWM_Driver','Voltage_Command','Reference'};
snap=struct();
for k=1:numel(protected)
    name=protected{k}; snap.(name)=motioncore_plant_signature([mdl '/' name]);
end
snap.load_value=get_param([mdl '/Load_Torque'],'Value');
snap.logging=motioncore_plant_signature([mdl '/Logging']);
ports=find_system([mdl '/Logging'],'SearchDepth',1,'BlockType','Inport');
snap.log_sources=struct();
for k=1:numel(ports)
    n=str2double(get_param(ports{k},'Port')); name=get_param(ports{k},'Name');
    snap.log_sources.(name)=source(mdl,'Logging',n);
end
if nargin>=2 && ~isempty(expected)
    for k=1:numel(protected)
        name=protected{k}; assert(isequaln(snap.(name),expected.(name)), ...
            'MotionCore:ChangedSubsystem','Protected subsystem changed: %s',name);
    end
    assert(isequal(snap.load_value,expected.load_value),'Load source changed.');
    names=fieldnames(expected.log_sources);
    for k=1:numel(names)
        name=names{k}; assert(isfield(snap.log_sources,name) && ...
            strcmp(snap.log_sources.(name),expected.log_sources.(name)), ...
            'MotionCore:LoggingWiring','Legacy root-level logging connection changed: %s',name);
    end
    for k=1:numel(expected.logging)
        item=expected.logging{k}; matches=cellfun(@(v)strcmp(v.path,item.path),snap.logging);
        assert(nnz(matches)==1 && isequaln(snap.logging{matches},item), ...
            'Legacy Logging block/wiring changed: %s',item.path);
    end
end
assert(strcmp(source(mdl,'Digital_PID',1),'Reference/1'));
assert(strcmp(source(mdl,'Digital_PID',2),'Feedback/1'));
assert(strcmp(source(mdl,'Digital_PID',3),'Voltage_Command/1'));
assert(strcmp(source(mdl,'DC_Motor_Plant',1),'PWM_Driver/1'));
assert(strcmp(source(mdl,'PWM_Driver',1),'Voltage_Command/1'));
assert(strcmp(source(mdl,'Voltage_Command',1),'Digital_PID/1'));
if encoderEnabled
    assert(strcmp(source(mdl,'Feedback',1),'RPM_Estimator/1'), ...
        'MotionCore:IdealFeedbackLeak','PI feedback must originate at RPM_Estimator.');
    assert(strcmp(source(mdl,'RPM_Estimator',1),'Encoder/1'));
    assert(strcmp(source(mdl,'Encoder',1),'DC_Motor_Plant/5'));
    expectedEdges={ ...
        'Feedback/Sample_Speed',1,'Feedback/Encoder_RPM/1'; ...
        'Feedback/Sampled_RPM',1,'Feedback/Sample_Speed/1'; ...
        'Encoder/Shaft_Angle',1,'Encoder/Omega_rad_s/1'; ...
        'Encoder/Sample_Angle',1,'Encoder/Shaft_Angle/1'; ...
        'Encoder/Counts_Per_Radian',1,'Encoder/Sample_Angle/1'; ...
        'Encoder/Integer_Count',1,'Encoder/Counts_Per_Radian/1'; ...
        'Encoder/Accumulated_Count',1,'Encoder/Integer_Count/1'; ...
        'RPM_Estimator/Previous_Count',1,'RPM_Estimator/Count/1'; ...
        'RPM_Estimator/Delta_Count',1,'RPM_Estimator/Count/1'; ...
        'RPM_Estimator/Delta_Count',2,'RPM_Estimator/Previous_Count/1'; ...
        'RPM_Estimator/RPM_Per_Count',1,'RPM_Estimator/Delta_Count/1'; ...
        'RPM_Estimator/RPM',1,'RPM_Estimator/RPM_Per_Count/1'};
    for k=1:size(expectedEdges,1)
        assert(strcmp(source(mdl,expectedEdges{k,1},expectedEdges{k,2}),expectedEdges{k,3}), ...
            'Encoder/feedback data-path mismatch: %s',expectedEdges{k,1});
    end
else
    assert(strcmp(source(mdl,'Feedback',1),'DC_Motor_Plant/1'),'Ideal-copy feedback differs from Phase 4.');
end
end
function name=source(mdl,block,n)
p=get_param([mdl '/' block],'PortHandles'); assert(numel(p.Inport)>=n);
line=get_param(p.Inport(n),'Line');
assert(line~=-1,'MotionCore:DisconnectedPort','Unconnected input: %s/%d',block,n);
h=get_param(line,'SrcPortHandle'); assert(h~=-1,'Unconnected source: %s',block);
parent=get_param(h,'Parent'); assert(startsWith(parent,[mdl '/']));
number=get_param(h,'PortNumber');
if ischar(number) || isstring(number),number=str2double(number);end
name=sprintf('%s/%d',parent(numel(mdl)+2:end),number);
end
