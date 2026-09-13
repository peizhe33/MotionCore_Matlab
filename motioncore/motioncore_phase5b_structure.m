function snap = motioncore_phase5b_structure(mdl,expected,enabled)
% Freeze every legacy subsystem, including the full Phase 5A Encoder and
% RPM_Estimator. Only two root branches change: estimator and count log.
if nargin<3,enabled=false;end
protected={'DC_Motor_Plant','Digital_PID','PWM_Driver','Voltage_Command', ...
    'Reference','Encoder','RPM_Estimator','Feedback'};
snap=struct();
for k=1:numel(protected)
    n=protected{k}; snap.(n)=motioncore_plant_signature([mdl '/' n]);
end
snap.logging=motioncore_plant_signature([mdl '/Logging']);
snap.load_value=get_param([mdl '/Load_Torque'],'Value');
ports=find_system([mdl '/Logging'],'SearchDepth',1,'BlockType','Inport');
snap.log_sources=struct();
for k=1:numel(ports)
    n=get_param(ports{k},'Name'); p=str2double(get_param(ports{k},'Port'));
    snap.log_sources.(n)=source(mdl,'Logging',p);
end
if nargin>=2 && ~isempty(expected)
    for k=1:numel(protected)
        n=protected{k}; assert(isequaln(snap.(n),expected.(n)),'Protected subsystem differs: %s',n);
    end
    assert(strcmp(snap.load_value,expected.load_value));
    for k=1:numel(expected.logging)
        item=expected.logging{k}; match=cellfun(@(v)strcmp(v.path,item.path),snap.logging);
        assert(nnz(match)==1 && isequaln(snap.logging{match},item),'Legacy logging block changed: %s',item.path);
    end
    names=fieldnames(expected.log_sources);
    for k=1:numel(names)
        n=names{k}; wanted=expected.log_sources.(n);
        if enabled && strcmp(n,'encoder_count'),wanted='Decoded_Count_Snapshot/1';end
        assert(strcmp(snap.log_sources.(n),wanted),'Legacy log source mismatch: %s',n);
    end
end
edges={'Digital_PID',1,'Reference/1';'Digital_PID',2,'Feedback/1'; ...
    'Digital_PID',3,'Voltage_Command/1';'Feedback',1,'RPM_Estimator/1'; ...
    'Voltage_Command',1,'Digital_PID/1';'PWM_Driver',1,'Voltage_Command/1'; ...
    'DC_Motor_Plant',1,'PWM_Driver/1';'Encoder',1,'DC_Motor_Plant/5'};
if enabled
    edges=[edges; {'RPM_Estimator',1,'Decoded_Count_Snapshot/1'; ...
        'Decoded_Count_Snapshot',1,'Quadrature_Decoder/1'; ...
        'Decoded_Count_Snapshot/Snapshot',1,'Decoded_Count_Snapshot/Decoded_Count/1'; ...
        'Decoded_Count_Snapshot/To_Estimator_Double',1,'Decoded_Count_Snapshot/Snapshot/1'; ...
        'Decoded_Count_Snapshot/Count',1,'Decoded_Count_Snapshot/To_Estimator_Double/1'; ...
        'Quadrature_Decoder',1,'Quadrature_AB/1';'Quadrature_Decoder',2,'Quadrature_AB/2'; ...
        'Quadrature_AB',1,'Encoder/2'}];
    % Compare decoder/generator internals to a fresh instance of the exact
    % shared factories that also construct the standalone decoder test.
    p=get_param([mdl '/Quadrature_Decoder'],'PortHandles'); assert(numel(p.Inport)==2);
    assert(strcmp(snap.log_sources.ideal_encoder_count_phase5a,'Encoder/1'));
else
    edges=[edges; {'RPM_Estimator',1,'Encoder/1'}];
end
for k=1:size(edges,1)
    assert(strcmp(source(mdl,edges{k,1},edges{k,2}),edges{k,3}), ...
        'MotionCore:Phase5BTopology','Unexpected feedback/plant source at %s/%d',edges{k,1},edges{k,2});
end
end
function n=source(mdl,block,p)
h=get_param([mdl '/' block],'PortHandles'); assert(numel(h.Inport)>=p);
line=get_param(h.Inport(p),'Line'); assert(line~=-1,'Disconnected input: %s/%d',block,p);
src=get_param(line,'SrcPortHandle'); assert(src~=-1);
parent=get_param(src,'Parent'); assert(startsWith(parent,[mdl '/']));
number=get_param(src,'PortNumber'); if ischar(number)||isstring(number),number=str2double(number);end
n=sprintf('%s/%d',parent(numel(mdl)+2:end),number);
end
