function file = build_motioncore_phase5b(goldenFile,b,stage)
% Full-file copy first; run copy regression before stage='quadrature'.
if nargin<3,stage='copy';end
assert(isfile(goldenFile),'Validated MotionCore_Phase5.slx is required.');
[ok,a]=fileattrib(goldenFile); assert(ok); goldenFile=a.Name;
[root,src]=fileparts(goldenFile); mdl=b.model; file=fullfile(root,[mdl '.slx']);
assert(strcmp(src,'MotionCore_Phase5') && strcmp(mdl,'MotionCore_Phase5B'));
if strcmp(stage,'copy')
    load_system(goldenFile);
    assert(strcmpi(get_param(src,'FileName'),goldenFile) && strcmp(get_param(src,'Dirty'),'off'));
    if bdIsLoaded(mdl)
        assert(strcmp(get_param(mdl,'Dirty'),'off'),'Save or close unsaved Phase 5B edits first.');
        assert(strcmpi(get_param(mdl,'FileName'),file),'A different Phase 5B file is loaded.');
        close_system(mdl,0);
    end
    if isfile(file)
        folder=fullfile(root,'phase5b_backups'); if ~isfolder(folder),mkdir(folder);end
        copyfile(file,[tempname(folder) '.slx']);
    end
    copyfile(goldenFile,file); load_system(file); return
end
assert(strcmp(stage,'quadrature') && bdIsLoaded(mdl));
assert(strcmpi(get_param(mdl,'FileName'),file));
assert(getSimulinkBlockHandle([mdl '/Quadrature_Decoder'])==-1,'Already built; use the runner to rebuild.');
mw=get_param(mdl,'ModelWorkspace'); assignin(mw,'b',b);
motioncore_phase5b_generator([mdl '/Quadrature_AB']);
motioncore_phase5b_decoder([mdl '/Quadrature_Decoder']);
set_param([mdl '/Quadrature_AB'],'Position',[740 645 910 730]);
set_param([mdl '/Quadrature_Decoder'],'Position',[465 640 665 760]);
q=[mdl '/Decoded_Count_Snapshot']; add_block('built-in/SubSystem',q,'Position',[260 490 435 575]);
add_block('simulink/Ports & Subsystems/In1',[q '/Decoded_Count'],'Position',[25 43 55 57]);
add_block('simulink/Discrete/Zero-Order Hold',[q '/Snapshot'], ...
    'SampleTime','p5.Tenc','Position',[100 30 160 70]);
add_block('simulink/Signal Attributes/Data Type Conversion',[q '/To_Estimator_Double'], ...
    'OutDataTypeStr','double','Position',[200 30 280 70]);
add_block('simulink/Ports & Subsystems/Out1',[q '/Count'],'Position',[340 43 370 57]);
wire(q,'Decoded_Count/1','Snapshot/1'); wire(q,'Snapshot/1','To_Estimator_Double/1');
wire(q,'To_Estimator_Double/1','Count/1');
% Physical angle is stimulus only. There is no ideal-count/angle input to
% either decoder or the active estimator.
wire(mdl,'Encoder/2','Quadrature_AB/1');
wire(mdl,'Quadrature_AB/1','Quadrature_Decoder/1');
wire(mdl,'Quadrature_AB/2','Quadrature_Decoder/2');
wire(mdl,'Quadrature_Decoder/1','Decoded_Count_Snapshot/1');
delete_line(mdl,'Encoder/1','RPM_Estimator/1');
wire(mdl,'Decoded_Count_Snapshot/1','RPM_Estimator/1');
% Preserve legacy encoder_count meaning: the sampled count actually used
% by the estimator. Retain the former ideal count on a new validation log.
port=get_param([mdl '/Logging/encoder_count'],'Port');
delete_line(mdl,'Encoder/1',['Logging/' port]);
wire(mdl,'Decoded_Count_Snapshot/1',['Logging/' port]);
add_block('simulink/Math Operations/Sum',[mdl '/Count_Difference_Validation'], ...
    'Inputs','+-','Position',[995 650 1025 690]);
wire(mdl,'Decoded_Count_Snapshot/1','Count_Difference_Validation/1');
wire(mdl,'Encoder/1','Count_Difference_Validation/2');
names={'encoder_A','encoder_B','quadrature_state','previous_quadrature_state', ...
    'decoded_encoder_count','ideal_encoder_count_phase5a','count_difference_vs_phase5a', ...
    'encoder_direction','invalid_transition_count','ideal_count_edge','encoder_edge_angle_rad'};
sources={'Quadrature_AB/1','Quadrature_AB/2','Quadrature_Decoder/2','Quadrature_Decoder/3', ...
    'Quadrature_Decoder/1','Encoder/1','Count_Difference_Validation/1', ...
    'Quadrature_Decoder/4','Quadrature_Decoder/5','Quadrature_AB/3','Quadrature_AB/4'};
q=[mdl '/Logging']; ports=find_system(q,'SearchDepth',1,'BlockType','Inport');
for k=1:numel(names)
    n=numel(ports)+k; y=25+55*n;
    add_block('simulink/Ports & Subsystems/In1',[q '/' names{k}], ...
        'Port',num2str(n),'Position',[25 y 55 y+14]);
    add_block('simulink/Sinks/To Workspace',[q '/Log_' names{k}], ...
        'VariableName',names{k},'SaveFormat','Timeseries','MaxDataPoints','inf', ...
        'Decimation','1','SampleTime','-1','Position',[155 y-5 330 y+25]);
    wire(q,[names{k} '/1'],['Log_' names{k} '/1']);
    wire(mdl,sources{k},['Logging/' num2str(n)]);
end
% The physical solver must also resolve the edge observation grid.
% Solver type and tolerances remain copied from the golden baseline.
set_param(mdl,'MaxStep',num2str(b.Tedge,17));
set_param(mdl,'SimulationCommand','update'); save_system(mdl,file);
end
function wire(q,s,d)
add_line(q,s,d,'autorouting','on');
end
