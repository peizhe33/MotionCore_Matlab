function modelFile = build_motioncore_phase5(phase4File,p5,stage)
% Two-stage build: 'copy' for identity regression; 'encoder' after it passes.
% Copy the WHOLE SLX: all root-level branches and legacy logging survive.
if nargin<3,stage='copy';end
assert(isfile(phase4File),'MotionCore:MissingPhase4','Supply the validated Phase 4 SLX.');
[ok,a]=fileattrib(phase4File); assert(ok); phase4File=a.Name;
[root,src]=fileparts(phase4File); mdl=p5.model;
assert(strcmp(src,'MotionCore_Phase4') && strcmp(mdl,'MotionCore_Phase5'));
modelFile=fullfile(root,[mdl '.slx']);
if strcmp(stage,'copy')
    load_system(phase4File);
    assert(strcmpi(get_param(src,'FileName'),phase4File) && strcmp(get_param(src,'Dirty'),'off'), ...
        'The loaded Phase 4 must be the saved validated source.');
    if bdIsLoaded(mdl)
        assert(strcmp(get_param(mdl,'Dirty'),'off'),'Save or close unsaved Phase 5 edits first.');
        assert(strcmpi(get_param(mdl,'FileName'),modelFile),'Different Phase 5 file is loaded.');
        close_system(mdl,0);
    end
    if isfile(modelFile)
        folder=fullfile(root,'phase5_backups'); if ~isfolder(folder),mkdir(folder);end
        copyfile(modelFile,[tempname(folder) '.slx']);
    end
    copyfile(phase4File,modelFile); load_system(modelFile);
    assert(strcmpi(get_param(mdl,'FileName'),modelFile));
    return
end
assert(strcmp(stage,'encoder'),'stage must be copy or encoder.');
assert(bdIsLoaded(mdl) && strcmpi(get_param(mdl,'FileName'),modelFile), ...
    'Run the copy stage and identity regression before adding encoder feedback.');
assert(getSimulinkBlockHandle([mdl '/Encoder'])==-1,'Encoder already exists; use the runner to rebuild.');
mw=get_param(mdl,'ModelWorkspace'); assignin(mw,'p5',p5);

q=[mdl '/Encoder']; add_block('built-in/SubSystem',q,'Position',[750 470 910 565]);
port(q,'In1','Omega_rad_s',1,[25 43 55 57]);
add_block('simulink/Continuous/Integrator',[q '/Shaft_Angle'], ...
    'InitialCondition','p5.theta_initial_rad','Position',[100 30 135 70]);
add_block('simulink/Discrete/Zero-Order Hold',[q '/Sample_Angle'], ...
    'SampleTime','p5.Tenc','Position',[180 30 230 70]);
add_block('simulink/Math Operations/Gain',[q '/Counts_Per_Radian'], ...
    'Gain','p5.ENCODER_CPR/(2*pi)','Position',[270 30 370 70]);
add_block('simulink/Math Operations/Rounding Function',[q '/Integer_Count'], ...
    'Operator','floor','Position',[420 30 475 70]);
port(q,'Out1','Accumulated_Count',1,[540 43 570 57]);
port(q,'Out1','Shaft_Angle_rad',2,[540 143 570 157]);
wire(q,'Omega_rad_s/1','Shaft_Angle/1'); wire(q,'Shaft_Angle/1','Sample_Angle/1');
wire(q,'Sample_Angle/1','Counts_Per_Radian/1'); wire(q,'Counts_Per_Radian/1','Integer_Count/1');
wire(q,'Integer_Count/1','Accumulated_Count/1'); wire(q,'Shaft_Angle/1','Shaft_Angle_rad/1');

q=[mdl '/RPM_Estimator']; add_block('built-in/SubSystem',q,'Position',[510 470 685 565]);
port(q,'In1','Count',1,[25 43 55 57]);
add_block('simulink/Discrete/Unit Delay',[q '/Previous_Count'], ...
    'SampleTime','p5.Tenc','InitialCondition','floor(p5.theta_initial_rad*p5.ENCODER_CPR/(2*pi))', ...
    'Position',[100 120 170 160]);
add_block('simulink/Math Operations/Sum',[q '/Delta_Count'], ...
    'Inputs','+-','Position',[230 30 260 80]);
add_block('simulink/Math Operations/Gain',[q '/RPM_Per_Count'], ...
    'Gain','60/(p5.ENCODER_CPR*p5.Tenc)','Position',[310 30 415 70]);
port(q,'Out1','RPM',1,[480 43 510 57]); port(q,'Out1','Delta_Count_Out',2,[480 143 510 157]);
wire(q,'Count/1','Previous_Count/1'); wire(q,'Count/1','Delta_Count/1');
wire(q,'Previous_Count/1','Delta_Count/2'); wire(q,'Delta_Count/1','RPM_Per_Count/1');
wire(q,'RPM_Per_Count/1','RPM/1'); wire(q,'Delta_Count/1','Delta_Count_Out/1');

% Only the feedback source changes. The existing Feedback/Sample_Speed ZOH
% continues sampling at Ts, also when an optional Tenc=M*Ts is used.
delete_line(mdl,'DC_Motor_Plant/1','Feedback/1');
wire(mdl,'DC_Motor_Plant/5','Encoder/1'); wire(mdl,'Encoder/1','RPM_Estimator/1');
wire(mdl,'RPM_Estimator/1','Feedback/1');
set_param([mdl '/Feedback/Shaft_RPM'],'Name','Encoder_RPM');
add_block('simulink/Math Operations/Sum',[mdl '/Encoder_RPM_Error'], ...
    'Inputs','+-','Position',[970 540 1000 580]);
wire(mdl,'RPM_Estimator/1','Encoder_RPM_Error/1');
wire(mdl,'DC_Motor_Plant/1','Encoder_RPM_Error/2');

q=[mdl '/Logging'];
ports=find_system(q,'SearchDepth',1,'BlockType','Inport');
numbers=cellfun(@(b)str2double(get_param(b,'Port')),ports);
assert(isequal(sort(numbers(:)).',(1:numel(ports))),'Noncontiguous legacy Logging input ports.');
names={'shaft_angle_rad','encoder_count','encoder_delta_count','encoder_rpm','encoder_rpm_error'};
sources={'Encoder/2','Encoder/1','RPM_Estimator/2','RPM_Estimator/1','Encoder_RPM_Error/1'};
for k=1:numel(names)
    n=numel(ports)+k; y=25+55*n;
    port(q,'In1',names{k},n,[25 y 55 y+14]);
    add_block('simulink/Sinks/To Workspace',[q '/Log_' names{k}], ...
        'VariableName',names{k},'SaveFormat','Timeseries','MaxDataPoints','inf', ...
        'Decimation','1','SampleTime','-1','Position',[155 y-5 315 y+25]);
    wire(q,[names{k} '/1'],['Log_' names{k} '/1']);
    wire(mdl,sources{k},['Logging/' num2str(n)]);
end
set_param(mdl,'SimulationCommand','update'); save_system(mdl,modelFile);
end
function port(q,kind,name,n,pos)
add_block(['simulink/Ports & Subsystems/' kind],[q '/' name],'Port',num2str(n),'Position',pos);
end
function wire(q,src,dst)
add_line(q,src,dst,'autorouting','on');
end
