function modelFile = build_motioncore_phase4(phase3File,mc,p3,p4)
% BUILD_MOTIONCORE_PHASE4  Build Phase 4 from the frozen Phase 3 model.
% Only the actuator path is changed: Voltage_Command -> PWM_Driver -> plant.
% The copied DC_Motor_Plant and Digital_PID subsystems are not edited.
if nargin<1 || isempty(phase3File)
    phase3File=fullfile(fileparts(mfilename('fullpath')),'MotionCore_Phase3.slx');
end
assert(isfile(phase3File),'MotionCore_Phase3.slx is required.');
assert(isstruct(mc) && isstruct(p3) && isstruct(p4),'Parameter structures are required.');
root=fileparts(phase3File); mdl=p4.model; modelFile=fullfile(root,[mdl '.slx']);
[~,src]=fileparts(phase3File);
load_system(phase3File);
assert(bdIsLoaded(src),'Could not load the frozen Phase 3 model.');
assert(strcmp(get_param(src,'Dirty'),'off'),'Save the Phase 3 model before Phase 4.');
assert(strcmpi(get_param(src,'FileName'),phase3File),'A different Phase 3 file is loaded.');
assert(~strcmp(src,mdl),'Phase 4 must have a distinct model name.');
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'Dirty'),'off'),'Save/close unsaved Phase 4 edits first.');
    assert(strcmpi(get_param(mdl,'FileName'),modelFile),'A different Phase 4 model is loaded.');
    close_system(mdl,0);
end
if isfile(modelFile)
    backupDir=fullfile(root,'phase4_backups');
    if ~isfolder(backupDir),mkdir(backupDir);end
    copyfile(modelFile,[tempname(backupDir) '.slx']);
end

load_system('simulink'); new_system(mdl);
mw=get_param(mdl,'ModelWorkspace'); mw.DataSource='Model File';
assignin(mw,'mc',mc); assignin(mw,'p3',p3); assignin(mw,'p4',p4);
for name={'SolverType','Solver','MaxStep','RelTol','AbsTol'}
    set_param(mdl,name{1},get_param(src,name{1}));
end
set_param(mdl,'StartTime','0','StopTime',num2str(p4.stop_time_s,17), ...
    'ReturnWorkspaceOutputs','on','SaveOutput','off','SignalLogging','off');

% Copy every validated Phase 3 subsystem into a new root. This leaves the
% source model and its golden plant untouched, including internal line data.
top={'Reference','Feedback','Voltage_Command','Digital_PID','DC_Motor_Plant', ...
    'Load_Torque','Logging'};
pos={[35 55 155 190],[770 345 915 395],[525 70 660 120], ...
    [265 50 425 235],[760 70 920 260],[555 240 610 275],[1110 40 1270 560]};
for k=1:numel(top)
    add_block([src '/' top{k}],[mdl '/' top{k}],'Position',pos{k});
end

% Add averaged PWM driver between the existing saturation and plant.
add_block('built-in/SubSystem',[mdl '/PWM_Driver'],'Position',[675 80 745 145]);
q=[mdl '/PWM_Driver'];
in(q,'Voltage_Command_Sat',1,[25 43 55 57]);
out(q,'Average_Voltage',1,[320 33 350 47]);
out(q,'Duty_Cycle',2,[320 93 350 107]);
gain(q,'Inv_Vdc','1/p4.Vdc_V',[95 30 150 60]);
add_block('simulink/Discontinuities/Saturation',[q '/Duty_Limit'], ...
    'UpperLimit','1','LowerLimit','0','Position',[185 30 240 60]);
gain(q,'Vdc_Reconstruction','p4.Vdc_V',[260 25 315 55]);
wire(q,'Voltage_Command_Sat/1','Inv_Vdc/1');
wire(q,'Inv_Vdc/1','Duty_Limit/1');
wire(q,'Duty_Limit/1','Vdc_Reconstruction/1');
wire(q,'Vdc_Reconstruction/1','Average_Voltage/1');
wire(q,'Duty_Limit/1','Duty_Cycle/1');

% Recreate only root-level lines from Phase 3, then insert the PWM stage.
wire(mdl,'Reference/1','Digital_PID/1');
wire(mdl,'Feedback/1','Digital_PID/2');
wire(mdl,'Digital_PID/1','Voltage_Command/1');
wire(mdl,'Voltage_Command/1','Digital_PID/3'); % anti-windup feedback
wire(mdl,'Voltage_Command/1','PWM_Driver/1');
wire(mdl,'PWM_Driver/1','DC_Motor_Plant/1');
wire(mdl,'Load_Torque/1','DC_Motor_Plant/2');
wire(mdl,'DC_Motor_Plant/1','Feedback/1');

% Recreate all validated Phase 3 root-level connections into Logging.
% The Logging subsystem itself was copied above, but its external input
% lines are root-model lines and therefore must be recreated separately.
srcLog = [src '/Logging'];

logLines = get_param(srcLog,'LineHandles');
legacyInLines = logLines.Inport;

assert(numel(legacyInLines) == 14, ...
    'Expected 14 validated Phase 3 Logging inputs.');

for pidx = 1:14

    hLine = legacyInLines(pidx);

    assert(hLine ~= -1, ...
        'Phase 3 Logging input %d is not connected.', pidx);

    srcBlockH = get_param(hLine,'SrcBlockHandle');
    srcPortH  = get_param(hLine,'SrcPortHandle');

    srcPath = getfullname(srcBlockH);

    % Convert for example:
    % MotionCore_Phase3/DC_Motor_Plant
    % into:
    % DC_Motor_Plant
    prefix = [src '/'];

    assert(startsWith(srcPath,prefix), ...
        'Unexpected Phase 3 logging source: %s',srcPath);

    relSrc = srcPath(numel(prefix)+1:end);

    srcPortNo = get_param(srcPortH,'PortNumber');

    if ischar(srcPortNo) || isstring(srcPortNo)
        srcPortNo = str2double(srcPortNo);
    end

    wire(mdl, ...
        [relSrc '/' num2str(srcPortNo)], ...
        ['Logging/' num2str(pidx)]);
end

% The Phase 3 logger has ports 1..14. Add four phase-4 channels without
% touching any existing logger wiring.
names={'voltage_command_sat_V','duty_cycle','voltage_avg_V','pwm_error_V'};
for k=1:numel(names)
    port=14+k; y=25+55*port;
    in([mdl '/Logging'],names{k},port,[25 y 55 y+14]);
    add_block('simulink/Sinks/To Workspace', ...
        [mdl '/Logging/Log_' names{k}], 'VariableName',names{k}, ...
        'SaveFormat','Timeseries','MaxDataPoints','inf','Decimation','1', ...
        'SampleTime','-1','Position',[155 y-5 300 y+25]);
    wire([mdl '/Logging'],[names{k} '/1'],['Log_' names{k} '/1']);
end
add_block('simulink/Math Operations/Sum',[mdl '/PWM_Average_Error'], ...
    'Inputs','+-','Position',[955 300 985 340]);
wire(mdl,'PWM_Driver/1','PWM_Average_Error/1');
wire(mdl,'Voltage_Command/1','PWM_Average_Error/2');
wire(mdl,'Voltage_Command/1','Logging/15');
wire(mdl,'PWM_Driver/2','Logging/16');
wire(mdl,'PWM_Driver/1','Logging/17');
wire(mdl,'PWM_Average_Error/1','Logging/18');

set_param(mdl,'SimulationCommand','update');
save_system(mdl,modelFile); close_system(mdl,0);
fprintf('Built averaged-PWM Phase 4 model: %s\n',modelFile);
end

function in(sys,name,n,pos)
add_block('simulink/Ports & Subsystems/In1',[sys '/' name], ...
    'Port',num2str(n),'Position',pos);
end
function out(sys,name,n,pos)
add_block('simulink/Ports & Subsystems/Out1',[sys '/' name], ...
    'Port',num2str(n),'Position',pos);
end
function gain(sys,name,value,pos)
add_block('simulink/Math Operations/Gain',[sys '/' name], ...
    'Gain',value,'Position',pos);
end
function wire(sys,src,dst)
add_line(sys,src,dst,'autorouting','on');
end
