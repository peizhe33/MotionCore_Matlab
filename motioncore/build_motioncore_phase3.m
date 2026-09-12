function modelFile = build_motioncore_phase3(baselineFile,mc,p3)
% Copy the validated plant into a separate model. Never call build_motioncore.
% baselineFile must point to the user's already validated MotionCore.slx.
root=fileparts(baselineFile); mdl=p3.model; modelFile=fullfile(root,[mdl '.slx']);
assert(isfile(baselineFile),'Validated MotionCore.slx is required.');
[~,base]=fileparts(baselineFile); load_system(baselineFile);
assert(strcmp(get_param(base,'Dirty'),'off'),'Save your baseline before continuing.');
assert(strcmpi(get_param(base,'FileName'),baselineFile),'Another baseline of this name is loaded.');
assert(~strcmp(base,mdl),'Phase 3 must use a different model name.');
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'Dirty'),'off'),'Save/close unsaved Phase 3 edits first.');
    assert(strcmpi(get_param(mdl,'FileName'),modelFile),'Different Phase 3 model is loaded.');
    close_system(mdl,0);
end
if exist(modelFile,'file')
    folder=fullfile(root,'phase3_backups'); if ~exist(folder,'dir'),mkdir(folder);end
    copyfile(modelFile,[tempname(folder) '.slx']);
end
load_system('simulink'); new_system(mdl);
mw=get_param(mdl,'ModelWorkspace'); mw.DataSource='Model File';
assignin(mw,'mc',mc); assignin(mw,'p3',p3);
% Preserve the validated continuous solver settings, changing only stop time.
for name={'SolverType','Solver','MaxStep','RelTol','AbsTol'}
    set_param(mdl,name{1},get_param(base,name{1}));
end
set_param(mdl,'StartTime','0','StopTime',num2str(p3.stop_time_s,17), ...
    'ReturnWorkspaceOutputs','on','SaveOutput','off','SignalLogging','off');

% The only plant construction operation: copy the existing subsystem intact.
add_block([base '/DC_Motor_Plant'],[mdl '/DC_Motor_Plant'], ...
    'Position',[760 70 920 260]);
empty([mdl '/Reference'],[35 55 155 105]);
q=[mdl '/Reference'];
add_block('simulink/Sources/Step',[q '/Speed_Request'], ...
    'Time','p3.step_time_s','Before','p3.initial_reference_rpm', ...
    'After','p3.reference_rpm','SampleTime','p3.Ts','Position',[30 30 80 60]);
add_block('simulink/Sources/Step',[q '/Optional_Second_Change'], ...
    'Time','p3.second_step_s','Before','0','After','p3.second_step_delta_rpm', ...
    'SampleTime','p3.Ts','Position',[30 105 80 135]);
sumblock(q,'Reference_Sum','++',[140 40 170 100]); out(q,'RPM',1,[230 60 260 74]);
wire(q,'Speed_Request/1','Reference_Sum/1'); wire(q,'Optional_Second_Change/1','Reference_Sum/2');
wire(q,'Reference_Sum/1','RPM/1');

empty([mdl '/Feedback'],[770 345 915 395]); q=[mdl '/Feedback'];
in(q,'Shaft_RPM',1,[25 43 55 57]); out(q,'Sampled_RPM',1,[240 43 270 57]);
add_block('simulink/Discrete/Zero-Order Hold',[q '/Sample_Speed'], ...
    'SampleTime','p3.Ts','Position',[115 30 170 70]);
wire(q,'Shaft_RPM/1','Sample_Speed/1'); wire(q,'Sample_Speed/1','Sampled_RPM/1');

empty([mdl '/Voltage_Command'],[525 70 660 120]); q=[mdl '/Voltage_Command'];
in(q,'Requested_V',1,[25 43 55 57]); out(q,'Applied_V',1,[240 43 270 57]);
add_block('simulink/Discontinuities/Saturation',[q '/Supply_Limits'], ...
    'UpperLimit','p3.voltage_max_V','LowerLimit','p3.voltage_min_V', ...
    'Position',[120 30 170 70]);
wire(q,'Requested_V/1','Supply_Limits/1'); wire(q,'Supply_Limits/1','Applied_V/1');

pid=[mdl '/Digital_PID']; empty(pid,[265 50 425 235]);
in(pid,'Reference_RPM',1,[25 33 55 47]); in(pid,'Measured_RPM',2,[25 113 55 127]);
in(pid,'Applied_Voltage',3,[25 313 55 327]);
sumblock(pid,'Speed_Error','+-',[110 40 140 100]);
gain(pid,'Proportional','p3.Kp',[190 35 250 65]);
sumblock(pid,'PID_Sum','++-',[510 40 540 130]);
sumblock(pid,'AntiWindup_Error','+-',[600 290 630 350]);
wire(pid,'Reference_RPM/1','Speed_Error/1'); wire(pid,'Measured_RPM/1','Speed_Error/2');
wire(pid,'Speed_Error/1','Proportional/1'); wire(pid,'Proportional/1','PID_Sum/1');
wire(pid,'Applied_Voltage/1','AntiWindup_Error/1'); wire(pid,'PID_Sum/1','AntiWindup_Error/2');

q=[pid '/Integral_With_AntiWindup']; empty(q,[290 95 455 170]);
in(q,'Error_RPM',1,[25 33 55 47]); in(q,'Tracking_Error_V',2,[25 133 55 147]);
gain(q,'Ki','p3.Ki',[100 25 155 55]); gain(q,'Kb','p3.Kb',[100 125 155 155]);
sumblock(q,'Integral_Rate','++',[205 45 235 105]);
gain(q,'Sample_Period','p3.Ts',[275 55 330 85]);
sumblock(q,'Next_Integral','++',[380 45 410 105]);
delay(q,'Integral_State','p3.integrator_initial_V',[465 55 520 85]);
out(q,'Integral_V',1,[585 63 615 77]);
wire(q,'Error_RPM/1','Ki/1'); wire(q,'Tracking_Error_V/1','Kb/1');
wire(q,'Ki/1','Integral_Rate/1'); wire(q,'Kb/1','Integral_Rate/2');
wire(q,'Integral_Rate/1','Sample_Period/1'); wire(q,'Sample_Period/1','Next_Integral/1');
wire(q,'Integral_State/1','Next_Integral/2'); wire(q,'Next_Integral/1','Integral_State/1');
wire(q,'Integral_State/1','Integral_V/1');
wire(pid,'Speed_Error/1','Integral_With_AntiWindup/1');
wire(pid,'AntiWindup_Error/1','Integral_With_AntiWindup/2');
wire(pid,'Integral_With_AntiWindup/1','PID_Sum/2');

q=[pid '/Filtered_Measurement_Derivative']; empty(q,[190 220 375 275]);
in(q,'RPM',1,[25 33 55 47]);
delay(q,'Previous_RPM','mc.initial.omega_rad_s*mc.units.rad_s_to_rpm',[100 125 155 155]);
sumblock(q,'RPM_Change','+-',[205 35 235 85]);
gain(q,'Difference_Coefficient','p3.derivative_b',[275 35 340 65]);
sumblock(q,'Filtered_Derivative','++',[400 40 430 100]);
delay(q,'Previous_Derivative','0',[400 160 455 190]);
gain(q,'Filter_Memory','p3.derivative_a',[275 145 340 175]);
out(q,'RPM_per_second',1,[510 53 540 67]);
wire(q,'RPM/1','RPM_Change/1'); wire(q,'RPM/1','Previous_RPM/1');
wire(q,'Previous_RPM/1','RPM_Change/2'); wire(q,'RPM_Change/1','Difference_Coefficient/1');
wire(q,'Difference_Coefficient/1','Filtered_Derivative/1');
wire(q,'Filtered_Derivative/1','Previous_Derivative/1');
wire(q,'Previous_Derivative/1','Filter_Memory/1');
wire(q,'Filter_Memory/1','Filtered_Derivative/2');
wire(q,'Filtered_Derivative/1','RPM_per_second/1');
gain(pid,'Derivative_Gain','p3.Kd',[420 220 475 250]);
wire(pid,'Measured_RPM/1','Filtered_Measurement_Derivative/1');
wire(pid,'Filtered_Measurement_Derivative/1','Derivative_Gain/1');
wire(pid,'Derivative_Gain/1','PID_Sum/3');
outs={'Raw_V','Error_RPM','Integral_V','Derivative_RPM_s','AntiWindup_V'};
srcs={'PID_Sum/1','Speed_Error/1','Integral_With_AntiWindup/1', ...
    'Filtered_Measurement_Derivative/1','AntiWindup_Error/1'};
for k=1:5
    out(pid,outs{k},k,[755 30+65*k 785 44+65*k]); wire(pid,srcs{k},[outs{k} '/1']);
end
add_block('simulink/Sources/Constant',[mdl '/Load_Torque'], ...
    'Value','p3.load_Nm','Position',[555 240 610 275]);
wire(mdl,'Reference/1','Digital_PID/1'); wire(mdl,'Feedback/1','Digital_PID/2');
wire(mdl,'Digital_PID/1','Voltage_Command/1'); wire(mdl,'Voltage_Command/1','Digital_PID/3');
wire(mdl,'Voltage_Command/1','DC_Motor_Plant/1');
wire(mdl,'Load_Torque/1','DC_Motor_Plant/2'); wire(mdl,'DC_Motor_Plant/1','Feedback/1');

names={'rpm','current_A','torque_Nm','back_emf_V','omega_rad_s', ...
    'voltage_V','load_Nm','reference_rpm','measured_rpm','error_rpm', ...
    'raw_voltage_V','integrator_V','derivative_rpm_s','aw_error_V'};
srcs={'DC_Motor_Plant/1','DC_Motor_Plant/2','DC_Motor_Plant/3', ...
    'DC_Motor_Plant/4','DC_Motor_Plant/5','Voltage_Command/1','Load_Torque/1', ...
    'Reference/1','Feedback/1','Digital_PID/2','Digital_PID/1', ...
    'Digital_PID/3','Digital_PID/4','Digital_PID/5'};
q=[mdl '/Logging']; empty(q,[1110 40 1270 455]);
for k=1:numel(names)
    y=25+55*k; in(q,names{k},k,[25 y 55 y+14]);
    add_block('simulink/Sinks/To Workspace',[q '/Log_' names{k}], ...
        'VariableName',names{k},'SaveFormat','Timeseries','MaxDataPoints','inf', ...
        'Decimation','1','SampleTime','-1','Position',[155 y-5 300 y+25]);
    wire(q,[names{k} '/1'],['Log_' names{k} '/1']);
    wire(mdl,srcs{k},['Logging/' num2str(k)]);
end
set_param(mdl,'SimulationCommand','update');
save_system(mdl,modelFile); open_system(mdl);
fprintf('Copied golden plant and built: %s\n',modelFile);
end

function empty(path,pos)
add_block('built-in/SubSystem',path,'Position',pos);
end
function in(sys,name,n,pos)
add_block('simulink/Ports & Subsystems/In1',[sys '/' name],'Port',num2str(n),'Position',pos);
end
function out(sys,name,n,pos)
add_block('simulink/Ports & Subsystems/Out1',[sys '/' name],'Port',num2str(n),'Position',pos);
end
function gain(sys,name,value,pos)
add_block('simulink/Math Operations/Gain',[sys '/' name],'Gain',value,'Position',pos);
end
function sumblock(sys,name,inputs,pos)
add_block('simulink/Math Operations/Sum',[sys '/' name],'Inputs',inputs,'Position',pos);
end
function delay(sys,name,ic,pos)
add_block('simulink/Discrete/Unit Delay',[sys '/' name],'InitialCondition',ic, ...
    'SampleTime','p3.Ts','Position',pos);
end
function wire(sys,src,dst)
add_line(sys,src,dst,'autorouting','on');
end
