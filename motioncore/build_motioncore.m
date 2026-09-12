function modelFile = build_motioncore(mc)
% BUILD_MOTIONCORE Construct and compile an equation-based open-loop plant.
% Usage: build_motioncore OR motioncore_init; build_motioncore(mc)
% Requires MATLAB + Simulink only. Saves beside this function.
root = fileparts(mfilename('fullpath'));
if nargin == 0
    run(fullfile(root, 'motioncore_init.m'));
end
assert(~isempty(ver('simulink')), 'MotionCore:MissingSimulink', ...
    'Install and license Simulink to construct the model.');
mdl = mc.model;
modelFile = fullfile(root, [mdl '.slx']);
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl, 'Dirty'), 'off'), ...
        'MotionCore:UnsavedModel', ...
        'Save or close your unsaved MotionCore model before rebuilding.');
    loadedFile = get_param(mdl, 'FileName');
    assert(strcmpi(loadedFile, modelFile), 'MotionCore:NameConflict', ...
        'A different model named MotionCore is loaded. Close it first.');
    close_system(mdl, 0);
end
% Preserve existing generated model/manual saved changes before rebuilding.
if exist(modelFile, 'file')
    backupDir = fullfile(root, 'backups');
    if ~exist(backupDir, 'dir'), mkdir(backupDir); end
    copyfile(modelFile, [tempname(backupDir) '.slx']);
end
load_system('simulink');
new_system(mdl);
mw = get_param(mdl, 'ModelWorkspace');
mw.DataSource = 'Model File';
assignin(mw, 'mc', mc); % Embedded snapshot: saved SLX is self-contained.
set_param(mdl, 'SolverType', 'Variable-step', 'Solver', mc.sim.solver, ...
    'StartTime', '0', 'StopTime', num2str(mc.sim.stop_time_s,17), ...
    'MaxStep', num2str(mc.sim.max_step_s,17), ...
    'RelTol', num2str(mc.sim.rel_tol,17), ...
    'AbsTol', num2str(mc.sim.abs_tol,17), ...
    'ReturnWorkspaceOutputs', 'on', 'SaveTime', 'on', ...
    'TimeSaveName', 'tout', 'SaveOutput', 'off', 'SignalLogging', 'off');

add_block('simulink/Sources/Step', [mdl '/Armature_Voltage_Step'], ...
    'Time', 'mc.input.step_time_s', ...
    'Before', 'mc.input.voltage_before_V', ...
    'After', 'mc.input.voltage_after_V', 'SampleTime', '0', ...
    'Position', [45 65 100 105]);
add_block('simulink/Sources/Constant', [mdl '/Load_Torque'], ...
    'Value', 'mc.input.load_torque_Nm', 'Position', [45 195 100 235]);
plant = [mdl '/DC_Motor_Plant'];
empty_subsystem(plant, [245 70 430 275]);
add_block('simulink/Ports & Subsystems/In1', [plant '/Va_V'], ...
    'Port', '1', 'Position', [25 63 55 77]);
add_block('simulink/Ports & Subsystems/In1', [plant '/TL_Nm'], ...
    'Port', '2', 'Position', [25 323 55 337]);

% Electrical row: di/dt = (Va - Ra*i - Ke*omega)/La.
add_block('simulink/Math Operations/Sum', [plant '/Electrical_Balance'], ...
    'Inputs', '+--', 'Position', [150 45 180 95]);
gain(plant, 'Inv_La', '1/mc.motor.La', [225 53 285 87]);
add_block('simulink/Continuous/Integrator', [plant '/Current_A'], ...
    'InitialCondition', 'mc.initial.current_A', ...
    'Position', [330 53 365 87]);
gain(plant, 'Resistance_Drop', 'mc.motor.Ra', [225 145 285 175]);
set_param([plant '/Resistance_Drop'], 'Orientation', 'left');
gain(plant, 'Torque_Constant', 'mc.motor.Kt', [420 53 480 87]);
gain(plant, 'Back_EMF', 'mc.motor.Ke', [530 200 590 230]);
set_param([plant '/Back_EMF'], 'Orientation', 'left');

% Mechanical row: domega/dt = (Kt*i - b*omega - TL)/J.
add_block('simulink/Math Operations/Sum', [plant '/Mechanical_Balance'], ...
    'Inputs', '+--', 'Position', [530 45 560 95]);
gain(plant, 'Inv_J', '1/mc.motor.J', [610 53 670 87]);
add_block('simulink/Continuous/Integrator', [plant '/Omega_rad_s'], ...
    'InitialCondition', 'mc.initial.omega_rad_s', ...
    'Position', [715 53 750 87]);
gain(plant, 'Viscous_Friction', 'mc.motor.b', [610 145 670 175]);
set_param([plant '/Viscous_Friction'], 'Orientation', 'left');
gain(plant, 'Rad_s_to_RPM', 'mc.units.rad_s_to_rpm', [800 53 890 87]);

wire(plant, 'Va_V/1', 'Electrical_Balance/1', 'Va_V');
wire(plant, 'Electrical_Balance/1', 'Inv_La/1', 'net_voltage_V');
wire(plant, 'Inv_La/1', 'Current_A/1', 'di_dt_A_s');
wire(plant, 'Current_A/1', 'Resistance_Drop/1', 'current_A');
wire(plant, 'Resistance_Drop/1', 'Electrical_Balance/2', 'Ra_i_V');
wire(plant, 'Current_A/1', 'Torque_Constant/1', '');
wire(plant, 'Torque_Constant/1', 'Mechanical_Balance/1', 'torque_Nm');
wire(plant, 'Viscous_Friction/1', 'Mechanical_Balance/2', 'friction_Nm');
wire(plant, 'TL_Nm/1', 'Mechanical_Balance/3', 'load_Nm');
wire(plant, 'Mechanical_Balance/1', 'Inv_J/1', 'net_torque_Nm');
wire(plant, 'Inv_J/1', 'Omega_rad_s/1', 'acceleration_rad_s2');
wire(plant, 'Omega_rad_s/1', 'Viscous_Friction/1', 'omega_rad_s');
wire(plant, 'Omega_rad_s/1', 'Back_EMF/1', '');
wire(plant, 'Back_EMF/1', 'Electrical_Balance/3', 'back_emf_V');
wire(plant, 'Omega_rad_s/1', 'Rad_s_to_RPM/1', '');
ports = {'RPM','Current','Torque','BackEMF','Omega'};
sources = {'Rad_s_to_RPM/1','Current_A/1','Torque_Constant/1', ...
    'Back_EMF/1','Omega_rad_s/1'};
for k = 1:numel(ports)
    y = 60 + (k-1)*70;
    add_block('simulink/Ports & Subsystems/Out1', [plant '/' ports{k}], ...
        'Port', num2str(k), 'Position', [1010 y 1040 y+14]);
    wire(plant, sources{k}, [ports{k} '/1'], '');
end

logsys = [mdl '/Logging'];
empty_subsystem(logsys, [590 50 775 320]);
names = {'rpm','current_A','torque_Nm','back_emf_V', ...
    'omega_rad_s','voltage_V','load_Nm'};
for k = 1:numel(names)
    y = 30 + (k-1)*65;
    add_block('simulink/Ports & Subsystems/In1', [logsys '/' names{k}], ...
        'Port', num2str(k), 'Position', [30 y 60 y+14]);
    add_block('simulink/Sinks/To Workspace', [logsys '/Log_' names{k}], ...
        'VariableName', names{k}, 'SaveFormat', 'Timeseries', ...
        'MaxDataPoints', 'inf', 'Decimation', '1', 'SampleTime', '-1', ...
        'Position', [175 y-5 300 y+25]);
    wire(logsys, [names{k} '/1'], ['Log_' names{k} '/1'], names{k});
end
wire(mdl, 'Armature_Voltage_Step/1', 'DC_Motor_Plant/1', 'voltage_V');
wire(mdl, 'Armature_Voltage_Step/1', 'Logging/6', '');
wire(mdl, 'Load_Torque/1', 'DC_Motor_Plant/2', 'load_Nm');
wire(mdl, 'Load_Torque/1', 'Logging/7', '');
for k = 1:5
    wire(mdl, ['DC_Motor_Plant/' num2str(k)], ...
        ['Logging/' num2str(k)], names{k});
end
set_param(mdl, 'SimulationCommand', 'update'); % Compile before saving.
save_system(mdl, modelFile);
open_system(mdl);
fprintf('Built and compiled: %s\n', modelFile);
end

function empty_subsystem(path, pos)
% Use a built-in empty subsystem, without a library template's default ports.
add_block('built-in/SubSystem', path, 'Position', pos);
end

function gain(sys, name, value, pos)
add_block('simulink/Math Operations/Gain', [sys '/' name], ...
    'Gain', value, 'Position', pos);
end

function wire(sys, src, dst, name)
h = add_line(sys, src, dst, 'autorouting', 'on');
if ~isempty(name), set_param(h, 'Name', name); end
end
