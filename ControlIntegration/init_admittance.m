%% ADMITTANCE CONTROLLER - INITIALIZATION
addpath(fullfile(fileparts(mfilename('fullpath')), ...
    'Models'));
% Location of the existing admittance repository.
admittanceDir = 'D:\University\Graduation Project\MATLAB Work Control Subteam\Admittance Control';

% Verify repository location.
assert(isfolder(admittanceDir), ...
    'Admittance repository not found.');

% Make the referenced subsystems accessible.
addpath(admittanceDir);

% Reuse the existing admittance initialization.
run(fullfile(admittanceDir, ...
    'setup_handguiding_lab.m'));

%% Robot geometry for kinematic mapping

init_robot;

robot = importrobot(urdfPath, 'DataFormat', 'row');
robot.Gravity = gravity;

%% ROBOT V2 - KINEMATIC CONFIGURATION

init_robot;

addpath(fullfile(integrationDir, 'Kinematics'));

% Import actual CAD-derived robot geometry.
robot = importrobot(urdfPath, 'DataFormat', 'row');
robot.Gravity = gravity;

% Configuration for our control algorithms.
controlRobot = struct();
controlRobot.dof = 6;

controlRobot.kinematics.fixedTransforms = zeros(4,4,6);
controlRobot.kinematics.jointAxes = zeros(3,6);

qZero = zeros(1,6);
Tprevious = eye(4);

% Extract fixed transformations and signed joint axes.
for i = 1:controlRobot.dof

    body = robot.Bodies{i};

    Tcurrent = getTransform(robot, qZero, body.Name);

    controlRobot.kinematics.fixedTransforms(:,:,i) = ...
        Tprevious \ Tcurrent;

    axis = body.Joint.JointAxis(:);

    controlRobot.kinematics.jointAxes(:,i) = ...
        axis / norm(axis);

    Tprevious = Tcurrent;

end

% Fixed transformation from the flange to the TCP.
controlRobot.tool.T_F_TCP = Tprevious \ ...
    getTransform(robot, qZero, 'tcp');

fprintf('Robot V2 kinematic geometry initialized.\n');

%% KINEMATIC MAPPING PARAMETERS

lambda = 0.02;

%% PRELIMINARY VELOCITY SERVO

Kv_servo = [15; 20; 12; 0.8; 0.5; 0.2];

%% ZERO WRENCH TEST INPUT

zero_wrench_ts = timeseries( ...
    zeros(size(ft_wrench_ts.Data)), ...
    ft_wrench_ts.Time);