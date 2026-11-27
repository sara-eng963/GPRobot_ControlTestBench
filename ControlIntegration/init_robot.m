%% ROBOT V2 - INITIALIZATION
% Initializes the mechanical robot model and simulation environment.
%
% Shared by:
%   - Physics_Testbench.slx
%   - Admittance_Testbench.slx
%   - FreeRTOS_Testbench.slx


%% 1. PROJECT PATHS

% Locate this script's directory (ControlIntegration).
integrationDir = fileparts(mfilename('fullpath'));

% Locate the repository root.
rootDir = fileparts(integrationDir);

% Locate the original mechanical assets.
mechanicsDir = fullfile(rootDir, 'mechanics', ...
    'URDF_Assembly_Robot_ver2');

% STL mesh files used by Simscape Multibody.
meshDir = fullfile(mechanicsDir, 'meshes');

% URDF used for kinematic and dynamic verification.
urdfPath = fullfile(mechanicsDir, 'urdf', ...
    'URDF_Assembly_Robot_ver2.urdf');

% Make the Simulink models and STL meshes accessible.
addpath(integrationDir);
addpath(fullfile(integrationDir,'Models'));
addpath(fullfile(integrationDir,'Trajectory'));
addpath(fullfile(integrationDir,'Testbenches'));
addpath(fullfile(integrationDir,'tests'));
addpath(fullfile(integrationDir,'ServoModels'));
addpath(fullfile(integrationDir,'Kinematics'));
addpath(meshDir);


%% 2. INITIAL CONDITIONS

% Initial angular position of joints J1 to J6.
% Inherited from the original testbench.
initialQ_deg = [0 0 -120 60 131.25 0];

% Convert degrees to radians for Simscape.
initialQ = deg2rad(initialQ_deg);



%% 3. PHYSICAL ENVIRONMENT

% Gravity vector in the world coordinate frame [m/s^2].
% Negative Z corresponds to downward gravity.
gravity = [0 0 -9.81];


%% 4. SIMULATION PARAMETERS

% Initial fixed integration step for the mechanical solver.
% Subject to numerical convergence verification.
plantTs = 0.0005;             % 0.5 ms


%% 5. DEPENDENCY CHECKS

% Verify that the required mechanical files exist.
assert(isfolder(meshDir), 'STL mesh directory not found.');

assert(isfile(urdfPath), 'Robot URDF not found.');

assert(isfile(fullfile(integrationDir, 'Models', 'RobotPlant.slx')), ...
    'RobotPlant.slx not found.');


%% 6. INITIALIZATION STATUS

fprintf('\n=== ROBOT V2 INITIALIZED ===\n');
fprintf('Initial joint angles [deg]:\n');
disp(initialQ_deg);

fprintf('Gravity [m/s^2]:\n');
disp(gravity);

fprintf('Plant integration step: %.4f s\n', plantTs);
fprintf('============================\n');