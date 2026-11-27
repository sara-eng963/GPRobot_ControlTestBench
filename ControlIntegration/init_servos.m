%% SERVO MOTOR INITIALIZATION

servoIntegrationDir = fileparts(mfilename('fullpath'));
servoRootDir = fileparts(servoIntegrationDir);

%% External motor repository
motorDir = fullfile(fileparts(servoRootDir), ...
    'Simulation-ControlGP-Motors', ...
    'Final_Architecture', 'motor_models');

assert(isfolder(motorDir), ...
    'Motor model repository not found.');

addpath(motorDir);
addpath(fullfile(servoIntegrationDir, 'ServoModels'));

%% Initialize motor parameters
M8010_init;
M8025_init;
M6030G9_init;

fprintf('\n=== SERVO MOTORS INITIALIZED ===\n');

%% Robot-specific velocity controller tuning

servoN = [81; 81; 81; 9; 9; 9];

servoJr = [
    M8010.rotor_inertia
    M8025.rotor_inertia
    M8010.rotor_inertia
    M6030G9.rotor_inertia
    M6030G9.rotor_inertia
    M6030G9.rotor_inertia
];

servoM = massMatrix(robot, initialQ(:).');

servoJeq = servoJr + diag(servoM)./servoN.^2;

servoWn = [15;15;20;25;25;25];
servoZeta = 1;

servoVelocityKp = 2*servoZeta.*servoWn.*servoJeq;
servoVelocityKi = servoWn.^2.*servoJeq;

disp(table((1:6)', servoJeq, ...
    servoVelocityKp, servoVelocityKi, ...
    'VariableNames',{'Joint','Jeq','Kp','Ki'}));