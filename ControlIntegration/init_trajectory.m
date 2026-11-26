%% ROBOT V2 - TRAJECTORY INITIALIZATION

%% 1. Mechanical robot initialization

init_robot;

robot = importrobot(urdfPath, 'DataFormat', 'row');
robot.Gravity = gravity;


%% 2. Trajectory repository

trajectoryRepo = fullfile(fileparts(rootDir), ...
    'Simulation-ControlGP');

pipelineDir = fullfile(trajectoryRepo, ...
    'Control', 'Trajectory', 'PathProgressPipeline');

assert(isfolder(pipelineDir), ...
    'Trajectory repository not found.');

addpath(fullfile(pipelineDir, 'helpers'));


%% 3. Controller parameters

Ts = 0.002;                 % 500 Hz

Kp_trajectory = 2;          % Position correction [1/s]

Kv_servo = [15; 20; 12; 0.8; 0.5; 0.2];


%% 4. Select and plan trajectory

if ~exist('trajectoryType','var')
    trajectoryType = "line";
end

[trajectoryData, plannerRobot] = ...
    pathProgressRunDemo(trajectoryType);

trajectoryReference = ...
    pathProgressPrepareReference(trajectoryData);

trajectoryDuration = trajectoryReference.edges(end);


%% 5. Match robot initial configuration

[qStart, ~, ~] = pathProgressEvaluateReference( ...
    trajectoryReference, 0);

initialQ = qStart(:).';
initialQ_deg = rad2deg(initialQ);


%% 6. Initialization summary

fprintf('\n=== TRAJECTORY INITIALIZED ===\n');
fprintf('Trajectory: %s\n', trajectoryType);
fprintf('Duration: %.4f s\n', trajectoryDuration);
fprintf('Controller frequency: %.0f Hz\n', 1/Ts);

fprintf('Initial joint angles [deg]:\n');
disp(initialQ_deg);

%% 7. SAMPLE TRAJECTORY REFERENCES

tRef = (0:ceil(trajectoryDuration/Ts))' * Ts;

[qRef, qdRef, qddRef] = ...
    pathProgressEvaluateReference( ...
    trajectoryReference, ...
    min(tRef, trajectoryDuration));

q_ref_ts   = timeseries(qRef.', tRef);
qd_ref_ts  = timeseries(qdRef.', tRef);
qdd_ref_ts = timeseries(qddRef.', tRef);

%% 8. TRAJECTORY SETTLING PARAMETERS

settlingPositionTol = 0.00035;   % rad
settlingVelocityTol = 0.003;     % rad/s

settlingDwell = 0.05;            % s
settlingTimeout = 1.25;          % s

q_goal = qRef(:,end);

%% 9. WAYPOINT CONFIGURATION

waypointTimes = trajectoryReference.edges(2:end);

[waypointGoals, ~, ~] = ...
    pathProgressEvaluateReference( ...
    trajectoryReference, waypointTimes);

numWaypoints = numel(waypointTimes);