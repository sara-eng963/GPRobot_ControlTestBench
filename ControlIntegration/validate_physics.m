%% ROBOT V2 - PHYSICS VALIDATION
% Independent verification of the RobotPlant subsystem.
% No actuator controllers or admittance algorithms.
%
% Run Section 0 first, then run the desired experiment.


%% 0. INITIALIZATION

init_robot;

% Import the URDF as an analytical reference model.
robot = importrobot(urdfPath, 'DataFormat', 'row');

% Use the same gravitational acceleration as Simscape.
robot.Gravity = gravity;

% Load the independent physics testbench.
model = 'Physics_Testbench';
load_system(model);

% Path to the joint torque Constant block.
torqueBlock = [model '/Joint Torque'];


%% P01. UNACTUATED GRAVITY RESPONSE

% Apply zero torque to all six joints.
tauApplied = zeros(6,1);

% Configure the simulation.
in = Simulink.SimulationInput(model);

in = in.setBlockParameter(torqueBlock, ...
    'Value', mat2str(tauApplied,17));

in = in.setModelParameter('StopTime', '0.5');

% Run the experiment.
outP01 = sim(in);


%% P02. STATIC GRAVITY COMPENSATION

% Calculate the torques required to hold the initial pose.
tauGravity = gravityTorque(robot, initialQ);

disp('Gravity compensation torques [Nm]:');
disp(tauGravity);

% Apply gravity compensation without any controller.
in = Simulink.SimulationInput(model);

in = in.setBlockParameter(torqueBlock, ...
    'Value', mat2str(tauGravity(:),17));

in = in.setModelParameter('StopTime', '0.5');

% Run the experiment.
outP02 = sim(in);

% Extract measured physical response

% Joint positions [rad].
q = outP02.logsout.getElement('q').Values.Data;

% Joint velocities [rad/s].
qd = outP02.logsout.getElement('qd').Values.Data;

% Joint accelerations [rad/s^2].
qdd = outP02.logsout.getElement('qdd').Values.Data;


% Calculate maximum deviation from the initial configuration.
positionError = max(abs(q - initialQ), [], 'all');

% Calculate maximum absolute velocity and acceleration.
maxVelocity = max(abs(qd), [], 'all');
maxAcceleration = max(abs(qdd), [], 'all');


% Display quantitative results.
fprintf('\n=== P02: GRAVITY COMPENSATION ===\n');

fprintf('Maximum position error: %.6e rad\n', positionError);
fprintf('Maximum velocity:      %.6e rad/s\n', maxVelocity);
fprintf('Maximum acceleration:  %.6e rad/s^2\n', maxAcceleration);

% Numerical acceptance tolerances.
positionTol = 1e-8;       % rad
velocityTol = 1e-7;       % rad/s
accelerationTol = 1e-6;   % rad/s^2

% Verify static equilibrium.
passP02 = positionError <= positionTol && ...
          maxVelocity <= velocityTol && ...
          maxAcceleration <= accelerationTol;

if passP02
    fprintf('P02: PASS\n');
else
    fprintf('P02: FAIL\n');
end
%% P03. INDIVIDUAL JOINT TORQUE RESPONSE

% Calculate the mass matrix at the initial configuration.
M = massMatrix(robot, initialQ);

% Calculate gravity compensation.
tauGravity = gravityTorque(robot, initialQ);

% Apply an additional 1 Nm to Joint 1.
tauTest = [1 0 0 0 0 0];

% Total applied joint torque.
tauApplied = tauGravity + tauTest;

% Calculate the theoretical initial acceleration.
qddExpected = M \ tauTest(:);

disp('Expected joint accelerations [rad/s^2]:');
disp(qddExpected);

% Configure the simulation.
in = Simulink.SimulationInput(model);

in = in.setBlockParameter(torqueBlock, ...
    'Value', mat2str(tauApplied(:),17));

in = in.setModelParameter('StopTime', '0.05');

% Run the experiment.
outP03 = sim(in);

% Extract measured joint accelerations.
qdd = outP03.logsout.getElement('qdd').Values.Data;

% Extract acceleration at the initial instant.
qddMeasured = qdd(1,:).';

% Compare theoretical and simulated accelerations.
accelerationError = abs(qddMeasured - qddExpected);

% Display results for all six joints.
joint = (1:6).';

disp(table(joint, qddExpected, qddMeasured, ...
    accelerationError));

fprintf('Maximum acceleration error: %.6e rad/s^2\n', ...
    max(accelerationError));

% Numerical acceptance tolerance [rad/s^2].
accelerationTol = 1e-6;

passP03 = all(isfinite(accelerationError)) && ...
          max(accelerationError) <= accelerationTol;

if passP03
    fprintf('P03: PASS\n');
else
    fprintf('P03: FAIL\n');
end

%% P04. FULL DYNAMIC VERIFICATION

% Calculate gravity compensation at the initial pose.
tauGravity = gravityTorque(robot, initialQ);

% Apply additional torques to all six joints [Nm].
tauTest = [1 0.5 -0.5 0.2 -0.1 0.1];

tauApplied = tauGravity + tauTest;

% Configure and run the simulation.
in = Simulink.SimulationInput(model);

in = in.setBlockParameter(torqueBlock, ...
    'Value', mat2str(tauApplied(:),17));

in = in.setModelParameter('StopTime', '0.05');

outP04 = sim(in);


% Extract the measured physical states.
q = outP04.logsout.getElement('q').Values.Data;
qd = outP04.logsout.getElement('qd').Values.Data;
qdd = outP04.logsout.getElement('qdd').Values.Data;

% Allocate storage for the predicted torques.
tauPredicted = zeros(size(q));

% Calculate inverse dynamics at every recorded sample.
for k = 1:size(q,1)

    tauPredicted(k,:) = inverseDynamics( ...
        robot, q(k,:), qd(k,:), qdd(k,:));

end


% Compare predicted and applied joint torques.
torqueError = abs(tauPredicted - tauApplied);

maxTorqueError = max(torqueError, [], 'all');

% Acceptance tolerance inherited from the original verification.
torqueTol = 1e-4;    % Nm

passP04 = all(isfinite(torqueError), 'all') && ...
          maxTorqueError <= torqueTol;


% Display results.
fprintf('\n=== P04: FULL DYNAMIC VERIFICATION ===\n');

fprintf('Maximum torque error: %.6e Nm\n', ...
    maxTorqueError);

if passP04
    fprintf('P04: PASS\n');
else
    fprintf('P04: FAIL\n');
end

%% P05. FORWARD KINEMATICS AND TCP VERIFICATION
% Reuses the simulated motion recorded during P04.
% Compares analytical FK against Simscape measurements.

% Extract measured joint positions.
q = outP04.logsout.getElement('q').Values.Data;

% Extract measured TCP position and rotation.
positionSignal = outP04.logsout.getElement( ...
    'tcp_position').Values;

rotationSignal = outP04.logsout.getElement( ...
    'tcp_rotation').Values;

RMeasured = rotationSignal.Data;

% Number of recorded samples.
N = size(q,1);

% Allocate error arrays.
positionError = zeros(N,1);
rotationError = zeros(N,1);

% Verify TCP pose throughout the recorded motion.
for k = 1:N

    % Calculate analytical TCP transformation.
    T = getTransform(robot, q(k,:), 'tcp');

    % Extract theoretical position and orientation.
    pExpected = T(1:3,4);
    RExpected = T(1:3,1:3);

    % Extract measured rotation at this sample.
    if rotationSignal.IsTimeFirst
        Rk = squeeze(RMeasured(k,:,:));
    else
        Rk = RMeasured(:,:,k);
    end

    % Extract the measured TCP position at sample k.
    pk = getdatasamples(positionSignal, k);
    pk = pk(:);
    
    % Calculate position error [m].
    positionError(k) = norm(pExpected - pk);
    % Calculate rotation matrix error.
    rotationError(k) = norm( ...
        RExpected - Rk, 'fro');

end

% Maximum errors across the simulation.
maxPositionError = max(positionError);
maxRotationError = max(rotationError);

% Proposed numerical acceptance tolerances.
positionTol = 1e-8;     % m
rotationTol = 1e-8;     % dimensionless

passP05 = all(isfinite(positionError)) && ...
          all(isfinite(rotationError)) && ...
          maxPositionError <= positionTol && ...
          maxRotationError <= rotationTol;

% Display results.
fprintf('\n=== P05: TCP VERIFICATION ===\n');

fprintf('Maximum TCP position error: %.6e m\n', ...
    maxPositionError);

fprintf('Maximum rotation matrix error: %.6e\n', ...
    maxRotationError);

if passP05
    fprintf('P05: PASS\n');
else
    fprintf('P05: FAIL\n');
end

%% P07. SOLVER CONVERGENCE

% Reuse the same torque excitation as P04.
tauGravity = gravityTorque(robot, initialQ);
tauTest = [1 0.5 -0.5 0.2 -0.1 0.1];
tauApplied = tauGravity + tauTest;

% Configure the experiment.
in = Simulink.SimulationInput(model);

in = in.setBlockParameter(torqueBlock, ...
    'Value', mat2str(tauApplied(:),17));

in = in.setModelParameter( ...
    'StopTime', '0.1', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4');

% Simulation A: current solver step.
inA = in.setModelParameter('FixedStep', '0.0005');
outA = sim(inA);

% Simulation B: refined solver step.
inB = in.setModelParameter('FixedStep', '0.00025');
outB = sim(inB);

% Extract joint positions.
qA = outA.logsout.getElement('q').Values.Data;
qB = outB.logsout.getElement('q').Values.Data;

% Compare final joint configurations.
finalError = abs(qA(end,:) - qB(end,:));

maxSolverError = max(finalError);

% Acceptance tolerance from Roaa's original test.
solverTol = 1e-5;    % rad

passP07 = all(isfinite(finalError)) && ...
          maxSolverError <= solverTol;

% Display results.
fprintf('\n=== P07: SOLVER CONVERGENCE ===\n');

fprintf('Maximum final joint difference: %.6e rad\n', ...
    maxSolverError);

if passP07
    fprintf('P07: PASS\n');
else
    fprintf('P07: FAIL\n');
end