%% K01. FORWARD KINEMATICS AND JACOBIAN VERIFICATION
% Requires init_admittance.m to have been executed.
%
% Compare our adapted control algorithms against
% MATLAB's Robotics System Toolbox reference.

rng(42);

N = 200;

% Generate random joint configurations.
qTests = -pi + 2*pi*rand(N,6);

% Include our initial robot configuration.
qTests = [initialQ(:).'; qTests];

N = size(qTests,1);

positionError = zeros(N,1);
rotationError = zeros(N,1);
jacobianError = zeros(N,1);

for k = 1:N

    q = qTests(k,:);

    % Our forward kinematics.
    T_ours = controlFK_URDF(controlRobot,q);

    % MATLAB reference forward kinematics.
    T_ref = getTransform(robot,q,'tcp');

    % Compare TCP position.
    positionError(k) = norm( ...
        T_ours(1:3,4) - T_ref(1:3,4));

    % Compare TCP orientation.
    rotationError(k) = norm( ...
        T_ours(1:3,1:3) - T_ref(1:3,1:3),'fro');

    % Our geometric Jacobian.
    J_ours = controlJacobian_URDF(controlRobot,q);

    % MATLAB reference Jacobian.
    J_ref = geometricJacobian(robot,q,'tcp');

    % Convert MATLAB convention to our convention.
    J_ref = J_ref([4:6 1:3],:);

    % Compare Jacobians.
    jacobianError(k) = max(abs(J_ours-J_ref),[],'all');

end

%% RESULTS

fprintf('\n=== KINEMATICS VERIFICATION ===\n');

fprintf('Configurations tested: %d\n',N);

fprintf('Maximum FK position error: %.6e m\n', ...
    max(positionError));

fprintf('Maximum FK rotation error: %.6e\n', ...
    max(rotationError));

fprintf('Maximum Jacobian element error: %.6e\n', ...
    max(jacobianError));

tol = 1e-10;

passFK = all(isfinite(positionError)) && ...
         all(isfinite(rotationError)) && ...
         max(positionError) < tol && ...
         max(rotationError) < tol;

passJacobian = all(isfinite(jacobianError)) && ...
               max(jacobianError) < tol;

fprintf('FK PASS: %d\n',passFK);
fprintf('Jacobian PASS: %d\n',passJacobian);