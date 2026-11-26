function J = controlJacobian_URDF(robot, q)
% Geometric Jacobian expressed in the base frame.
%
% [v; omega] = J * qd

% Obtain TCP pose, joint origins and joint axes.
[T_B_TCP, jointOrigins, jointAxes] = ...
    controlFK_URDF(robot, q);

% Extract TCP position.
p_TCP = T_B_TCP(1:3,4);

J = zeros(6,6);

for i = 1:6

    p_joint = jointOrigins(:,i);
    z_joint = jointAxes(:,i);

    % Linear velocity contribution.
    Jv = cross(z_joint, p_TCP - p_joint);

    % Angular velocity contribution.
    Jw = z_joint;

    % Linear-first convention.
    J(:,i) = [Jv; Jw];

end

end