function [T_B_TCP, jointOrigins, jointAxes] = ...
    controlFK_URDF(robot, q)
% Forward kinematics using CAD-derived fixed transformations.
%
% q: Joint angles [rad]
% T_B_TCP: Base-to-TCP transformation

q = q(:);

T = eye(4);

jointOrigins = zeros(3,6);
jointAxes = zeros(3,6);

for i = 1:6
    % Apply fixed transformation to the joint frame.
    T = T * robot.kinematics.fixedTransforms(:,:,i);

    % Local joint rotation axis.
    a = robot.kinematics.jointAxes(:,i);

    % Store joint origin and axis in the base frame.
    jointOrigins(:,i) = T(1:3,4);
    jointAxes(:,i) = T(1:3,1:3) * a;

    % Skew-symmetric matrix of the rotation axis.
    S = [
         0    -a(3)  a(2);
         a(3)  0    -a(1);
        -a(2)  a(1)  0
    ];

    % Rodrigues rotation formula.
    R = eye(3) + sin(q(i))*S + ...
        (1-cos(q(i)))*(S*S);

    A = eye(4);
    A(1:3,1:3) = R;

    % Accumulate joint transformation.
    T = T * A;

end

% Apply the fixed flange-to-TCP transformation.
T_B_TCP = T * robot.tool.T_F_TCP;

end