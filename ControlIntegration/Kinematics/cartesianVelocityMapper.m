function qd_ref = cartesianVelocityMapper(robot, q, V_ref, lambda)

% Explicit fixed-size column vectors.
q = reshape(q,6,1);
V_ref = reshape(V_ref,6,1);

% Calculate geometric Jacobian.
J = controlJacobian_URDF(robot,q);

% Damped least-squares velocity mapping.
qd_ref = J' * ((J*J' + lambda^2*eye(6)) \ V_ref);

end