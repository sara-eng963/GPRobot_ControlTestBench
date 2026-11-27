function [q, qd, qdd] = evaluate_trajectory_reference(t_ref)
% Adapter to Roaa's existing trajectory evaluator.

reference = evalin('base','trajectoryReference');

t_ref = max(0, min(t_ref, reference.edges(end)));

[q, qd, qdd] = ...
    pathProgressEvaluateReference(reference, t_ref);

end