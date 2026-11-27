function [tStart,tEnd,goal,isLast] = ...
    trajectory_segment_info(segment)

reference = evalin('base','trajectoryReference');

tStart = reference.edges(segment);
tEnd = reference.edges(segment+1);

[goal,~,~] = ...
    pathProgressEvaluateReference(reference,tEnd);

isLast = segment == numel(reference.segment);

end