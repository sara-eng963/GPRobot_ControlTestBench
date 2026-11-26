function [ok,qStart] = plan_taught_trajectory(type,n,Q,P,R)

ok = false;
qStart = zeros(6,1);

try
    %% Existing planner dependencies
    root = fileparts(fileparts(mfilename('fullpath')));
    repo = fullfile(fileparts(root),'Simulation-ControlGP');
    tr = fullfile(repo,'Control','Trajectory');

    oldPath = path;
    cleanup = onCleanup(@() path(oldPath));

    addpath(repo,tr, ...
        fullfile(tr,'TimeScaling'), ...
        fullfile(tr,'Pipeline'), ...
        fullfile(tr,'PathProgressPipeline','helpers'), ...
        fullfile(repo,'Control','Kinematics'));

    [robot,tcpFK,solveSample] = loadTrajectoryRobot();

    %% Validate waypoint count
    switch type
        case 1
            assert(n == 2,'Line requires 2 points.');
            N = 1;
        case {2,3}
            assert(n == 3,'Circle or arc requires 3 points.');
            N = 1;
        case 4
            assert(n >= 3,'Multi-segment requires 3+ points.');
            N = n-1;
        otherwise
            error('Invalid path type.');
    end

    seed = Q(:,1);
    data = struct();

    %% Generate geometry and fit trajectory
    for k = 1:N

        axis = [0 0 1];
        sweep = 0;
        Rstart = R(:,:,k);
        Rend = R(:,:,k+1);

        switch type
            case {1,4}
                assert(norm(P(:,k+1)-P(:,k)) > 0.0005, ...
                    'Waypoints too close.');

                raw = generateLineWaypoints( ...
                    P(:,k),P(:,k+1),100);

            case 2
                [raw,arc] = generateCircleWaypoints( ...
                    P(:,1),P(:,2),P(:,3),200);

                axis = arc.axis(:).';
                sweep = arc.thetaTotal;
                Rend = R(:,:,3);

            case 3
                [raw,arc] = generateFullCircleWaypoints( ...
                    P(:,1),P(:,2),P(:,3),1,400);

                Rend = Rstart;
        end

        [geometry,sOriginal,arcLength] = ...
            arcLengthParameterize(raw,0.005);

        L = sOriginal(end);

        segment = struct( ...
            'rawGeometry',raw, ...
            'arcGeometry',geometry, ...
            'arcLength',arcLength, ...
            'length',L, ...
            'startRotation',Rstart, ...
            'endRotation',Rend, ...
            'axis',axis, ...
            'orientationSweep',sweep);

        timingLength = L;

        if type == 2 || type == 3
            segment.exactCircle = arc;
            segment.exactLength = ...
                arc.radius*abs(arc.thetaTotal);

            timingLength = segment.exactLength;
        end

        [t,s,sd,sdd,sddd,profile] = ...
            sCurveTimeScaling( ...
                0.10/timingLength, ...
                0.25/timingLength, ...
                1.00/timingLength,0.05);

        segment.timing = [t(:).';s(:).';sd(:).'; ...
                          sdd(:).';sddd(:).'];

        segment.profile = profile;

        fit = pathProgressFitSegment( ...
            segment,tcpFK,solveSample, ...
            robot.params.joints.positionLimits, ...
            seed,true);

        data.segments(k) = segment;
        data.fit(k) = fit;

        seed = fit.qNodes(:,end);
    end

    %% Publish trajectory for execution
    reference = pathProgressPrepareReference(data);

    [qStart,~,~] = ...
        pathProgressEvaluateReference(reference,0);

    assignin('base','trajectoryReference',reference);

    ok = true;

    fprintf('PLAN READY: %d segments, %.3f seconds.\n', ...
        N,reference.edges(end));

catch ME
    fprintf(2,'PLANNING FAILED: %s\n',ME.message);
end

end