%% WAYPOINT RECORDING - AUTOMATED TEST

mdl = 'FullSystem_Testbench';

%% 1. Configure simulation

in = Simulink.SimulationInput(mdl);

in = in.setModelParameter( ...
    'StopTime','5', ...
    'EnablePacing','off', ...
    'SignalLogging','on', ...
    'ReturnWorkspaceOutputs','on');

in = in.setBlockParameter( ...
    [mdl '/RobotMode'],'Value','0');

in = in.setBlockParameter( ...
    [mdl '/RecordRequest'],'Value','0');

%% Apply synthetic operator force

tWrench = (0:Ts:5)';

W = zeros(numel(tWrench),6);

% Apply 8 N along X during teaching
W(tWrench >= 0.6 & tWrench < 2.5,1) = 8;

testWrench = timeseries(W,tWrench);

in = in.setVariable('ft_wrench_ts',testWrench);

% Select scripted wrench
in = in.setBlockParameter( ...
    [mdl '/WrenchSource'],'sw','0');

% Select ideal sensor bypass
in = in.setBlockParameter( ...
    [mdl '/SensorMode'],'sw','0');

sm = simulation(in);

%% 2. Define test sequence

% End time | Mode | Record request | Expected count

tests = [
    0.2    0    0    0
    0.4    0    1    0
    0.6    0    0    0
    0.8    1    0    0
    1.0    1    1    1
    1.5    1    1    1
    1.7    1    0    1
    1.9    1    1    2
    2.1    1    0    2
    2.3    1    1    3
    2.5    1    0    3
    2.7    0    1    3
    2.9    0    0    3
];

%% 3. Execute automated button presses

for k = 1:size(tests,1)

    sm = setBlockParameter(sm, ...
        [mdl '/RobotMode'],'Value', ...
        num2str(tests(k,2)));

    sm = setBlockParameter(sm, ...
        [mdl '/RecordRequest'],'Value', ...
        num2str(tests(k,3)));

    step(sm,PauseTime=tests(k,1));

end

outRecording = stop(sm);

%% 4. Analyze results

countTS = outRecording.logsout ...
    .getElement('waypoint_count').Values;

actual = interp1(countTS.Time, ...
    countTS.Data(:), ...
    tests(:,1)-2*Ts,'previous');

expected = tests(:,4);

results = table( ...
    tests(:,1), tests(:,2), tests(:,3), ...
    expected, actual, actual == expected, ...
    'VariableNames', ...
    {'Time','Mode','Record','Expected','Actual','Pass'});

disp(results);

%% 5. Final verdict

fprintf('\n=== WAYPOINT RECORDING TEST ===\n');

assert(all(actual == expected), ...
    'FAIL: Waypoint recording logic incorrect.');

fprintf('PASS: Recording restricted to TEACHING.\n');
fprintf('PASS: Held button does not duplicate points.\n');
fprintf('PASS: Consecutive waypoint recording.\n');
fprintf('PASS: Waypoints retained in IDLE.\n');
fprintf('ALL TESTS PASSED.\n');

%% 6. Visualization

figure('Name','Waypoint Recording Test');

stairs(countTS.Time,countTS.Data,'LineWidth',2);

xlabel('Simulation Time [s]');
ylabel('Recorded Waypoints');
title('Automated Waypoint Recording');
yticks(0:3);
grid on;

%% EXTRACT RECORDED WAYPOINTS

logs = outRecording.logsout;

n = countTS.Data(end);

Qts = logs.getElement('taught_Q').Values;
Pts = logs.getElement('taught_P').Values;
Rts = logs.getElement('taught_R').Values;

Q = reshape(getdatasamples(Qts,numel(Qts.Time)),6,100);
P = reshape(getdatasamples(Pts,numel(Pts.Time)),3,100);
R = reshape(getdatasamples(Rts,numel(Rts.Time)),3,3,100);

taughtQ = Q(:,1:n);
taughtP = P(:,1:n);
taughtR = R(:,:,1:n);

fprintf('\n=== RECORDED WAYPOINTS ===\n');

disp('Joint positions [deg]:');
disp(rad2deg(taughtQ.'));

disp('TCP positions [m]:');
disp(taughtP.');

disp('Distances between consecutive waypoints [mm]:');
disp(1000*vecnorm(diff(taughtP,1,2),2,1));

%% Verify distinct recorded positions

distances_mm = 1000*vecnorm(diff(taughtP,1,2),2,1);

assert(all(distances_mm > 0.5), ...
    'FAIL: Recorded waypoints are not sufficiently distinct.');

fprintf('PASS: Distinct waypoint positions recorded.\n');

figure('Name','Recorded Waypoints');

plot3(taughtP(1,:),taughtP(2,:),taughtP(3,:), ...
    'bo-','LineWidth',2,'MarkerFaceColor','b');

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Taught Waypoints');
axis equal;
grid on;