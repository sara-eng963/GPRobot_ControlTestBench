%% ADMITTANCE INTEGRATION - VALIDATION
% Run initialization once, then execute individual experiments.

%% 0. INITIALIZATION

init_admittance;

model = 'Admittance_Testbench';
load_system(model);


%% A06.1 - STATIONARY ROBOT

t = (0:Ts:1)';

W = zeros(numel(t),6);

test_wrench_ts = timeseries(W,t);

in = Simulink.SimulationInput(model);

in = in.setVariable('test_wrench_ts',test_wrench_ts);

in = in.setBlockParameter( ...
    [model '/Operator Wrench'], ...
    'VariableName','test_wrench_ts');

in = in.setModelParameter('StopTime','1');

outA061 = sim(in);


%% A06.2 - SINGLE-AXIS HAND-GUIDING

t = (0:Ts:4)';

% Reused profile from analyze_dynamic_handguiding.m.
timeKnots  = [0 0.5 1 1.4 1.9 2.2 2.7 4];
forceKnots = [0 0   5 5  -3  -3   0   0];

W = zeros(numel(t),6);

W(:,1) = interp1(timeKnots,forceKnots,t,'linear');

test_wrench_ts = timeseries(W,t);

in = Simulink.SimulationInput(model);

in = in.setVariable('test_wrench_ts',test_wrench_ts);

in = in.setBlockParameter( ...
    [model '/Operator Wrench'], ...
    'VariableName','test_wrench_ts');

in = in.setModelParameter('StopTime','4');

outA062 = sim(in);

%% A06.3 - CARTESIAN TRACKING VERIFICATION
% Analyze the recorded single-axis experiment.

logs = outA062.logsout;

% Extract recorded signals.
qTS     = logs.getElement('q').Values;
qdTS    = logs.getElement('qd').Values;
qdRefTS = logs.getElement('qd_ref').Values;
VrefTS  = logs.getElement('V_ref').Values;

t = qTS.Time(:);

% Verify that controller signals share the same timebase.
signals = {qdTS, qdRefTS, VrefTS};

for i = 1:numel(signals)
    ti = signals{i}.Time(:);
    assert(numel(ti) == numel(t) && ...
        max(abs(ti-t)) < 1e-9, ...
        'Logged signals have different timebases.');
end

% Extract all signals in a consistent N-by-6 format.
N = numel(t);

q     = zeros(N,6);
qd    = zeros(N,6);
qdRef = zeros(N,6);
Vref  = zeros(N,6);

for k = 1:N

    q(k,:) = reshape(getdatasamples(qTS,k),1,6);

    qd(k,:) = reshape(getdatasamples(qdTS,k),1,6);

    qdRef(k,:) = reshape(getdatasamples(qdRefTS,k),1,6);

    Vref(k,:) = reshape(getdatasamples(VrefTS,k),1,6);

end

% Allocate Cartesian velocity arrays.
N = numel(t);

Vactual = zeros(N,6);
Vmapped = zeros(N,6);

% Calculate Cartesian velocities using our verified Jacobian.
for k = 1:N

    J = controlJacobian_URDF(controlRobot,q(k,:));

    Vactual(k,:) = (J * qd(k,:).').';
    Vmapped(k,:) = (J * qdRef(k,:).').';

end

% Calculate tracking errors.
eMapping = Vref - Vmapped;
eServo   = Vmapped - Vactual;
eTotal   = Vref - Vactual;

% Display numerical results.
fprintf('\n=== CARTESIAN TRACKING ===\n');

fprintf('Peak desired X velocity: %.4f mm/s\n', ...
    1000*max(abs(Vref(:,1))));

fprintf('Peak actual X velocity: %.4f mm/s\n', ...
    1000*max(abs(Vactual(:,1))));

fprintf('Peak X mapping error: %.4f mm/s\n', ...
    1000*max(abs(eMapping(:,1))));

fprintf('Peak X servo error: %.4f mm/s\n', ...
    1000*max(abs(eServo(:,1))));

fprintf('Peak X tracking error: %.4f mm/s\n', ...
    1000*max(abs(eTotal(:,1))));

% Visual comparison.
figure('Name','Cartesian Tracking');

plot(t,1000*Vref(:,1),'--','LineWidth',1.5);
hold on;
plot(t,1000*Vmapped(:,1),':','LineWidth',1.5);
plot(t,1000*Vactual(:,1),'LineWidth',1.5);

xlabel('Time [s]');
ylabel('X Velocity [mm/s]');
title('Desired vs Actual Cartesian Velocity');
legend('Admittance Reference','DLS Mapping','Actual Robot');
grid on;