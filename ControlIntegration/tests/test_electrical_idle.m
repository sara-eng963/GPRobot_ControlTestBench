function result = test_electrical_idle(qTarget,Tend)
% TEST_ELECTRICAL_IDLE  Diagnose the electrical full system at zero command.
% Run init_fullsystem and init_servos first, with the model saved.
% No wiring, gains, gravity, solver settings or saved models are changed.
mdl = 'FullSystem_Electrical_Testbench';
load_system(mdl);
assert(strcmp(get_param(mdl,'SimulationStatus'),'stopped'), ...
    'Stop the model before running the test.');
Ts = evalin('base','Ts');
if nargin < 1 || isempty(qTarget)
    qTarget = evalin('base','initialQ');
end

qTarget = qTarget(:);
rbt = evalin('base','robot');
oldFormat = rbt.DataFormat;
cleanupFormat = onCleanup(@() setRobotFormat(rbt,oldFormat)); %#ok<NASGU>
rbt.DataFormat = 'row';
posTol = evalin('base','settlingPositionTol');
velTol = evalin('base','settlingVelocityTol');
assert(numel(qTarget)==6,'Expected six joint initial angles.');
if nargin < 2
    Tend = 0.6;
end
validateattributes(Tend,{'numeric'},{'scalar','finite','positive'});

% Use the original plant output BEFORE the controller's zero-order holds.
% These exact hold names were resolved by the existing regression log.
qHold = [mdl '/Zero-Order Hold'];
vHold = [mdl '/Zero-Order Hold1'];
pQ = inputSource(qHold,1);
pV = inputSource(vHold,1);
pCmd = outputPort([mdl '/CommandSelector'],1);
pTau = outputPort([mdl '/ServoBank'],1);
pFeed = inputSource([mdl '/ServoBank'],2);
feedBlock = get_param(pFeed,'Parent');
% A From block is a virtual alias; log the corresponding Goto producer.
if strcmp(get_param(feedBlock,'BlockType'),'From')
    tag = get_param(feedBlock,'GotoTag');
    gotos = find_system(mdl,'LookUnderMasks','all','FollowLinks','off', ...
        'BlockType','Goto','GotoTag',tag);
    assert(numel(gotos)==1,'Cannot uniquely resolve servo feedback tag %s.',tag);
    pFeed = inputSource(gotos{1},1);
end
ports = [pQ pV pCmd pTau];
names = {'idle_q_raw','idle_qd_raw','idle_cmd','idle_tau'};
idx = find(ports==pFeed,1);
if isempty(idx)
    ports(end+1) = pFeed;
    names{end+1} = 'idle_servo_feedback';
    feedbackName = names{end};
else
    feedbackName = names{idx};
end
fprintf('\n=== ELECTRICAL IDLE DIAGNOSTIC ===\n');
fprintf('Solver: %s | MaxStep: %s | Controller Ts: %.6f s\n', ...
    get_param(mdl,'Solver'),get_param(mdl,'MaxStep'),Ts);
fprintf('Servo feedback source: %s\n',feedBlock);
fprintf('Resolved producer: %s\n',get_param(pFeed,'Parent'));
fprintf('Gravity [m/s^2]: '); disp(rbt.Gravity);

% Temporarily enable logging and restore port settings on exit.
keys = {'DataLogging','DataLoggingNameMode','DataLoggingName', ...
    'DataLoggingLimitDataPoints','DataLoggingDecimateData','DataLoggingDecimation'};
old = cell(numel(ports),numel(keys)*2);
for k=1:numel(ports)
    for j=1:numel(keys)
        old{k,2*j-1}=keys{j}; old{k,2*j}=get_param(ports(k),keys{j});
    end
end
cleanup = onCleanup(@() restoreLogging(ports,old)); %#ok<NASGU>
for k=1:numel(ports)
    set_param(ports(k),'DataLogging','on','DataLoggingNameMode','Custom', ...
        'DataLoggingName',names{k},'DataLoggingLimitDataPoints','off', ...
        'DataLoggingDecimateData','off');
    fprintf('Logging %s from %s\n',names{k},get_param(ports(k),'Parent'));
end

in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime',num2str(Tend),'CaptureErrors','on', ...
    'EnablePacing','off','SignalLogging','on','SignalLoggingName','logsout', ...
    'ReturnWorkspaceOutputs','on','SaveTime','on','TimeSaveName','tout');
in = in.setBlockParameter([mdl '/RobotMode'],'Value','0');
for name = {'RecordRequest','PlanRequest','ClearRequest'}
    in = in.setBlockParameter([mdl '/TrajectoryInterface/' name{1}],'Value','0');
end
in = in.setBlockParameter([mdl '/WrenchSource'],'sw','0');
in = in.setBlockParameter([mdl '/SensorMode'],'sw','0');
% Zero external teaching stimulus; do not modify base-workspace inputs.
t = (0:Ts:ceil((Tend+Ts)/Ts)*Ts).';
w = timeseries(zeros(numel(t),6),t);
in = in.setVariable('ft_wrench_ts',w);
in = in.setVariable('zero_wrench_ts',w);
in = in.setVariable('initialQ',qTarget.');
in = in.setVariable('initialQ_deg',rad2deg(qTarget.'));

out = sim(in);
result = struct('SimulationOutput',out,'Completed',false);
stopEvent = string(out.SimulationMetadata.ExecutionInfo.StopEvent);
err = string(out.ErrorMessage);
result.Completed = stopEvent=="ReachedStopTime" && all(strlength(err)==0);
fprintf('\nStop event: %s\n',char(stopEvent));
if any(strlength(err)>0), fprintf(2,'%s\n',char(join(err,newline))); end
if ~result.Completed
    fprintf(2,'IDLE execution did not complete; inspecting partial logs if available.\n');
end
try
    logs = out.logsout;
    [tq,Q] = matrixLog(logs,'idle_q_raw');
    [tv,V] = matrixLog(logs,'idle_qd_raw');
    [tc,C] = matrixLog(logs,'idle_cmd');
    [tt,Tau] = matrixLog(logs,'idle_tau');
    [tf,F] = matrixLog(logs,feedbackName);
catch ME
    fprintf(2,'Cannot inspect motion logs: %s\n',ME.message);
    return;
end
if ~all(isfinite([Q(:);V(:);C(:);Tau(:);F(:)]))
    fprintf(2,'FAIL: Nonfinite motion/torque samples.\n');
    return;
end
last = min([tq(end),tv(end),tc(end),tt(end),tf(end)]);
first = max([tq(1),tv(1),tc(1),tt(1),tf(1)]);
fprintf('Common logged interval: %.6f to %.6f s\n',first,last);
if last<=first
    fprintf(2,'Not enough common samples for diagnostics.\n'); return;
end

% Catalogue values are diagnostics, not verified drive limits.
A=evalin('base','M8010'); B=evalin('base','M8025'); D=evalin('base','M6030G9');
rated = [A.rated_output_torque;B.rated_output_torque;A.rated_output_torque; ...
    repmat(D.rated_output_torque,3,1)];
peak = [A.peak_start_stop_output_torque;B.peak_start_stop_output_torque; ...
    A.peak_start_stop_output_torque;repmat(D.peak_start_stop_output_torque,3,1)];
G0 = gravityTorque(rbt,qTarget.').';
G0 = G0(:);

% Peak metrics from native logs; final holding from actual raw feedback.
qEnd = sample(tq,Q,last,'linear');
qFirst = sample(tq,Q,first,'linear');
vEnd = sample(tv,V,last,'linear');
maxDrift = max(abs(Q-qTarget),[],2);
maxV = max(abs(V),[],2);
maxT = max(abs(Tau),[],2);
maxC = max(abs(C),[],2);
late = linspace(max(first,last-0.1),last,201).';
VL = sample(tv,V,late,'linear');
TL = sample(tt,Tau,late,'linear');
QL = sample(tq,Q,late,'linear');
GL = zeros(size(QL));
for k=1:numel(late), GL(:,k)=reshape(gravityTorque(rbt,QL(:,k).'),6,1); end
lateMaxV = max(abs(VL),[],2);
lateTau = mean(TL,2);
lateG = mean(GL,2);
lateDiff = lateTau-lateG;
% Difference from zero-order-held feedback, not an error if intentional.
validV = tv>=first & tv<=last;
Fv = sample(tf,F,tv(validV),'previous');
maxFeedDiff = max(abs(V(:,validV)-Fv),[],2);

result.Motion = table((1:6)',rad2deg(qTarget),rad2deg(qFirst-qTarget), ...
    rad2deg(qEnd-qTarget),rad2deg(maxDrift),maxC,maxV,lateMaxV,vEnd,maxFeedDiff, ...
    'VariableNames',{'Joint','Target_deg','InitialError_deg','FinalDrift_deg', ...
    'MaxDrift_deg','MaxCommand_rad_s','PeakSpeed_rad_s','LatePeakSpeed_rad_s', ...
    'FinalSpeed_rad_s','MaxFeedbackDiff_rad_s'});
result.Torque = table((1:6)',G0,lateG,lateTau,lateDiff,maxT,rated,peak, ...
    maxT>peak, 'VariableNames',{'Joint','InitialGravity_Nm','LateGravity_Nm', ...
    'LateMotorMean_Nm','LateMotorMinusGravity_Nm','PeakAbsMotor_Nm', ...
    'Rated_Nm','PeakRating_Nm','PeakExceeded'});
fprintf('\n=== JOINT MOTION ===\n'); disp(result.Motion);
fprintf('\n=== MOTOR TORQUE VS REQUIRED GRAVITY TORQUE ===\n'); disp(result.Torque);

% Compare the signed quantities at several times, not only absolute peaks.
checkT = unique([first; 0.01; 0.05; 0.10; 0.20; 0.40; last]);
checkT = checkT(checkT>=first & checkT<=last);
Qp = sample(tq,Q,checkT,'linear');
Vp = sample(tv,V,checkT,'linear');
Tp = sample(tt,Tau,checkT,'linear');
Gp = zeros(size(Qp));
for k=1:numel(checkT), Gp(:,k)=reshape(gravityTorque(rbt,Qp(:,k).'),6,1); end
jointNames = {'J1','J2','J3','J4','J5','J6'};
result.DriftSnapshots = array2table([checkT rad2deg(Qp-qTarget).'], ...
    'VariableNames',[{'Time_s'} jointNames]);
result.VelocitySnapshots = array2table([checkT Vp.'], ...
    'VariableNames',[{'Time_s'} jointNames]);
result.TorqueSnapshots = array2table([checkT Tp.'], ...
    'VariableNames',[{'Time_s'} jointNames]);
result.GravitySnapshots = array2table([checkT Gp.'], ...
    'VariableNames',[{'Time_s'} jointNames]);
fprintf('\n=== SIGNED JOINT DRIFT [deg] ===\n'); disp(result.DriftSnapshots);
fprintf('\n=== SIGNED JOINT VELOCITY [rad/s] ===\n'); disp(result.VelocitySnapshots);
fprintf('\n=== SIGNED MOTOR OUTPUT TORQUE [Nm] ===\n'); disp(result.TorqueSnapshots);
fprintf('\n=== REQUIRED GRAVITY COMPENSATION [Nm] ===\n'); disp(result.GravitySnapshots);

result.ZeroCommand = all(maxC<=1e-10);
result.PositionHold = all(maxDrift<=posTol);
result.LateVelocityHold = all(lateMaxV<=velTol);
fprintf('\n=== IDLE SUMMARY ===\n');
fprintf('Simulation reached stop time: %s\n',passFail(result.Completed));
fprintf('Zero commanded velocity: %s\n',passFail(result.ZeroCommand));
fprintf('Position holding (max drift <= %.6g rad): %s\n',posTol,passFail(result.PositionHold));
fprintf('Late speed (last 0.1 s <= %.6g rad/s): %s\n',velTol,passFail(result.LateVelocityHold));
fprintf('Peak torque within catalogue values: %s\n',passFail(all(maxT<=peak)));
fprintf('Gravity comparison is quasi-static, not a complete dynamic torque balance.\n');
fprintf('No changes saved to the model.\n');
end

function p=outputPort(block,i)
h=get_param(block,'PortHandles'); p=h.Outport(i);
end
function p=inputSource(block,i)
h=get_param(block,'PortHandles'); l=get_param(h.Inport(i),'Line');
assert(l~=-1,'Unconnected input: %s/%d.',block,i);
p=get_param(l,'SrcPortHandle');
while p==-1
    parent=get_param(l,'LineParent');
    assert(parent~=-1 && parent~=l,'Cannot trace signal source: %s.',block);
    l=parent; p=get_param(l,'SrcPortHandle');
end
end
function [t,X]=matrixLog(logs,name)
e=logs.getElement(name); assert(~isempty(e),'Missing signal %s.',name);
ts=e.Values; assert(isa(ts,'timeseries'),'Expected timeseries: %s.',name);
t=ts.Time(:); d=double(ts.Data); n=numel(t);
assert(n>0 && numel(d)==6*n,'Unexpected signal dimensions: %s.',name);
if ts.IsTimeFirst, X=reshape(d,n,6).'; else, X=reshape(d,6,n); end
end
function Xq=sample(t,X,tq,method)
[t,ia]=unique(t(:),'last'); X=X(:,ia);
if numel(t)==1, Xq=repmat(X,1,numel(tq)); return; end
% Clamp tiny endpoint discrepancies only; callers use common log interval.
tq=min(max(tq(:),t(1)),t(end));
Xq=interp1(t,X.',tq,method).';
end
function restoreLogging(ports,old)
for k=1:numel(ports)
    try, set_param(ports(k),old{k,:});
    catch ME, warning('Logging restoration: %s',ME.message); end
end
end
function setRobotFormat(rbt,fmt)
rbt.DataFormat=fmt;
end
function s=passFail(ok)
if ok, s='PASS'; else, s='FAIL'; end
end
