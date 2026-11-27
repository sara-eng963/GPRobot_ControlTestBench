function result = test_electrical_admittance(axisIdx, effort)

if nargin < 1
    axisIdx = 1;
end

if nargin < 2
    effort = 5;
end
% Isolate Cartesian mapping and electrical velocity tracking.
% Run init_fullsystem first. Select one wrench axis (1-6) and effort.
% Defaults reproduce A02 (Fx = 5 N). Ideal sensor bypass.
validateattributes(axisIdx,{'numeric'},{'scalar','integer','>=',1,'<=',6});
validateattributes(effort,{'numeric'},{'scalar','real','finite'});
wrenchNames = {'Fx','Fy','Fz','Mx','My','Mz'};
wrenchUnits = {'N','N','N','Nm','Nm','Nm'};
axisName = wrenchNames{axisIdx};
effortUnit = wrenchUnits{axisIdx};
% No gain, wiring, solver, or saved model changes. TEACHING throughout.
mdl = 'FullSystem_Electrical_Testbench';
load_system(mdl);
assert(strcmp(get_param(mdl,'SimulationStatus'),'stopped'), ...
    'Stop the model before A02.');
Ts = evalin('base','Ts');
assert(abs(Ts-0.002)<1e-12,'Expected 500 Hz controller.');
rbt = evalin('base','robot');
fmt = rbt.DataFormat;
formatCleanup = onCleanup(@() restoreFormat(rbt,fmt)); %#ok<NASGU>
rbt.DataFormat = 'row';
q0 = evalin('base','initialQ'); q0 = q0(:).';
assert(numel(q0)==6,'Expected six initial joint angles.');
Tend = 5; ton = 1.5; toff = 2.7;
t = (0:Ts:Tend).';
W = zeros(numel(t),6);
W(t>=ton & t<toff,axisIdx) = effort;

% Resolve raw physical feedback before the controller's ZOHs.
% Expand/compile first to avoid stale subsystem-reference port handles.
set_param(mdl,'SimulationCommand','update');
ports = [inputSource([mdl '/Zero-Order Hold'],1), ...
         inputSource([mdl '/Zero-Order Hold1'],1), ...
         outputPort([mdl '/CommandSelector'],1), ...
         outputPort([mdl '/ServoBank'],1)];
names = {'a02_q','a02_qd','a02_cmd','a02_tau'};
assert(numel(unique(ports))==numel(ports),'Unexpected duplicate logging ports.');
keys = {'DataLogging','DataLoggingNameMode','DataLoggingName', ...
    'DataLoggingLimitDataPoints','DataLoggingDecimateData','DataLoggingDecimation'};
old = cell(numel(ports),2*numel(keys));
for k=1:numel(ports)
    for j=1:numel(keys)
        old{k,2*j-1}=keys{j}; old{k,2*j}=get_param(ports(k),keys{j});
    end
end
loggingCleanup = onCleanup(@() restoreLogging(ports,old)); %#ok<NASGU>
for k=1:numel(ports)
    set_param(ports(k),'DataLogging','on','DataLoggingNameMode','Custom', ...
        'DataLoggingName',names{k},'DataLoggingLimitDataPoints','off', ...
        'DataLoggingDecimateData','off');
    fprintf('Logging %s from %s\n',names{k},get_param(ports(k),'Parent'));
end

in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime',num2str(Tend),'EnablePacing','off', ...
    'SignalLogging','on','SignalLoggingName','logsout','ReturnWorkspaceOutputs','on');
in = in.setBlockParameter([mdl '/RobotMode'],'Value','1');
for name={'RecordRequest','PlanRequest','ClearRequest'}
    in=in.setBlockParameter([mdl '/TrajectoryInterface/' name{1}],'Value','0');
end
in=in.setBlockParameter([mdl '/WrenchSource'],'sw','0');
in=in.setBlockParameter([mdl '/SensorMode'],'sw','0');
in=in.setVariable('initialQ',q0);
in=in.setVariable('initialQ_deg',rad2deg(q0));
in=in.setVariable('ft_wrench_ts',timeseries(W,t));
in=in.setVariable('zero_wrench_ts',timeseries(zeros(size(W)),t));
fprintf('\n=== ELECTRICAL ADMITTANCE TRACKING: %s ===\n',axisName);
fprintf('Initial pose [deg]: '); disp(rad2deg(q0));
fprintf('TEACHING throughout; %s = %.4g %s from %.1f to %.1f s.\n', ...
    axisName,effort,effortUnit,ton,toff);
wall=tic; out=sim(in);
fprintf('Simulation wall time: %.2f s\n',toc(wall));
assert(strcmp(string(out.SimulationMetadata.ExecutionInfo.StopEvent),'ReachedStopTime'), ...
    'A02 did not reach its stop time.');
logs=out.logsout;
% These two existing log names were used in A01.
allNames=logs.getElementNames;
assert(all(ismember({'W_measured','V_ref'},allNames)), ...
    'Missing W_measured or V_ref logging. Restore the A01 signal logs.');
[tw,wm]=readSix(logs,'W_measured');
[tr,vr]=readSix(logs,'V_ref');
[tq,q]=readSix(logs,'a02_q');
[td,qd]=readSix(logs,'a02_qd');
[tc,c]=readSix(logs,'a02_cmd');
[tt,tau]=readSix(logs,'a02_tau');
% Uniform comparison grid: held commands and interpolated physical states.
first=max([tw(1),tr(1),tq(1),td(1),tc(1),tt(1)]);
last=min([tw(end),tr(end),tq(end),td(end),tc(end),tt(end)]);
assert(first<=0.01 && last>=Tend-Ts,'Incomplete common logging interval.');
t=t(t>=first-1e-10 & t<=last+1e-10);
Wm=sample(tw,wm,t,'previous'); Vref=sample(tr,vr,t,'previous');
Q=sample(tq,q,t,'linear'); Qd=sample(td,qd,t,'linear');
C=sample(tc,c,t,'previous'); Tau=sample(tt,tau,t,'previous');
Vmapped=zeros(6,numel(t)); Vactual=Vmapped; P=zeros(3,numel(t));
for k=1:numel(t)
    Jm=geometricJacobian(rbt,Q(:,k).','tcp');
    J=Jm([4:6 1:3],:); % [linear; angular], base frame
    Vmapped(:,k)=J*C(:,k);
    Vactual(:,k)=J*Qd(:,k);
    T=getTransform(rbt,Q(:,k).','tcp'); P(:,k)=T(1:3,4);
end
E_map=Vmapped-Vref; E_servo=Vactual-Vmapped; E_total=Vactual-Vref;
steady=t>=2.4 & t<toff;
active=t>=ton & t<toff;
pre=t>=1.2 & t<ton;
late=t>=4.5 & t<=Tend;
assert(all([any(steady),any(active),any(pre),any(late)]),'Missing evaluation window.');
releaseIdx=find(t>=toff,1);
assert(all(isfinite([Wm(:);Vref(:);Vmapped(:);Vactual(:);C(:);Qd(:);Tau(:)])), ...
    'Nonfinite logged response.');
result=struct;
result.SimulationOutput=out;
result.Time=t; result.W=Wm; result.V_ref=Vref; result.V_mapped=Vmapped;
result.V_actual=Vactual; result.q=Q; result.qd=Qd; result.qd_cmd=C;
result.tau=Tau; result.tcp_position=P;
result.ErrorMapping=E_map; result.ErrorServo=E_servo; result.ErrorTotal=E_total;
result.InitialQ_deg=rad2deg(q0);
% Keep linear and angular units separate; no mixed-unit six-axis norm.
result.Steady=table(["X";"Y";"Z";"Rx";"Ry";"Rz"], ...
    ["mm/s";"mm/s";"mm/s";"rad/s";"rad/s";"rad/s"], ...
    scaledMean(Vref,steady),scaledMean(Vmapped,steady),scaledMean(Vactual,steady), ...
    scaledRms(E_map,steady),scaledRms(E_servo,steady),scaledRms(E_total,steady), ...
    'VariableNames',{'Axis','Unit','ReferenceMean','MappedMean','ActualMean', ...
    'MappingRMS','ServoRMS','TotalRMS'});
result.Joints=table((1:6)',max(abs(C),[],2),max(abs(qd),[],2), ...
    sqrt(mean((Qd(:,active)-C(:,active)).^2,2)),max(abs(tau),[],2), ...
    'VariableNames',{'Joint','PeakCommand_rad_s','PeakActual_rad_s', ...
    'ActiveTrackingRMS_rad_s','PeakOutputTorque_Nm'});
result.MeanFx_N=mean(Wm(1,steady)); % Retained for compatibility.
result.AxisIdx=axisIdx;
result.WrenchAxis=axisName;
result.RequestedEffort=effort;
result.EffortUnit=effortUnit;
result.MeanMeasuredEffort=mean(Wm(axisIdx,steady));
result.PreForcePeakTCP_mm_s=1000*max(vecnorm(Vactual(1:3,pre)));
result.LatePeakTCP_mm_s=1000*max(vecnorm(Vactual(1:3,late)));
result.ReleaseNetDisplacement_mm=1000*(P(:,end)-P(:,releaseIdx));
result.ReleasePathLength_mm=1000*sum(vecnorm(diff(P(:,releaseIdx:end),1,2)));
fprintf('\n=== STEADY WINDOW: 2.4 TO 2.7 s ===\n');
fprintf('Measured mean %s: %.4f %s\n', ...
    axisName,result.MeanMeasuredEffort,effortUnit);
disp(result.Steady);
fprintf('\n=== JOINT TRACKING AND TORQUE ===\n'); disp(result.Joints);
fprintf('\n=== ZERO-WRENCH AND RELEASE ===\n');
fprintf('Pre-force peak TCP speed (1.2-1.5 s): %.4f mm/s\n',result.PreForcePeakTCP_mm_s);
fprintf('Late peak TCP speed (4.5-5 s): %.4f mm/s\n',result.LatePeakTCP_mm_s);
fprintf('Release net displacement [X Y Z] (mm): '); disp(result.ReleaseNetDisplacement_mm.');
fprintf('Release path length (mm): %.4f\n',result.ReleasePathLength_mm);
fprintf('No tuning changes. No performance PASS thresholds imposed in this diagnostic.\n');

figure('Name',['Electrical Admittance Tracking - ' axisName],'Color','w');
tl=tiledlayout(3,2,'TileSpacing','compact');
for j=1:3
    ax=nexttile(tl,j);
    plot(ax,t,1000*[Vref(j,:);Vmapped(j,:);Vactual(j,:)].','LineWidth',1.15);
    grid(ax,'on'); xline(ax,ton,':'); xline(ax,toff,':');
    ylabel(ax,'mm/s'); xlabel(ax,'Time (s)');
    xyz={'X','Y','Z'}; title(ax,['TCP V' xyz{j}]);
    if j==1, legend(ax,'Reference','Mapped','Actual','Location','best'); end
end
ax=nexttile(tl,4); plot(ax,t,(Qd-C).'); grid(ax,'on');
ylabel(ax,'rad/s'); xlabel(ax,'Time (s)'); title(ax,'Joint velocity tracking error');
ax=nexttile(tl,5); plot(ax,t,Tau.'); grid(ax,'on');
ylabel(ax,'Nm'); xlabel(ax,'Time (s)'); title(ax,'Joint output torque');
ax=nexttile(tl,6); plot(ax,t,Wm(axisIdx,:)); grid(ax,'on');
ylabel(ax,effortUnit); xlabel(ax,'Time (s)'); title(ax,['Measured ' axisName]);
end

function y=scaledMean(X,idx)
y=mean(X(:,idx),2).*[1000;1000;1000;1;1;1];
end
function y=scaledRms(X,idx)
y=sqrt(mean(X(:,idx).^2,2)).*[1000;1000;1000;1;1;1];
end
function p=outputPort(b,i)
h=get_param(b,'PortHandles'); p=h.Outport(i);
end
function p=inputSource(b,i)
h=get_param(b,'PortHandles'); l=get_param(h.Inport(i),'Line');
assert(l~=-1,'Unconnected input: %s/%d.',b,i);
p=get_param(l,'SrcPortHandle');
while p==-1
    parent=get_param(l,'LineParent');
    assert(parent~=-1 && parent~=l,'Cannot trace source: %s.',b);
    l=parent; p=get_param(l,'SrcPortHandle');
end
end
function [t,X]=readSix(logs,name)
e=logs.getElement(name); assert(~isempty(e),'Missing signal %s.',name);
ts=e.Values; assert(isa(ts,'timeseries'),'Expected timeseries: %s.',name);
t=ts.Time(:); d=double(ts.Data); n=numel(t);
assert(n>0 && numel(d)==6*n,'Unexpected dimensions: %s.',name);
if ts.IsTimeFirst, X=reshape(d,n,6).'; else, X=reshape(d,6,n); end
end
function Xq=sample(t,X,tq,method)
[t,ia]=unique(t(:),'last'); X=X(:,ia);
assert(numel(t)>=2,'Insufficient log samples.');
tq=min(max(tq(:),t(1)),t(end));
Xq=interp1(t,X.',tq,method).';
end
function restoreLogging(ports,old)
for k=1:numel(ports)
    try
        set_param(ports(k),old{k,:});
    catch ME
        warning('RobotTest:RestoreLogging','Logging restoration: %s',ME.message);
    end
end
end
function restoreFormat(rbt,fmt)
rbt.DataFormat=fmt;
end
