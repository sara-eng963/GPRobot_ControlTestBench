%% FULL SYSTEM — ALL-GEOMETRY REGRESSION
% Run init_fullsystem once, save the model, then run this script.
% Uses existing live interface parameters; no controller tuning or model save.
% Each geometry is one continuous simulation. Results distinguish integration,
% completion, planned geometry, sampled safety diagnostics, and tracking.
fullSystemResults = runFullSystemSuite();

function results = runFullSystemSuite()
mdl = 'FullSystem_Testbench';
load_system(mdl);
assert(strcmp(get_param(mdl,'SimulationStatus'),'stopped'), ...
    'Stop the simulation before running this test.');
Ts = evalin('base','Ts');
assert(abs(Ts-0.002)<1e-12,'Expected the existing 500 Hz controller.');
rbt = evalin('base','robot');
rbt.DataFormat = 'row';
originalRef = evalin('base','trajectoryReference');
q0 = evalin('base','qStart');
q0 = q0(:).' + deg2rad([2 0 0 0 0 0]);
velocityTol = evalin('base','settlingVelocityTol');
assert(exist('pathProgressEvaluateReference','file')==2, ...
    'Run init_fullsystem first.');

% Log additional measurements automatically; restore all settings on exit.
% The previous regression's five live signals and recorder logs are also used.
ui = [mdl '/TrajectoryInterface/'];
spec = {
    [mdl '/CommandSelector'], 1, 'reg_servo_cmd', 1;
    [mdl '/RobotMode'], 1, 'reg_mode', 1;
    '@q', 1, 'reg_q', 1;
    '@qd', 1, 'reg_qd', 1;
    '@tcp_pos_sampled', 1, 'reg_tcp', 1;
    '@stop_requested', 1, 'reg_stop', 1;
    [ui 'WaypointRecorder'], 1, 'waypoint_count', 1;
    [ui 'WaypointRecorder'], 2, 'taught_Q', 25;
    [ui 'WaypointRecorder'], 3, 'taught_P', 25;
    [ui 'WaypointRecorder'], 4, 'taught_R', 25
    };
[ports,settings,dirty] = captureLogging(mdl,spec);
cleanup = onCleanup(@() restoreSuite(mdl,ports,settings,dirty,originalRef)); %#ok<NASGU>
for i = 1:size(spec,1)
    set_param(ports(i),'DataLogging','on', ...
        'DataLoggingNameMode','Custom','DataLoggingName',spec{i,3}, ...
        'DataLoggingLimitDataPoints','off', ...
        'DataLoggingDecimation',num2str(spec{i,4}), ...
        'DataLoggingDecimateData',onOff(spec{i,4}>1));
end

names = ["Straight Line" "Circular Arc" "Full Circle" "Multi-segment"];
blank = struct('Geometry',"",'Integration',"NOT RUN", ...
    'Termination',"NOT RUN",'SegmentsPlanned',NaN,'SettlingEntries',NaN, ...
    'PlanGeometry',"NOT CHECKED",'RecordedPose',"NOT CHECKED",'JointLimits',"NOT CHECKED", ...
    'URDFSpeed',"NOT CHECKED", ...
    'SelfCollision',"NOT CHECKED",'ZeroCommand',"NOT CHECKED", ...
    'Stopped',"NOT CHECKED",'Duration_s',NaN,'PlanMax_mm',NaN, ...
    'CrossTrackRMS_mm',NaN,'CrossTrackMax_mm',NaN, ...
    'TrackingRMS_mm',NaN,'TrackingMax_mm',NaN,'FinalSpeed_rad_s',NaN, ...
    'Message',"");
rows = repmat(blank,4,1);
figure('Name','Full-system regression — planned and actual paths','Color','w');
layout = tiledlayout(2,2,'TileSpacing','compact');

for type = 1:4
    rows(type).Geometry = names(type);
    fprintf('\n========== %s ==========\n',char(names(type)));
    assignin('base','trajectoryReference',originalRef);
    ax = nexttile(layout,type);
    try
        rows(type) = runCase(mdl,type,names(type),blank,Ts,rbt,q0,velocityTol,ax);
    catch ME
        rows(type).Integration = "FAIL";
        rows(type).Message = string(ME.message);
        fprintf(2,'FAIL: %s\n',ME.message);
        title(ax,names(type)+" — ERROR");
        stopIfRunning(mdl);
    end
end
results = struct2table(rows);
fprintf('\n=== FULL SYSTEM REGRESSION SUMMARY ===\n');
disp(results(:,{'Geometry','Integration','Termination','SegmentsPlanned', ...
    'SettlingEntries','RecordedPose','PlanGeometry','JointLimits','URDFSpeed','SelfCollision', ...
    'ZeroCommand','Stopped'}));
disp(results(:,{'Geometry','Duration_s','PlanMax_mm','CrossTrackRMS_mm', ...
    'CrossTrackMax_mm','TrackingRMS_mm','TrackingMax_mm','FinalSpeed_rad_s'}));
for i=1:height(results)
    if strlength(results.Message(i))>0
        fprintf('%s: %s\n',char(results.Geometry(i)),char(results.Message(i)));
    end
end
fprintf('\nIntegration PASS is not equivalent to successful endpoint settling.\n');
fprintf('Timeouts, incomplete geometry checks and collision findings remain visible.\n');
fprintf('No controller gains or tolerances were modified.\n');
end

function row = runCase(mdl,type,name,blank,Ts,rbt,q0,velocityTol,ax)
row = blank; row.Geometry = name;
ui = [mdl '/TrajectoryInterface/'];
% End time, requested mode, record, plan, clear.
% Negative tests: reject generation with 0 points; refuse unplanned playback.
pre = [0.10 0 0 1 0; 0.20 0 0 0 0; 0.40 2 0 0 0; 0.60 0 0 0 0];
if type==1
    % Preserve the successful straight-line teaching stimulus.
    teach = [0.8 1 0 0 0; 1.0 1 1 0 0; 1.7 1 0 0 0; ...
             1.9 1 1 0 0; 2.7 1 0 0 0; 3.0 0 0 0 0];
    planStart=3.0; expectedCount=2; expectedSegments=1;
else
    % X and then Y produce three generally non-collinear taught positions.
    teach = [0.8 1 0 0 0; 1.0 1 1 0 0; 1.9 1 0 0 0; ...
             2.1 1 1 0 0; 3.2 1 0 0 0; 3.4 1 1 0 0; ...
             4.3 1 0 0 0; 4.5 0 0 0 0];
    planStart=4.5; expectedCount=3; expectedSegments=1+(type==4);
end
playStart=planStart+0.4;
playEnd=playStart+25;  % test watchdog, NOT a controller timeout modification
idleEnd=playEnd+0.4;
clearEnd=idleEnd+0.2;
endTime=clearEnd+0.6;
sequence=[pre;teach;planStart+0.2 0 0 1 0;playStart 0 0 0 0; ...
    playEnd 2 0 0 0;idleEnd 0 0 0 0;clearEnd 0 0 0 1; ...
    clearEnd+0.2 0 0 0 0;clearEnd+0.4 2 0 0 0;endTime 0 0 0 0];

in=Simulink.SimulationInput(mdl);
in=in.setModelParameter('StopTime',num2str(endTime+1), ...
    'EnablePacing','off','SignalLogging','on','ReturnWorkspaceOutputs','on');
in=in.setVariable('initialQ',q0);
in=in.setVariable('initialQ_deg',rad2deg(q0));
t=(0:Ts:ceil((endTime+1)/Ts)*Ts).'; W=zeros(numel(t),6);
if type==1
    W(t>=0.6 & t<2,1)=8;
else
    W(t>=0.6 & t<1.8,1)=8;
    W(t>=2.1 & t<3.4,2)=8;
end
in=in.setVariable('ft_wrench_ts',timeseries(W,t));
in=in.setBlockParameter([mdl '/WrenchSource'],'sw','0');
in=in.setBlockParameter([mdl '/SensorMode'],'sw','0');
blocks={[mdl '/RobotMode'],[ui 'RecordRequest'],[ui 'PlanRequest'],[ui 'ClearRequest']};
for j=1:numel(blocks)
    in=in.setBlockParameter(blocks{j},'Value','0');
end
in=in.setBlockParameter([ui 'PathType'],'Value',num2str(type));
sm=simulation(in);
cleanup=onCleanup(@() stopIfRunning(mdl)); %#ok<NASGU>
for k=1:size(sequence,1)
    for j=1:4
        sm=setBlockParameter(sm,blocks{j},'Value',num2str(sequence(k,j+1)));
    end
    step(sm,PauseTime=sequence(k,1));
    if abs(sequence(k,1)-playStart)<Ts/2
        % Capture the reference actually published by the live planner.
        reference=evalin('base','trajectoryReference');
    end
end
out=stop(sm);
logs=out.logsout;
count=getTS(logs,'waypoint_count'); plan=getTS(logs,'live_plan_ready');
ready=getTS(logs,'live_approach_ready'); clock=getTS(logs,'live_playback_time');
status=getTS(logs,'live_execution_status'); stopReq=getTS(logs,'reg_stop');
mode=getTS(logs,'reg_mode'); cmd=getTS(logs,'reg_servo_cmd');
q=getTS(logs,'reg_q'); qd=getTS(logs,'reg_qd'); tcp=getTS(logs,'reg_tcp');

% Snapshot before clear (buffer logging is decimated to reduce memory).
snapshot=planStart-0.05;
n=scalarAt(count,snapshot);
assert(n==expectedCount,'Wrong number of taught waypoints: %g.',n);
Q=reshape(sampleAt(getTS(logs,'taught_Q'),snapshot),6,100); Q=Q(:,1:n);
P=reshape(sampleAt(getTS(logs,'taught_P'),snapshot),3,100); P=P(:,1:n);
R=reshape(sampleAt(getTS(logs,'taught_R'),snapshot),3,3,100); R=R(:,:,1:n);
assert(all(isfinite([Q(:);P(:);R(:)])),'Nonfinite taught data.');
assert(all(vecnorm(diff(P,1,2))>0.0005),'Taught points are too close.');
assert(scalarAt(plan,0.15)==0,'Empty waypoint buffer accepted.');
assert(scalarAt(plan,playStart-0.05)==1,'Live planning failed.');
assert(numel(reference.segment)==expectedSegments,'Unexpected segment count.');
row.SegmentsPlanned=expectedSegments; row.Duration_s=reference.edges(end);
assert(scalarAt(count,endTime-0.05)==0,'Clear did not reset count.');
assert(scalarAt(plan,endTime-0.05)==0,'Clear did not invalidate plan.');

% Verify command selection on the actual servo input, not just a status flag.
[tc,C]=seriesMatrix(cmd,6);
assert(all(isfinite(C),'all'),'Nonfinite velocity command.');
blocked=(tc>=0.22 & tc<0.38) | (tc>=clearEnd+0.22 & tc<clearEnd+0.38);
idle=scalarSeriesAt(mode,tc)==0;
assert(any(blocked) && any(idle),'No blocked/IDLE command samples.');
assert(max(abs(C(:,blocked|idle)),[],'all')<1e-10, ...
    'Invalid playback or IDLE produced a nonzero command.');
tr=ready.Time(:); rv=double(ready.Data(:));
a=find(tr>=playStart & tr<playEnd & rv==1,1);
assert(~isempty(a),'Approach did not complete within the test window.');
approachDone=tr(a);
assert(all(abs(double(clock.Data(clock.Time>=playStart & clock.Time<approachDone)))<1e-10), ...
    'Playback clock advanced before approach completed.');
assert(max(double(clock.Data(:)))>0,'Playback clock never advanced.');
st=status.Time(:); sv=double(status.Data(:));
active=st>=approachDone & st<playEnd;
assert(any(sv(active)==2),'Trajectory never reached settling.');
terminal=find(active & (sv==3 | sv==4),1);
assert(~isempty(terminal),'No COMPLETE/TIMEOUT within test window.');
terminalTime=st(terminal); code=sv(terminal);
seq=sv(active);
row.SettlingEntries=nnz([true;diff(seq)~=0] & seq==2);  % number of settling entries (including timeout)
if code==3
    row.Termination="COMPLETE";
    assert(row.SettlingEntries==expectedSegments,'Completed before all segments settled.');
else
    row.Termination="TIMEOUT";
end
assert(scalarAt(stopReq,terminalTime+2*Ts)==1,'Terminal stop request missing.');
post=tc>=terminalTime+2*Ts & tc<playEnd-2*Ts;
assert(any(post),'No post-termination command samples.');
row.ZeroCommand=passFail(max(abs(C(:,post)),[],'all')<1e-10);
assert(row.ZeroCommand=="PASS",'Terminal status did not stop the velocity command.');
assert(scalarAt(clock,idleEnd-0.05)==0,'Clock did not reset in IDLE.');
assert(scalarAt(ready,idleEnd-0.05)==0,'Approach state did not reset in IDLE.');
assert(scalarAt(status,idleEnd-0.05)==1,'Execution state did not reset in IDLE.');
assert(scalarAt(ready,clearEnd+0.35)==0,'Invalidated plan allowed approach.');
[tv,V]=seriesMatrix(qd,6);
assert(all(isfinite(V),'all'),'Nonfinite actual velocity.');
late=tv>=max(terminalTime+2*Ts,playEnd-0.5) & tv<playEnd-2*Ts;
assert(any(late),'No late velocity samples.');
row.FinalSpeed_rad_s=max(abs(V(:,late)),[],'all');
row.Stopped=passFail(row.FinalSpeed_rad_s<=velocityTol);
row.Integration="PASS";
fprintf('PASS: recording, live planning, approach, playback, terminal stop, clear and invalid-plan rejection.\n');
fprintf('Termination: %s; planned segments: %d; settling entries: %d\n', ...
    char(row.Termination),row.SegmentsPlanned,row.SettlingEntries);

%% Independent reference and motion diagnostics (no changes to control)
[tp,actualP]=seriesMatrix(tcp,3);
[tq,actualQ]=seriesMatrix(q,6);
assert(all(isfinite([actualP(:);actualQ(:)])),'Nonfinite plant feedback.');
limits=jointLimits(rbt);
assert(size(limits,1)==6,'Unexpected URDF joint ordering/count.');
refT=unique([0:Ts:reference.edges(end),reference.edges]);
[qr,vr,ar,timing]=pathProgressEvaluateReference(reference,refT);
assert(all(isfinite([qr(:);vr(:);ar(:)])),'Nonfinite planned reference.');
plannedP=FKpositions(rbt,qr);
expectedP=geometryAt(type,P,timing);
row.PlanMax_mm=1000*max(vecnorm(plannedP-expectedP));
[qe,ve,ae]=pathProgressEvaluateReference(reference,reference.edges);
endpointOK=all(abs(ve)<1e-10,'all') && all(abs(ae)<1e-10,'all');
for j=2:numel(reference.segment)
    left=reference.segment(j-1); right=reference.segment(j);
    qLeft=pathProgressEvaluateCubic(left.nodes,left.coefficients,1);
    qRight=pathProgressEvaluateCubic(right.nodes,right.coefficients,0);
    endpointOK=endpointOK && max(abs(qLeft-qRight))<=1e-4;
end
% Recorded start joint branch and orientation at prescribed endpoints.
endpointOK=endpointOK && max(abs(qe(:,1)-Q(:,1)))<=1e-4;
if type==4, orientationTargets=R;
elseif type==2, orientationTargets=R(:,:,[1 3]);
elseif type==3, orientationTargets=R(:,:,[1 1]);
else, orientationTargets=R; end
for j=1:size(qe,2)
    T=getTransform(rbt,qe(:,j).','tcp');
    E=orientationTargets(:,:,j).'*T(1:3,1:3);
    w=[E(3,2)-E(2,3);E(1,3)-E(3,1);E(2,1)-E(1,2)];
    endpointOK=endpointOK && atan2(norm(w),trace(E)-1)<=1.1e-4;
end
row.PlanGeometry=passFail(row.PlanMax_mm<=0.011 && endpointOK);
% Check all stored joint/TCP pairs against independent CAD URDF FK.
recordedOK=true;
for j=1:n
    T=getTransform(rbt,Q(:,j).','tcp');
    E=R(:,:,j).'*T(1:3,1:3);
    w=[E(3,2)-E(2,3);E(1,3)-E(3,1);E(2,1)-E(1,2)];
    recordedOK=recordedOK && norm(T(1:3,4)-P(:,j))<=1.1e-5 ...
        && norm(R(:,:,j).'*R(:,:,j)-eye(3),'fro')<=1e-6 ...
        && abs(det(R(:,:,j))-1)<=1e-6 ...
        && atan2(norm(w),trace(E)-1)<=1.1e-4;
end
row.RecordedPose=passFail(recordedOK);
within=@(x) all(x>=limits(:,1)-1e-8 & x<=limits(:,2)+1e-8,'all');
row.JointLimits=passFail(within(qr) && within(actualQ));
fprintf('Max planned geometry error: %.6f mm (%s)\n',row.PlanMax_mm,char(row.PlanGeometry));
fprintf('Joint position limits (planned + measured): %s\n',char(row.JointLimits));
fprintf('Peak planned joint speed [rad/s]:\n'); disp(max(abs(vr),[],2).');
fprintf('Peak measured joint speed [rad/s]:\n'); disp(max(abs(V),[],2).');
fprintf('Peak selected joint command [rad/s]:\n'); disp(max(abs(C),[],2).');
row.URDFSpeed=urdfSpeedDiagnostic(rbt,vr,V,C);

% Track actual path during execution and settling only, excluding approach.
idx=find(tp>=approachDone & tp<=terminalTime);
assert(~isempty(idx),'No measured TCP samples in execution window.');
idx=idx(1:4:end); actualPath=actualP(:,idx);
err=1000*polylineDistance(actualPath,expectedP);
row.CrossTrackRMS_mm=sqrt(mean(err.^2)); row.CrossTrackMax_mm=max(err);
% t_ref is the internal frozen/segmented reference clock, not PlaybackClock.
% It was logged in the independent testbench; use it if it is still exposed.
try
    rt=getTS(logs,'t_ref');
    tRef=scalarSeriesAt(rt,tp(idx));
    tRef=max(0,min(tRef,reference.edges(end)));
    qSync=pathProgressEvaluateReference(reference,tRef);
    syncError=1000*vecnorm(FKpositions(rbt,qSync)-actualPath);
    row.TrackingRMS_mm=sqrt(mean(syncError.^2));
    row.TrackingMax_mm=max(syncError);
catch ME
    row.Message="Synchronized tracking unavailable: "+string(ME.message);
end
fprintf('Cross-path RMS / max: %.3f / %.3f mm\n',row.CrossTrackRMS_mm,row.CrossTrackMax_mm);
fprintf('Synchronized tracking RMS / max: %.3f / %.3f mm\n',row.TrackingRMS_mm,row.TrackingMax_mm);
fprintf('Zero-command stop: %s; late actual speed: %.6g rad/s (%s)\n', ...
    char(row.ZeroCommand),row.FinalSpeed_rad_s,char(row.Stopped));

% Sampled self-collision diagnostic, using available URDF geometry only.
row.SelfCollision=collisionDiagnostic(rbt,qr,refT,actualQ,tq);
plot3(ax,expectedP(1,:),expectedP(2,:),expectedP(3,:),'b--','LineWidth',1.6);
hold(ax,'on');
plot3(ax,actualPath(1,:),actualPath(2,:),actualPath(3,:),'r-','LineWidth',1.3);
plot3(ax,P(1,:),P(2,:),P(3,:),'ko','MarkerFaceColor','k');
axis(ax,'equal'); grid(ax,'on'); view(ax,3);
xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
title(ax,name+" — "+row.Termination); legend(ax,'Planned','Actual','Taught','Location','best');
end

function [ports,settings,dirty]=captureLogging(mdl,spec)
keys={'DataLogging','DataLoggingNameMode','DataLoggingName', ...
    'DataLoggingLimitDataPoints','DataLoggingDecimation','DataLoggingDecimateData'};
ports=zeros(size(spec,1),1); settings=cell(size(spec,1),numel(keys)*2);
dirty=get_param(mdl,'Dirty');
for i=1:size(spec,1)
    source=spec{i,1};
    if startsWith(source,'@')
        % Resolve the signal producer from its Goto tag, independently of
        % the plant/subsystem instance name or virtual subsystem nesting.
        tag=source(2:end);
        matches=find_system(mdl,'LookUnderMasks','all','FollowLinks','off', ...
            'BlockType','Goto','GotoTag',tag);
        assert(numel(matches)==1, ...
            'Expected exactly one Goto with tag "%s"; found %d.',tag,numel(matches));
        ph=get_param(matches{1},'PortHandles');
        line=get_param(ph.Inport(1),'Line');
        assert(line~=-1,'Goto tag "%s" has no connected input.',tag);
        ports(i)=get_param(line,'SrcPortHandle');
        % A branch may need to be traced back to the root line.
        while ports(i)==-1
            parent=get_param(line,'LineParent');
            assert(parent~=-1 && parent~=line, ...
                'Cannot resolve source of Goto tag "%s".',tag);
            line=parent;
            ports(i)=get_param(line,'SrcPortHandle');
        end
    else
        ph=get_param(source,'PortHandles');
        ports(i)=ph.Outport(spec{i,2});
    end
    fprintf('Logging %s from %s\n',spec{i,3},get_param(ports(i),'Parent'));
    for j=1:numel(keys)
        settings{i,2*j-1}=keys{j}; settings{i,2*j}=get_param(ports(i),keys{j});
    end
end
end

function restoreSuite(mdl,ports,settings,dirty,reference)
stopIfRunning(mdl);
assignin('base','trajectoryReference',reference);
for i=1:numel(ports)
    try, set_param(ports(i),settings{i,:});
    catch ME, warning('Could not restore logging: %s',ME.message); end
end
% Do not save the model or suppress its dirty flag: user can inspect any changes.
if strcmp(dirty,'on'), set_param(mdl,'Dirty','on'); end
end

function stopIfRunning(mdl)
try
    if ~strcmp(get_param(mdl,'SimulationStatus'),'stopped')
        set_param(mdl,'SimulationCommand','stop');
    end
catch ME
    warning('Simulation cleanup: %s',ME.message);
end
end

function ts=getTS(logs,name)
e=logs.getElement(name); assert(~isempty(e),'Missing logged signal: %s.',name);
ts=e.Values; assert(isa(ts,'timeseries'),'Expected timeseries for %s.',name);
end
function x=sampleAt(ts,t)
k=find(ts.Time<=t+1e-10,1,'last'); assert(~isempty(k),'No sample at %.3f s.',t);
x=getdatasamples(ts,k);
end
function x=scalarAt(ts,t)
x=double(sampleAt(ts,t)); assert(isscalar(x),'Expected scalar signal.');
end
function x=scalarSeriesAt(ts,t)
[tt,ia]=unique(ts.Time(:),'last'); d=double(ts.Data(:));
x=interp1(tt,d(ia),t(:),'previous','extrap');
end
function [t,X]=seriesMatrix(ts,width)
t=ts.Time(:); n=numel(t); d=double(ts.Data);
assert(numel(d)==width*n,'Unexpected logged signal dimensions.');
if ts.IsTimeFirst, X=reshape(d,n,width).'; else, X=reshape(d,width,n); end
end
function p=FKpositions(rbt,q)
p=zeros(3,size(q,2));
for i=1:size(q,2)
    T=getTransform(rbt,q(:,i).','tcp'); p(:,i)=T(1:3,4);
end
end
function limits=jointLimits(rbt)
limits=zeros(0,2);
for i=1:numel(rbt.Bodies)
    j=rbt.Bodies{i}.Joint;
    if ~strcmp(j.Type,'fixed'), limits(end+1,:)=j.PositionLimits; end %#ok<AGROW>
end
end
function p=geometryAt(type,P,timing)
% Independent analytic geometry from the recorded points and progress.
s=timing(2,:); p=zeros(3,numel(s));
if type==1 || type==4
    for k=1:size(P,2)-1
        a=timing(1,:)==k; p(:,a)=P(:,k)+(P(:,k+1)-P(:,k))*s(a);
    end
else
    u=P(:,2)-P(:,1); v=P(:,3)-P(:,1); normal=cross(u,v);
    assert(norm(normal)>1e-12,'Circle points are collinear.');
    normal=normal/norm(normal);
    center=P(:,1)+[u.';v.';normal.']\[dot(u,u)/2;dot(v,v)/2;0];
    radius=norm(P(:,1)-center); e1=(P(:,1)-center)/radius; e2=cross(normal,e1);
    mid=mod(atan2(dot(P(:,2)-center,e2),dot(P(:,2)-center,e1)),2*pi);
    last=mod(atan2(dot(P(:,3)-center,e2),dot(P(:,3)-center,e1)),2*pi);
    if type==3, sweep=2*pi;
    elseif mid<=last+1e-12, sweep=last;
    else, sweep=last-2*pi; end
    p=center+radius*(e1*cos(sweep*s)+e2*sin(sweep*s));
end
end
function d=polylineDistance(X,P)
% Distance to finite straight chords of the densely sampled reference.
d=inf(1,size(X,2));
for j=1:size(P,2)-1
    a=P(:,j); v=P(:,j+1)-a; l2=dot(v,v);
    if l2==0, continue; end
    s=min(1,max(0,(v.'*(X-a))/l2));
    d=min(d,vecnorm(X-(a+v*s)));
end
end
function result=urdfSpeedDiagnostic(rbt,vr,V,C)
% URDF velocity values are provisional model limits, NOT motor ratings.
result="NOT CHECKED";
try
    doc=xmlread(char(evalin('base','urdfPath')));
    nodes=doc.getElementsByTagName('joint');
    names=strings(0,1);
    for j=1:numel(rbt.Bodies)
        b=rbt.Bodies{j};
        if ~strcmp(b.Joint.Type,'fixed'), names(end+1,1)=string(b.Joint.Name); end %#ok<AGROW>
    end
    lim=nan(numel(names),1);
    for j=0:nodes.getLength-1
        node=nodes.item(j);
        k=find(names==string(char(node.getAttribute('name'))),1);
        l=node.getElementsByTagName('limit');
        if ~isempty(k) && l.getLength>0
            limitNode=l.item(0);
            lim(k)=str2double(char(limitNode.getAttribute('velocity')));
        end
    end
    if numel(lim)==6 && all(isfinite(lim) & lim>0)
        peaks=max([max(abs(vr),[],2),max(abs(V),[],2),max(abs(C),[],2)],[],2);
        if any(peaks>lim+1e-8), result="PROVISIONAL FLAG";
        else, result="WITHIN MODEL VALUES"; end
    end
catch ME
    fprintf('URDF speed comparison unavailable: %s\n',ME.message);
end
fprintf('Provisional URDF speed comparison: %s\n',char(result));
end
function result=collisionDiagnostic(rbt,qr,tr,qa,ta)
result="NOT CHECKED";
try
    moving=0; withGeometry=0;
    for j=1:numel(rbt.Bodies)
        b=rbt.Bodies{j};
        if ~strcmp(b.Joint.Type,'fixed')
            moving=moving+1; withGeometry=withGeometry+~isempty(b.Collisions);
        end
    end
    fprintf('URDF collision geometry: %d/%d moving bodies; base: %d\n', ...
        withGeometry,moving,~isempty(rbt.Base.Collisions));
    if withGeometry==0 && isempty(rbt.Base.Collisions)
        fprintf('Self-collision NOT CHECKED: no moving-body/base collision geometry.\n'); return;
    end
    % Sample nominal and actual states at approximately 50 ms intervals.
    pr=unique([1,round(interp1(tr,1:numel(tr),0:0.05:tr(end),'nearest')),numel(tr)]);
    [ta,ia]=unique(ta,'last'); qa=qa(:,ia);
    pa=unique([1,round(interp1(ta,1:numel(ta),ta(1):0.05:ta(end),'nearest')),numel(ta)]);
    hits=0; states=[qr(:,pr),qa(:,pa)];
    for j=1:size(states,2)
        hit=checkCollision(rbt,states(:,j).',SkippedSelfCollisions="parent");
        hits=hits+any(hit(:));
    end
    if hits>0, result="COLLISION FLAG";
    elseif withGeometry<moving || isempty(rbt.Base.Collisions), result="PARTIAL GEOMETRY";
    else, result="NO SAMPLED FLAGS"; end
    fprintf('Self-collision: %s (%d/%d samples flagged).\n',char(result),hits,size(states,2));
catch ME
    fprintf(2,'Self-collision NOT CHECKED: %s\n',ME.message);
end
end
function s=passFail(ok)
if ok, s="PASS"; else, s="FAIL"; end
end
function s=onOff(ok)
if ok, s='on'; else, s='off'; end
end
