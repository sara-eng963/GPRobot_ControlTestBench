function folder=run_visible_admittance(subset,rawFolder)
% Run independent trials, recording each before evaluating it.
if nargin==0, subset="all"; end
here=fileparts(mfilename('fullpath'));
model=build_visible_admittance();
cfg=testbench_config(); d=cfg.demo; dt=cfg.sampleTime_s;
folder=fullfile(here,'results',char(datetime('now','TimeZone','Africa/Cairo','Format','yyyyMMdd_HHmmss_SSS')));
mkdir(folder); copyfile(fullfile(here,'PREREGISTRATION.txt'),folder);
settings=struct('matlab',version,'model',get_param(model,'FileName'), ...
    'mapper',which('rb2_visible_mapper'),'oracle',which('rb2_minimal_controller'), ...
    'sampleTime',dt,'solverType','ode4','solverStep',.0005);
save(fullfile(folder,'configuration.mat'),'cfg','settings');
assert(~isempty(settings.mapper) && ~isempty(settings.oracle),'visible:Prerequisite','Both supplied native gateways are required.');
if subset=="rescore"
    inputs=load(fullfile(rawFolder,'fixtures.mat'),'cases');
    name=strings(0,1); controller=name; plant=name; evidence=name;
    for a=inputs.cases
        tag=a.name;
        if ~isfile(fullfile(rawFolder,tag+".mat")), continue; end
        try
            data=load(fullfile(rawFolder,tag+".mat"),'s','cmd','sample');
            [comparison,m,c,p,msg]=assess(here,a,data.s,cfg,data.sample,data.cmd);
            save(fullfile(folder,tag+"_rescore.mat"),'comparison','m');
            row(tag,c,p,msg);
        catch err
            row(tag,"FAIL","FAIL",saveError(folder,tag,err));
        end
    end
    fprintf('RESULTS %s\n',folder);
    return
end
original=Simulink.fileGenControl('getConfig');
cleanup=onCleanup(@()Simulink.fileGenControl('setConfig','config',original));
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'), ...
    'CodeGenFolder',fullfile(folder,'codegen'),'createDir',true);
mark(model,'Cartesian Admittance',1,'twist_now'); mark(model,'Cartesian Admittance',2,'twist_published');
mark(model,'Cartesian-to-Joint Mapper',1,'mapper_qd');
mark(model,'User Applied Wrench',1,'applied_wrench'); mark(model,'SensorReference',2,'ema_wrench');
mark(model,'JointVelocityCommand',1,'provided_qd');
mark(model,'JointVelocitySelection',1,'selected_qd'); mark(model,'RobotPlant',1,'q');
mark(model,'RobotPlant',2,'qd'); mark(model,'RobotPlant',4,'tcp');
mark(model,'RobotPlant',5,'tcp_orientation');
mark(model,'VelocityServo/ProvisionalPI',1,'torque');
mark(model,'VelocityServo/ProvisionalPI',2,'torque_requested');
mark(model,'VelocityServo/ProvisionalPI',3,'torque_limited');
name=strings(0,1); controller=name; plant=name; evidence=name;
ctrlTol=1e-12;
for tag=["isolated_zero","isolated_positive_limits","isolated_negative_limits"]
    disp("Running "+tag);
    try
        raw=isolated(tag,model,cfg);
        save(fullfile(folder,tag+".mat"),'raw','-v7');
        errorC=max(abs(raw.twist-raw.oracle),[],'all');
        delta=diff([zeros(1,6);raw.twist],1,1);
        acc=max(abs(delta),[],1);
        speed=max(abs(raw.twist),[],1);
        bounds=all(acc<=dt*d.admittanceAccelerationLimit+ctrlTol) && ...
            all(speed<=d.admittanceSpeedLimit+ctrlTol);
        if tag=="isolated_zero"
            valid=max(speed)<=ctrlTol;
        else
            valid=all(abs(acc-dt*d.admittanceAccelerationLimit)<=ctrlTol) && ...
                all(abs(speed-d.admittanceSpeedLimit)<=ctrlTol);
        end
        publishedError=max(abs(raw.published-[zeros(1,6);raw.twist(1:end-1,:)]),[],'all');
        c=verdict(bounds && valid && max(errorC,publishedError)<=ctrlTol);
        message=sprintf("C max %.3g; published max %.3g; peak speeds %s; peak delta/dt %s", ...
            errorC,publishedError,mat2str(speed,3),mat2str(acc/dt,3));
    catch err
        c="FAIL"; message=saveError(folder,tag,err);
    end
    row(tag,c,"N/A",message);
end
if subset=="isolated"
    fprintf('RESULTS %s\n',folder);
    return
end
f=struct('name',"zero_wrench",'w',zeros(1,6),'start',.10,'release',.52,'hold',0,'stop',.40, ...
    'axis',0,'sgn',0,'archive',"",'standalone',false);
cases=f;
for k=1:6
    labels=["Fx","Fy","Fz","Mx","My","Mz"];
    amp=[.18 .18 .18 .015 .015 .015];
    for sign=[1 -1]
        a=f; a.stop=.92; a.axis=k; a.sgn=sign; a.w(k)=amp(k)*sign;
        if sign>0, suffix="positive"; else, suffix="negative"; end
        a.name=labels(k)+"_"+suffix;
        if k==1, a.archive="force_"+suffix; elseif k==4, a.archive="moment_"+suffix; end
        cases(end+1)=a;
    end
end
a=f; a.name="push_sustain_release"; a.stop=.92; a.release=.44; a.hold=1;
a.w=[.12 -.06 0 0 .010 0]; cases(end+1)=a;
a=f; a.name="standalone_selector"; a.standalone=true; a.stop=.40; cases(end+1)=a;
save(fullfile(folder,'fixtures.mat'),'cases');
for a=cases
    tag=a.name; disp("Running "+tag);
    try
        u=Simulink.SimulationInput(model);
        names=["Fx","Fy","Fz","Mx","My","Mz","pushStart","pushRelease","holdProfile"];
        vals=[a.w a.start a.release a.hold];
        for k=1:numel(names)
            u=u.setBlockParameter([model '/User Applied Wrench'],char(names(k)),num2str(vals(k),17));
        end
        sample=(0:dt:a.stop+dt).';
        cmd=zeros(numel(sample),6);
        if a.standalone
            window=sample>=.08 & sample<.30;
            cmd(window,1)=.02*sin(pi*(sample(window)-.08)/.22).^2;
        end
        u=u.setExternalInput([sample cmd]);
        u=u.setVariable('velocityCase',struct('standalone',a.standalone),'Workspace',model);
        u=u.setModelParameter('SignalLogging','on','SignalLoggingName','logsout', ...
            'ReturnWorkspaceOutputs','on','StopTime',num2str(a.stop),'FixedStep','.0005');
        result=sim(u);
        s=readLogs(result.logsout);
        save(fullfile(folder,tag+".mat"),'s','a','cmd','sample','-v7');
    catch err
        row(tag,"BLOCKED","FAIL",saveError(folder,tag,err)); continue
    end
    try
        [comparison,m,c,p,msg]=assess(here,a,s,cfg,sample,cmd);
        save(fullfile(folder,tag+".mat"),'comparison','m','-append');
        row(tag,c,p,msg);
    catch err
        row(tag,"FAIL","FAIL",saveError(folder,tag,err));
    end
end
fprintf('RESULTS %s\n',folder);
    function row(id,c,p,msg)
        name(end+1,1)=id; controller(end+1,1)=c; plant(end+1,1)=p; evidence(end+1,1)=msg;
        writetable(table(name,controller,plant,evidence),fullfile(folder,'summary.csv'));
        disp(c+" controller | "+p+" plant | "+msg);
    end
end

function raw=isolated(tag,model,cfg)
d=cfg.demo; dt=cfg.sampleTime_s;
h='visible_admittance_control_fixture';
new_system(h);
cleanup=onCleanup(@()close_system(h,0));
sw=get_param(h,'ModelWorkspace');
vars={'admittanceTs','admittanceMass','admittanceDamping','admittanceAccelerationLimit','admittanceSpeedLimit'};
val={dt,d.virtualMass,d.virtualDamping,d.admittanceAccelerationLimit,d.admittanceSpeedLimit};
for k=1:numel(vars), sw.assignin(vars{k},val{k}); end
add_block([model '/Cartesian Admittance'],[h '/Admittance']);
add_block('simulink/Ports & Subsystems/In1',[h '/Wrench'], ...
    'PortDimensions','6','Interpolate','off','SampleTime','admittanceTs');
add_block('simulink/Sources/Constant',[h '/False'],'Value','0','SampleTime','admittanceTs');
add_block('simulink/Sinks/Terminator',[h '/Now']);
add_block('simulink/Sinks/Terminator',[h '/Published']);
add_line(h,'Wrench/1','Admittance/1'); add_line(h,'False/1','Admittance/2');
add_line(h,'Admittance/1','Now/1'); add_line(h,'Admittance/2','Published/1');
mark(h,'Admittance',1,'twist'); mark(h,'Admittance',2,'published');
t=(0:dt:.94).'; w=zeros(numel(t),6);
if tag~="isolated_zero"
    direction=1-2*(tag=="isolated_negative_limits");
    active=t>=.10 & t<.54;
    w(active,:)=repmat(direction*[4 4 4 .8 .8 .8],sum(active),1);
end
u=Simulink.SimulationInput(h);
u=u.setExternalInput([[t w];t(end)+dt zeros(1,6)]);
u=u.setModelParameter('Solver','FixedStepDiscrete','FixedStep',num2str(dt), ...
    'StopTime','.94','SignalLogging','on','ReturnWorkspaceOutputs','on');
r=sim(u); s=readLogs(r.logsout);
resetOracle(cfg); q=cfg.home_rad; ref=zeros(numel(t),6);
for k=1:numel(t), [q,~,ref(k,:)]=rb2_minimal_controller('step',q,w(k,:),dt); end
raw=struct('t',t,'w',w,'twist',align(s.twist,t,'nearest'), ...
    'published',align(s.published,t,'nearest'),'oracle',ref);
end

function [comparison,m,c,p,msg]=assess(here,a,s,cfg,sample,cmd)
dt=cfg.sampleTime_s;
t=s.mapper_qd.t;
assert(abs(t(1))<=1e-12 && all(abs(diff(t)-dt)<=1e-12), ...
    'visible:ControlClock','Unexpected mapper publication clock.');
input=align(s.ema_wrench,t,'nearest'); q=align(s.q,t,'nearest');
ref=zeros(numel(t),6); nativeQd=ref;
if ~a.standalone
    resetOracle(cfg);
    for k=1:numel(t)
        [~,nativeQd(k,:),ref(k,:)]=rb2_minimal_controller('step',q(k,:),input(k,:),dt);
    end
end
cmdExpected=[zeros(1,6);nativeQd(1:end-1,:)];
standaloneMsg="";
if a.standalone
    % The preserved root Inport uses zero-order hold on exact timestamps.
    % A timestamp slightly after the solver tick belongs to the next hold.
    cmdExpected=interp1(sample,cmd,t,'previous');
    if isfield(s,'provided_qd')
        cmdExpected=align(s.provided_qd,t,'nearest');
    end
    nominal=interp1(sample,cmd,t,'nearest');
    m.nominalCmdError=maxabs(cmdExpected-nominal);
    m.nominalCmdTicks=nnz(any(abs(cmdExpected-nominal)>1e-12,2));
    standaloneMsg=sprintf("; nominal grid vs held input %.5g rad/s on %d ticks", ...
        m.nominalCmdError,m.nominalCmdTicks);
end
testW=zeros(size(t));
active=t>=a.start-1e-13 & t<a.release-1e-13;
testW(active)=1;
if ~a.hold, testW(active)=sin(pi*(t(active)-a.start)/(a.release-a.start)).^2; end
expectedW=testW*a.w;
m.ctrl=[maxabs(ref-align(s.twist_now,t,'nearest')), ...
     maxabs([zeros(1,6);ref(1:end-1,:)]-align(s.twist_published,t,'nearest')), ...
     maxabs([zeros(1,6);nativeQd(1:end-1,:)]-align(s.mapper_qd,t,'nearest')), ...
     maxabs(cmdExpected-align(s.selected_qd,t,'nearest')), ...
     maxabs(expectedW-align(s.applied_wrench,t,'nearest'))];
validCtrl=all(m.ctrl<=1e-12);
m.qExcursion=maxabs(s.q.x-s.q.x(1,:));
m.peakSpeed=maxabs(s.qd.x);
m.tcpShift=max(vecnorm(s.tcp.x-s.tcp.x(1,:),2,2));
m.tcpReleaseDrift=NaN;
if a.release<=s.tcp.t(end)
    m.tcpReleaseDrift=norm(s.tcp.x(end,:)-align(s.tcp,a.release,'linear'));
end
m.maxTau=max(abs(s.torque.x),[],1);
m.limited_s=sum(s.torque_limited.x>0,1)*dt;
p0=velocity_parameters();
validPlant=m.qExcursion<=.08 && m.peakSpeed<=.5 && all(m.maxTau<=p0.limit+1e-12) ...
    && maxabs(s.q.x(1,:)-cfg.home_rad)<=1e-8 && maxabs(s.qd.x(1,:))<=1e-8 ...
    && abs(s.q.t(end)-a.stop)<=1e-8;
for key=string(fieldnames(s)).', validPlant=validPlant && all(isfinite(s.(key).x),'all'); end
comparison=struct('t',t,'ema',input,'twistOracle',ref,'mapperOracle',nativeQd);
directionMsg="";
if a.axis>0
    command=align(s.selected_qd,t,'nearest'); measured=align(s.qd,t,'nearest');
    robot=testbench_robot(cfg); commandComponent=zeros(numel(t),1); measuredComponent=commandComponent;
    j=[4:6 1:3]; j=j(a.axis);
    for k=1:numel(t)
        jac=geometricJacobian(robot,q(k,:),'tcp');
        commandComponent(k)=jac(j,:)*command(k,:).';
        measuredComponent(k)=jac(j,:)*measured(k,:).';
    end
    window=t>=.23 & t<=.39;
    m.signedMapped=a.sgn*mean(commandComponent(window));
    m.signedMeasured=a.sgn*mean(measuredComponent(window));
    comparison.commandComponent=commandComponent; comparison.measuredComponent=measuredComponent;
    validCtrl=validCtrl && m.signedMapped>0;
    validPlant=validPlant && m.signedMeasured>0;
    directionMsg=sprintf("; signed mapped/measured %.5g / %.5g (m/s or rad/s)",m.signedMapped,m.signedMeasured);
end
archiveMsg="";
if strlength(a.archive)>0
    archive=fullfile(fileparts(here),'results','20261125_142820_293',a.archive+".mat");
    if isfile(archive)
        prior=load(archive,'s');
        x={'q','qd','tcp','torque'}; old={'q_measured','qd_measured','tcp_measured','tau_applied'};
        for j=1:4
            k=x{j}; b=prior.s.(old{j});
            m.archiveDiff(j)=maxabs(s.(k).x-align(b,s.(k).t,'linear'));
        end
        validPlant=validPlant && all(m.archiveDiff<=1e-8);
        archiveMsg=sprintf("; old plant q/qd/TCP/tau max %s",mat2str(m.archiveDiff,3));
    else
        archiveMsg="; old-plant comparison unavailable";
    end
end
c=verdict(validCtrl); p=verdict(validPlant);
msg=sprintf("controller twist/published/qd/selected/wrench max %s; q %.4g rad; qd %.4g rad/s; TCP %.4g mm; TCP after release %.4g mm; limiting ticks %d", ...
    mat2str(m.ctrl,3),m.qExcursion,m.peakSpeed,1e3*m.tcpShift,1e3*m.tcpReleaseDrift,round(sum(m.limited_s)/dt)) ...
    +string(directionMsg)+string(archiveMsg)+string(standaloneMsg);
end

function y=maxabs(x), y=max(abs(x),[],'all'); end
function resetOracle(cfg)
d=cfg.demo;
rb2_minimal_controller('reset',cfg.home_rad,d.virtualMass,d.virtualDamping, ...
    d.admittanceSpeedLimit,d.admittanceAccelerationLimit,d.jointSpeedLimit_rad_s, ...
    d.fullJacobianMinimumSigma,cfg.sampleTime_s);
end
function mark(model,block,index,label)
p=get_param([model '/' block],'PortHandles');
set_param(p.Outport(index),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',label);
end
function s=readLogs(logs)
s=struct;
for k=1:logs.numElements
    a=logs.getElement(k); v=a.Values; t=v.Time(:);
    if v.IsTimeFirst, x=reshape(v.Data,numel(t),[]);
    else, x=reshape(v.Data,[],numel(t)).'; end
    [t,i]=unique(t,'last'); s.(a.Name)=struct('t',t,'x',x(i,:));
end
end
function x=align(s,t,method)
x=interp1(s.t,s.x,min(max(t,s.t(1)),s.t(end)),method);
end
function x=verdict(good)
if good, x="PASS"; else, x="FAIL"; end
end
function msg=saveError(folder,tag,err)
details=getReport(err,'extended','hyperlinks','off');
save(fullfile(folder,tag+"_exception.mat"),'details');
msg=regexprep(err.message,'[\r\n]+',' ');
end
