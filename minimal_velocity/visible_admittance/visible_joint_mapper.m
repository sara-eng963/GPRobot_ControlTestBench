function visible_joint_mapper(b)
% Apply the original mapper and position guards to this tick's limited twist.
b.NumDialogPrms=1;
b.DialogPrmsTunable={'Nontunable'};
b.NumInputPorts=4; b.NumOutputPorts=1;
for k=1:4
    b.InputPort(k).Dimensions=6;
    b.InputPort(k).DatatypeID=0;
    b.InputPort(k).DirectFeedthrough=false;
end
b.OutputPort(1).Dimensions=6; b.OutputPort(1).DatatypeID=0;
cfg=testbench_config();
b.SampleTimes=[cfg.sampleTime_s 0];
b.RegBlockMethod('Start',@start);
b.RegBlockMethod('Outputs',@outputs);
b.RegBlockMethod('Update',@update);
b.RegBlockMethod('Terminate',@finish);
end
function start(b)
cfg=testbench_config();
assert(all(isfinite([cfg.demo.virtualMass cfg.demo.virtualDamping ...
    cfg.demo.admittanceSpeedLimit cfg.demo.admittanceAccelerationLimit cfg.sampleTime_s])));
assert(all([cfg.demo.virtualMass cfg.demo.virtualDamping ...
    cfg.demo.admittanceSpeedLimit cfg.demo.admittanceAccelerationLimit cfg.sampleTime_s]>0));
r=testbench_robot(cfg);
cache=testbench_collision_cache(r);
[g,cache]=testbench_safety(r,cfg.home_rad,cfg,cfg.home_rad,false,cache);
assert(g.accepted,'minimal:Park','%s',g.reason);
standalone=logical(b.DialogPrm(1).Data);
if ~standalone
    rb2_visible_mapper('reset',cfg.home_rad,cfg.demo.jointSpeedLimit_rad_s, ...
        cfg.demo.fullJacobianMinimumSigma,cfg.sampleTime_s);
end
s=struct('cfg',cfg,'robot',r,'cache',cache,'standalone',standalone, ...
    'q',cfg.home_rad,'previous',cfg.home_rad,'qd',zeros(1,6));
set_param(b.BlockHandle,'UserData',s,'UserDataPersistent','off');
end
function outputs(b)
s=get_param(b.BlockHandle,'UserData');
b.OutputPort(1).Data=s.qd(:);
end
function update(b)
s=get_param(b.BlockHandle,'UserData');
q=b.InputPort(1).Data(:).'; qd=b.InputPort(2).Data(:).';
cfg=s.cfg;
assert(all(isfinite([q qd])) && all(abs(qd)<=cfg.demo.jointSpeedLimit_rad_s));
if b.CurrentTime==0
    assert(max(abs(q-cfg.home_rad))<1e-8 && max(abs(qd))<1e-8);
end
[g,s.cache]=testbench_safety(s.robot,q,cfg,s.previous,true,s.cache);
assert(g.accepted,'minimal:MeasuredGuard','%s',g.reason);
if ~s.standalone
    twist=b.InputPort(3).Data(:).'; w=b.InputPort(4).Data(:).';
    assert(all(isfinite([w twist])),'visible:AdmittanceInput','Nonfinite wrench or limited twist.');
    [next,s.qd]=rb2_visible_mapper('step',q,twist,cfg.sampleTime_s);
    assert(max(abs(next-s.q))<=cfg.adapter.maxJointStep_rad);
    [g,s.cache]=testbench_safety(s.robot,next,cfg,s.q,true,s.cache);
    assert(g.accepted,'minimal:ReferenceGuard','%s',g.reason);
    s.q=next;
end
s.previous=q;
set_param(b.BlockHandle,'UserData',s);
end
function finish(b)
set_param(b.BlockHandle,'UserData',[]);
end
