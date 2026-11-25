function model=build_visible_admittance()
% Rebuild only the derived model, starting from the preserved minimal baseline.
here=fileparts(mfilename('fullpath'));
parent=fileparts(here); root=fileparts(parent);
addpath(here,parent,root,fullfile(root,'src'),fullfile(root,'control'), ...
    fullfile(root,'mechanics','URDF_Assembly_Robot_ver2','meshes'),'-begin');
model='robot_ver2_velocity_visible';
file=fullfile(here,[model '.slx']);
if ~isfile(file), copyfile(fullfile(parent,'robot_ver2_velocity_minimal.slx'),file); end
load_system(file);
if getSimulinkBlockHandle([model '/Cartesian Admittance'])>0, return; end
load_system('simulink');
prefix=[model '/'];
m=get_param(model,'ModelWorkspace');
cfg=testbench_config(); d=cfg.demo;
m.assignin('admittanceTs',cfg.sampleTime_s);
m.assignin('admittanceMass',d.virtualMass);
m.assignin('admittanceDamping',d.virtualDamping);
m.assignin('admittanceAccelerationLimit',d.admittanceAccelerationLimit);
m.assignin('admittanceSpeedLimit',d.admittanceSpeedLimit);
delete_block([prefix 'DriveAndController']);
delete_block([prefix 'AdmittanceReference']);
% Remove dangling branches left by the two obsolete blocks.
for line=find_system(model,'FindAll','on','SearchDepth',1,'Type','line').'
    if ishandle(line)
        src=get_param(line,'SrcPortHandle'); dst=get_param(line,'DstPortHandle');
        if isequal(src,-1) || (all(dst==-1) && isempty(get_param(line,'LineChildren')))
            delete_line(line);
        end
    end
end
notes=find_system(model,'FindAll','on','SearchDepth',1,'Type','annotation');
for h=notes(:).'
    if contains(get_param(h,'Name'),'laboratory comparison'), delete(h); end
end
move('SensorReference',[398 98 655 185]);
move('UnusedButterworth',[713 80 735 100]);
move('JointVelocityCommand',[775 330 825 352]);
move('UseStandaloneVelocity',[770 405 840 430]);
move('JointVelocitySelection',[985 356 1030 449]);
move('VelocityServo',[1125 356 1290 508]);
move('RobotPlant',[1445 350 1700 610]);
move('Measurements',[2080 335 2260 640]);
set_param([prefix 'UseStandaloneVelocity'],'Value','double(velocityCase.standalone)');
wrench=[prefix 'User Applied Wrench'];
sub(model,'User Applied Wrench',[60 96 291 184]);
buildWrench(wrench);
a=[prefix 'Cartesian Admittance'];
sub(model,'Cartesian Admittance',[815 108 1015 226]);
buildAdmittance(a);
sub(model,'Cartesian-to-Joint Mapper',[1110 111 1355 244]);
mapper=[prefix 'Cartesian-to-Joint Mapper'];
ins={'q measured [rad]','qd measured [rad per s]','Limited TCP twist','EMA wrench'};
for k=1:4, in(mapper,ins{k},k,24+44*k); end
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[mapper '/Original C Joint Mapper'], ...
    'FunctionName','visible_joint_mapper','Parameters','velocityCase.standalone','Position',[185 62 360 206]);
out(mapper,'Joint velocity [rad per s]',1,112,465);
for k=1:4, wire(mapper,[ins{k} '/1'],sprintf('Original C Joint Mapper/%d',k)); end
wire(mapper,'Original C Joint Mapper/1','Joint velocity [rad per s]/1');
wire(model,'User Applied Wrench/1','SensorReference/1','physical wrench at TCP');
wire(model,'User Applied Wrench/1','RobotPlant/2');
wire(model,'SensorReference/2','Cartesian Admittance/1','EMA wrench');
wire(model,'UseStandaloneVelocity/1','Cartesian Admittance/2');
wire(model,'Cartesian Admittance/1','Cartesian-to-Joint Mapper/3','limited TCP twist for this tick');
wire(model,'Cartesian Admittance/2','Measurements/5','published TCP twist');
wire(model,'SensorReference/2','Cartesian-to-Joint Mapper/4');
wire(model,'Cartesian-to-Joint Mapper/1','JointVelocitySelection/3','J1-J6 command [rad/s]');
wire(model,'RobotPlant/1','Cartesian-to-Joint Mapper/1');
wire(model,'RobotPlant/2','Cartesian-to-Joint Mapper/2');
sub(model,'Response Scopes',[1860 115 2040 272]);
s=[prefix 'Response Scopes'];
labels={'Applied wrench','Filtered wrench','Commanded qd','Measured qd','Measured q'};
for k=1:5, in(s,labels{k},k,40+55*k); end
scope(s,'Wrench  Applied and EMA',[402 88 535 144],2);
scope(s,'Joint Velocity  Command and Measured',[402 196 535 255],2);
scope(s,'Joint Displacement from Park  deg',[402 305 535 347],1);
block(s,'Sources/Constant','Initial q',[102 357 157 379],'Value','initialQ','SampleTime','inf');
block(s,'Math Operations/Sum','Subtract initial q',[207 310 233 345],'Inputs','+-');
block(s,'Math Operations/Gain','Radians to degrees',[269 311 330 344],'Gain','180/pi');
wire(s,'Applied wrench/1','Wrench  Applied and EMA/1','applied [N, N m]');
wire(s,'Filtered wrench/1','Wrench  Applied and EMA/2','filtered [N, N m]');
wire(s,'Commanded qd/1','Joint Velocity  Command and Measured/1','commanded J1-J6 [rad/s]');
wire(s,'Measured qd/1','Joint Velocity  Command and Measured/2','measured J1-J6 [rad/s]');
wire(s,'Measured q/1','Subtract initial q/1');
wire(s,'Initial q/1','Subtract initial q/2');
wire(s,'Subtract initial q/1','Radians to degrees/1');
wire(s,'Radians to degrees/1','Joint Displacement from Park  deg/1','measured J1-J6 [deg]');
wire(model,'User Applied Wrench/1','Response Scopes/1');
wire(model,'SensorReference/2','Response Scopes/2');
wire(model,'JointVelocitySelection/1','Response Scopes/3');
wire(model,'RobotPlant/2','Response Scopes/4');
wire(model,'RobotPlant/1','Response Scopes/5');
note(model,'Admittance mode: UseStandaloneVelocity = 0. Standalone command: 1.',[760 502]);
note(model,'True force and moment in world/base axes at TCP. EMA measurement feeds the controller only.',[59 38]);
note(a,'The mapper uses V(k) now. The delay publishes V(k-1) and stores the speed-clamped state.',[138 33]);
callback=['h=fileparts(get_param(bdroot,''FileName'')); p=fileparts(h); r=fileparts(p); ' ...
    'addpath(h,p,r,fullfile(r,''src''),fullfile(r,''control''),fullfile(r,''mechanics'',''URDF_Assembly_Robot_ver2'',''meshes''),''-begin''); clear h p r;'];
set_param(model,'PostLoadFcn',callback,'InitFcn',callback);
save_system(model,file);
    function move(n,position), set_param([prefix n],'Position',position); end
end

function buildAdmittance(s)
in(s,'EMA wrench',1,151); in(s,'Standalone',2,231);
block(s,'Sources/Constant','Zero wrench',[45 192 118 213],'Value','zeros(1,6)','SampleTime','admittanceTs');
block(s,'Signal Routing/Switch','Enable admittance',[145 136 179 202],'Criteria','u2 >= Threshold','Threshold','.5');
block(s,'Discrete/Unit Delay','Previous wrench',[215 190 266 225],'InitialCondition','zeros(1,6)','SampleTime','admittanceTs');
block(s,'Math Operations/Sum','W plus Wprev',[305 132 329 163],'Inputs','++');
gain(s,'Multiply by dt',[369 128 441 169],'admittanceTs');
gain(s,'Multiply by 2M minus D dt',[370 64 487 104],'2*admittanceMass-admittanceTs*admittanceDamping');
block(s,'Math Operations/Sum','Tustin numerator',[533 90 556 124],'Inputs','++');
gain(s,'Divide by 2M plus D dt',[595 87 731 127],'1./(2*admittanceMass+admittanceTs*admittanceDamping)');
block(s,'Math Operations/Sum','Subtract previous twist',[769 91 795 125],'Inputs','+-');
block(s,'Discontinuities/Saturation','Limit delta twist',[841 88 921 128], ...
    'UpperLimit','admittanceTs*admittanceAccelerationLimit','LowerLimit','-admittanceTs*admittanceAccelerationLimit');
block(s,'Math Operations/Sum','Add to previous twist',[959 89 983 125],'Inputs','++');
block(s,'Discontinuities/Saturation','Limit twist speed',[1024 87 1114 129], ...
    'UpperLimit','admittanceSpeedLimit','LowerLimit','-admittanceSpeedLimit');
block(s,'Discrete/Unit Delay','Previous limited twist',[724 282 837 325], ...
    'InitialCondition','zeros(1,6)','SampleTime','admittanceTs','Orientation','left');
out(s,'Twist for mapper update',1,102,1220); out(s,'Published TCP twist',2,368,1220);
wire(s,'Zero wrench/1','Enable admittance/1');
wire(s,'Standalone/1','Enable admittance/2');
wire(s,'EMA wrench/1','Enable admittance/3');
wire(s,'Enable admittance/1','Previous wrench/1');
wire(s,'Enable admittance/1','W plus Wprev/1');
wire(s,'Previous wrench/1','W plus Wprev/2');
wire(s,'W plus Wprev/1','Multiply by dt/1');
wire(s,'Multiply by dt/1','Tustin numerator/2');
wire(s,'Previous limited twist/1','Multiply by 2M minus D dt/1');
wire(s,'Multiply by 2M minus D dt/1','Tustin numerator/1');
wire(s,'Tustin numerator/1','Divide by 2M plus D dt/1');
wire(s,'Divide by 2M plus D dt/1','Subtract previous twist/1');
wire(s,'Previous limited twist/1','Subtract previous twist/2');
wire(s,'Subtract previous twist/1','Limit delta twist/1');
wire(s,'Limit delta twist/1','Add to previous twist/1');
wire(s,'Previous limited twist/1','Add to previous twist/2');
wire(s,'Add to previous twist/1','Limit twist speed/1');
wire(s,'Limit twist speed/1','Twist for mapper update/1');
wire(s,'Limit twist speed/1','Previous limited twist/1');
wire(s,'Previous limited twist/1','Published TCP twist/1');
end

function buildWrench(s)
block(s,'Sources/Digital Clock','Control Tick',[35 119 90 147],'SampleTime','admittanceTs');
block(s,'Sources/Constant','Push start',[35 173 90 198],'Value','pushStart','SampleTime','admittanceTs');
block(s,'Math Operations/Sum','Time since start',[135 123 159 163],'Inputs','+-');
gain(s,'Divide by duration and scale by pi',[191 120 332 167],'pi/(pushRelease-pushStart)');
block(s,'Math Operations/Trigonometric Function','Sine',[372 120 423 164],'Operator','sin');
block(s,'Math Operations/Math Function','Square',[463 120 519 164],'Operator','square');
block(s,'Sources/Step','Start gate',[363 245 418 276],'Time','pushStart','Before','0','After','1','SampleTime','admittanceTs');
block(s,'Sources/Step','Release gate',[363 300 418 330],'Time','pushRelease','Before','0','After','1','SampleTime','admittanceTs');
block(s,'Math Operations/Sum','Active window',[459 248 486 286],'Inputs','+-');
block(s,'Sources/Constant','Hold flag',[394 199 458 222],'Value','holdProfile','SampleTime','admittanceTs');
block(s,'Sources/Constant','Unit hold',[415 65 471 92],'Value','1','SampleTime','admittanceTs');
block(s,'Signal Routing/Switch','Pulse or hold',[558 103 593 204],'Criteria','u2 >= Threshold','Threshold','.5');
block(s,'Math Operations/Product','Gate waveform',[645 153 673 193],'Inputs','**');
block(s,'Sources/Constant','Force and moment components',[610 251 752 286], ...
    'Value','[Fx Fy Fz Mx My Mz]','SampleTime','admittanceTs');
block(s,'Math Operations/Product','Physical wrench',[799 152 835 195],'Inputs','**');
out(s,'Wrench at TCP',1,172,927);
wire(s,'Control Tick/1','Time since start/1'); wire(s,'Push start/1','Time since start/2');
wire(s,'Time since start/1','Divide by duration and scale by pi/1');
wire(s,'Divide by duration and scale by pi/1','Sine/1');
wire(s,'Sine/1','Square/1');
wire(s,'Unit hold/1','Pulse or hold/1'); wire(s,'Hold flag/1','Pulse or hold/2');
wire(s,'Square/1','Pulse or hold/3');
wire(s,'Start gate/1','Active window/1'); wire(s,'Release gate/1','Active window/2');
wire(s,'Pulse or hold/1','Gate waveform/1'); wire(s,'Active window/1','Gate waveform/2');
wire(s,'Gate waveform/1','Physical wrench/1'); wire(s,'Force and moment components/1','Physical wrench/2');
wire(s,'Physical wrench/1','Wrench at TCP/1');
mask=Simulink.Mask.create(s);
names={'Fx','Fy','Fz','Mx','My','Mz','pushStart','pushRelease','holdProfile'};
prompts={'Fx (N)','Fy (N)','Fz (N)','Mx (N m)','My (N m)','Mz (N m)', ...
    'Push start (s)','Push release (s)','Profile (0 smooth sine-squared pulse, 1 held push)'};
defaults={'.18','0','0','0','0','0','.10','.52','0'};
for k=1:numel(names)
    mask.addParameter('Type','edit','Name',names{k},'Prompt',prompts{k},'Value',defaults{k});
end
mask.Initialization=['assert(all(isfinite([Fx Fy Fz Mx My Mz pushStart pushRelease holdProfile])) && ' ...
    'isscalar(pushStart) && isscalar(pushRelease) && pushStart >= 0 && pushRelease > pushStart ' ...
    '&& isscalar(holdProfile) && any(holdProfile == [0 1]) && numel([Fx Fy Fz Mx My Mz]) == 6, ' ...
    '''Use six finite scalar components, valid push times and profile 0 or 1.'');'];
mask.Description='Physical wrench in world/base axes at the TCP. Identical source feeds RobotPlant and SensorReference.';
end

function sub(s,n,p), add_block('built-in/Subsystem',[s '/' n],'Position',p); end
function in(s,n,k,y)
block(s,'Ports & Subsystems/In1',n,[33 y-10 63 y+10],'Port',num2str(k),'PortDimensions','6');
if strcmp(n,'Standalone'), set_param([s '/' n],'PortDimensions','1'); end
end
function out(s,n,k,y,x)
block(s,'Ports & Subsystems/Out1',n,[x y-10 x+30 y+10],'Port',num2str(k));
end
function gain(s,n,p,g), block(s,'Math Operations/Gain',n,p,'Gain',g,'Multiplication','Element-wise(K.*u)'); end
function block(s,l,n,p,varargin), add_block(['simulink/' l],[s '/' n],'Position',p,varargin{:}); end
function scope(s,n,p,count)
block(s,'Sinks/Scope',n,p,'NumInputPorts',num2str(count));
c=get_param([s '/' n],'ScopeConfiguration');
c.ShowLegend=true;
c.LayoutDimensions=[count 1];
end
function wire(s,x,y,varargin)
h=add_line(s,x,y,'autorouting','on');
if nargin==4 && isempty(get_param(h,'Name')), set_param(h,'Name',varargin{1}); end
end
function note(s,txt,p), n=Simulink.Annotation(s,txt); n.Position=p; end
