%% TRAJECTORY TRACKING ANALYSIS

%% 1. Run selected trajectory

model = 'Trajectory_Testbench';

in = Simulink.SimulationInput(model);

in = in.setModelParameter( ...
    'ReturnWorkspaceOutputs', 'on');

outTrajectory = sim(in);


%% 2. Extract actual TCP position

logs = outTrajectory.logsout;

tcpSignal = logs.getElement('tcp_position_m');
tcpTS = tcpSignal.Values;

tActual = tcpTS.Time;

pActual = zeros(numel(tActual),3);

for k = 1:numel(tActual)
    pActual(k,:) = reshape( ...
        getdatasamples(tcpTS,k),1,3);
end


%% 3. Extract desired TCP path

pDesired = trajectoryData.pPath.';


%% 4. Plot desired vs actual trajectory
figure('Name','Trajectory Tracking Analysis', ...
       'Color','w');

ax = axes;
hold(ax,'on');

% Desired trajectory - blue dashed
plot3(pDesired(:,1), pDesired(:,2), pDesired(:,3), ...
      '--','Color',[0 0.35 0.85],'LineWidth',2.5);

% Actual trajectory - red solid
plot3(pActual(:,1), pActual(:,2), pActual(:,3), ...
      '-','Color',[0.85 0.15 0.15],'LineWidth',2);

% Start and endpoint
scatter3(pDesired(1,1),pDesired(1,2), ...
         pDesired(1,3),70,[0 0.6 0.2],'filled');

scatter3(pDesired(end,1),pDesired(end,2), ...
         pDesired(end,3),70,'k','filled');

xlabel('X [m]','Color','k');
ylabel('Y [m]','Color','k');
zlabel('Z [m]','Color','k');

title("TCP Trajectory Tracking - " + trajectoryType, ...
      'Interpreter','none','Color','k');

legend('Desired Path','Actual Path', ...
       'Start','Desired End', ...
       'Location','best', ...
       'Color','w','TextColor','k');

% Explicit light-theme formatting
set(ax,'Color','w', ...
       'XColor','k', ...
       'YColor','k', ...
       'ZColor','k', ...
       'GridColor',[0.4 0.4 0.4], ...
       'GridAlpha',0.2, ...
       'FontSize',12, ...
       'LineWidth',1);

grid on;
axis equal;
view(3);
rotate3d on;

%% 5. TIME-ALIGNED TRACKING ERROR

refTS = logs.getElement('t_ref').Values;
statusTS = logs.getElement('settling_status').Values;

% Synchronize the trajectory clock with actual TCP samples
tReference = interp1( ...
    refTS.Time(:), refTS.Data(:), ...
    tActual(:), 'previous', 'extrap');

tDesired = trajectoryData.tGlobal(:);

tReference = max(tDesired(1), ...
    min(tReference,tDesired(end)));

% Desired position at the actual reference time
pDesiredSync = interp1( ...
    tDesired,pDesired,tReference,'linear');

positionError_mm = ...
    1000 * vecnorm(pDesiredSync-pActual,2,2);

rmsError = sqrt(mean(positionError_mm.^2));
maxError = max(positionError_mm);
finalError = positionError_mm(end);


%% 6. EXECUTION SUMMARY

statusNames = ["EXECUTING","SETTLING", ...
               "COMPLETE","TIMEOUT"];

finalStatus = statusNames(statusTS.Data(end));

fprintf('\n=== TRAJECTORY EXECUTION SUMMARY ===\n');
fprintf('Trajectory: %s\n',trajectoryType);
fprintf('Status: %s\n',finalStatus);
fprintf('Planned duration: %.3f s\n',trajectoryDuration);
fprintf('Simulation duration: %.3f s\n',tActual(end));
fprintf('Reference time reached: %.3f s\n',tReference(end));
fprintf('RMS error: %.3f mm\n',rmsError);
fprintf('Maximum error: %.3f mm\n',maxError);
fprintf('Final active-target error: %.3f mm\n',finalError);


%% 7. PLOT TRACKING ERROR AND REFERENCE CLOCK

figure('Name','Execution Analysis','Color','w');

tiledlayout(2,1);

nexttile;
plot(tActual,positionError_mm,'LineWidth',1.5);
ylabel('Position error [mm]');
title('TCP Tracking Error','Color','k');
grid on;
set(gca,'Color','w','XColor','k','YColor','k');

nexttile;
plot(tActual,tReference,'LineWidth',1.5);
xlabel('Simulation time [s]');
ylabel('Reference time [s]');
title('Trajectory Clock','Color','k');
grid on;
set(gca,'Color','w','XColor','k','YColor','k');