%% FULL SYSTEM - INITIALIZATION

%% 1. Admittance and F/T sensor
init_admittance;

admittanceInitialQ = initialQ;
admittanceTs = Ts;

%% 2. Trajectory initialization
trajectoryType = "line";

init_trajectory;

assert(abs(Ts - admittanceTs) < 1e-12, ...
    'Controller sample times do not match.');

%% 3. Operating mode
% 0 = IDLE
% 1 = TEACHING
% 2 = PLAYBACK

robotMode = 0;

%% 4. Select initial robot configuration
if robotMode ~= 2
    initialQ = admittanceInitialQ;
    initialQ_deg = rad2deg(initialQ);
end

%% 5. Initialization summary
modeNames = ["IDLE", "TEACHING", "PLAYBACK"];

fprintf('\n=== FULL SYSTEM INITIALIZED ===\n');
fprintf('Operating mode: %s\n', modeNames(robotMode+1));
fprintf('Control frequency: %.0f Hz\n', 1/Ts);
fprintf('Plant step: %.4f s\n', plantTs);

disp('Initial joint angles [deg]:');
disp(initialQ_deg);