%% PARAMETER ESTIMATION - EXPERIMENT CONFIGURATION

jointID = 1;

% Preliminary external test fixture parameters
J_load = 0.05;     % kg*m^2
B_load = 0.02;     % Nm*s/rad

% Experiment sampling
Ts_exp = 0.002;

% Synthetic velocity step
t_exp = (0:Ts_exp:3)';

qd_cmd = zeros(size(t_exp));
qd_cmd(t_exp >= 0.5 & t_exp < 2) = 0.2;

qd_cmd_ts = timeseries(qd_cmd,t_exp);