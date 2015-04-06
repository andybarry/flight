
% load logs

clear
dir = '2015-03-31-field-test/gps-logs/';
filename = 'lcmlog_2015_03_31_11.mat';



dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

loadDeltawing


[t_start t_end] = FindActiveTimes(u.logtime, u.is_autonomous, 0.5);

longrun_start = t_start(6);
longrun_end = t_end(6);

% TODO: compute control and then check