
% plots a bunch of things from different logs

% plot altitude


% list of log files

% TODO: generate?

files = { '2014-02-12-crash-on-takeoff/lcmlog_2014_02_12_02.mat'
          '2014-02-12-crash-on-takeoff/lcmlog_2014_02_12_04.mat'
          '2014-02-04-outside-full-tether/lcmlog_2014_02_04_02.mat'
          '2014-02-04-outside-full-tether/lcmlog_2014_02_04_03.mat'};

dir_prefix = '/home/abarry/rlg/logs/';
close all
for i=1:length(files)
    
    dir = dir_prefix;
    filename = files{i};
    % load log files
    loadDeltawing
    
    
    % ok, we've got the log file, make plots!
    
    figure
    % get interseting times for altitude
    [start_times, end_times] = FindActiveTimes(baro.logtime, baro.altitude, mean(baro.altitude) + 3);
    
    plot(baro.logtime, baro.altitude * 3.28084);
    xlim([start_times(1)- 10, end_times(end)+20]);
    xlabel('Time (s)');
    ylabel('Altitude (ft)');
    title(filename);
    
    
end
