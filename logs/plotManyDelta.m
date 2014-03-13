
% plots a bunch of things from different logs

% plot altitude


% list of log files

% TODO: generate?

files = { '2014-02-12-crash-on-takeoff/lcmlog_2014_02_12_02.mat'
          '2014-02-12-crash-on-takeoff/lcmlog_2014_02_12_04.mat'
          '2014-02-04-outside-full-tether/lcmlog_2014_02_04_02.mat'
          '2014-02-04-outside-full-tether/lcmlog_2014_02_04_03.mat'};

dir_prefix = '/home/abarry/rlg/logs/';

log_save_dir = '/home/abarry/rlg/talks/figures/plots/delta/';

%% altitude

for i=1:length(files)
    
    dir = dir_prefix;
    filename = files{i};
    % load log files
    loadDeltawing
    
    
    % ok, we've got the log file, make plots!
    
    figure(i)
    clf
    % get interseting times for altitude
    [start_times, end_times] = FindActiveTimes(baro.logtime, baro.altitude, mean(baro.altitude) + 3);
    
    [AX, H1, H2] = plotyy(baro.logtime, baro.altitude * 3.28084, u.logtime, u.throttle);
    xlim(AX(1), [start_times(1)- 10, end_times(end)+20]);
    xlim(AX(2), [start_times(1)- 10, end_times(end)+20]);
    xlabel('Time (s)');
    ylabel('Altitude (ft)');
    ylabel(AX(2), 'Throttle (ms pulse)')
    title([log.name '.' log.number]);
    grid on
    
    
    saveasAll([log_save_dir log.name '-' log.number '-alt'], 15, AX)
    
end


%% accelerations


for i=1:length(files)
    
    dir = dir_prefix;
    filename = files{i};
    % load log files
    loadDeltawing
    
    
    % ok, we've got the log file, make plots!
    
    figure(i)
    clf
    % get interseting times for altitude
    [start_times, end_times] = FindActiveTimes(baro.logtime, baro.altitude, mean(baro.altitude) + 3);
    
    totalG = [imu.accel.x, imu.accel.y, imu.accel.z];
    totalG = sqrt(sum(abs(totalG).^2,2))/9.8;
    
    [AX, H1, H2] = plotyy(imu.logtime, totalG, u.logtime, u.throttle);
    xlim(AX(1), [start_times(1)- 10, end_times(end)+20]);
    xlim(AX(2), [start_times(1)- 10, end_times(end)+20]);
    xlabel('Time (s)');
    ylabel('Accel (x)');
    ylabel(AX(2), 'Accel (y)')
    title([log.name '.' log.number]);
    hold on
    plot([0 1e5], [1 1], 'k')
    
    grid on
    
    saveasAll([log_save_dir log.name '-' log.number '-accel'], 15, AX)
    
    
end