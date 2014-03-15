
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
    
    [AX, H1, H2] = plotyy(baro.logtime, baro.altitude * 3.28084, u.logtime, ServoPercent(u.throttle));
    
    %hold on
    %plot(est.logtime, est.pos.z*3.28084, 'r');
    
    ylabel('Altitude (ft)');
    
    SetupPlotScript
    
    saveasAll([log_save_dir log.name '/' log.number '-alt'], 15, AX)
    
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
    
    [AX, H1, H2] = plotyy(imu.logtime, totalG, u.logtime, ServoPercent(u.throttle));
    
    ylabel('Acceleration, All Axes (G)');
    
    SetupPlotScript
    
    saveasAll([log_save_dir log.name '/' log.number '-accel'], 15, AX)
    
end

%% gyro

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
    
    
    [AX, H1, H2] = plotyy(imu.logtime, imu.gyro.x, u.logtime, ServoPercent(u.throttle));
    
    ylabel('Gyro X-axis (rad/sec)');
    
    SetupPlotScript
    
    saveasAll([log_save_dir log.name '/' log.number '-gyro'], 15, AX)
    
end

%% GPS


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
    
    
    [AX, H1, H2] = plotyy(gps.logtime, gps.speed*2.23694, u.logtime, ServoPercent(u.throttle));
    
    ylabel('GPS Speed (MPH)');
    
    SetupPlotScript
    
    saveasAll([log_save_dir log.name '/' log.number '-gps-speed'], 15, AX)
    
end


%% battery

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
    
    
    [AX, H1, H2] = plotyy(battery.logtime, battery.voltage, u.logtime, ServoPercent(u.throttle));
    
    ylabel('Battery Voltage (V)');
    
    SetupPlotScript
    
    %saveasAll([log_save_dir log.name '-' log.number '-battery'], 15, AX)
    
end


%% airspeed

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
    
    
    [AX, H1, H2] = plotyy(baro.logtime, baro.airspeed*2.23694, u.logtime, ServoPercent(u.throttle));
    
    ylabel('Speed (MPH)');
    
    hold on
    plot(gps.logtime, gps.speed*2.23694, 'r')
    ylim(AX(1), [0 45])
    
    hlegend = legend('Airspeed', 'GPS');
    
    SetupPlotScript
    
    
    
    saveasAll([log_save_dir log.name '/' log.number '-airspeed'], 15, AX)
    
end

%% stereo

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
    
    
    thistime = (stereo.utime - stereo.utime(1)) / 1e6;
    
    framerate = 1./diff(thistime);
    
    %[AX, H1, H2] = plotyy(thistime(2:end), framerate, u.logtime, ServoPercent(u.throttle));
    
    plot(thistime(2:end), framerate, 'x');
    title([log.name '.' log.number]);

    
    xlim([start_times(1)- 10, end_times(end)+20]);
    
    xlabel('Time (s)')
    ylabel('Framerate (fps)');
    grid on
    
    hold on
    plot([0 thistime(end)], [100 100], 'k');
   
    

    
    %SetupPlotScript
    
    saveasAll([log_save_dir log.name '/' log.number '-stereo-framerate'], 15)
    
end




