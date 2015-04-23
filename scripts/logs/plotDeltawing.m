
% load the log

%dir = '2013-11-05-delta-crash/';
%filename = 'lcmlog_2013_11_05_00.mat';
clear
dir = '2015-04-21-field-test/gps-logs/';
filename = 'lcmlog_2015_04_21_04.mat';

addpath('/home/abarry/simflight/');

dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

loadDeltawing


% get start and end times
[throttle_start, throttle_end] = FindActiveTimes(u.logtime, u.throttle, 1150);

% get altitude start and end times
[alt_start, alt_end] = FindActiveTimes(altimeter.logtime, altimeter.altitude, 7);
% now plot relevant things

start_times = throttle_start - 1;
end_times = throttle_end + 1;

figure(1)
clf
plot(altimeter.logtime, altimeter.altitude);
xlim([start_times(1) - 1, end_times(end) + 1]);
title('Barometric Altitude');
xlabel('Log time (s)');
ylabel('Altitude');

hold on

plot(airspeed.logtime, airspeed.airspeed.*.44704,'r');
title('Airspeed');
xlabel('Log time (s)');
ylabel('Airspeed / Altitude');
legend('Altitude', 'Airspeed');

figure(2)
plot(u.logtime, u.throttle);
xlim([start_times(1) - 1, end_times(end) + 1]);
xlabel('Time (s)')
ylabel('Pulse (ms)');

hold on
plot(u.logtime, u.elevonL, 'r');
plot(u.logtime, u.video_record, 'k');
legend('Throttle', 'ElevonL', 'Video rec');

figure(3)
plot(u.logtime, u.is_autonomous);

% 
% figure(3)
% plot(est.logtime, est.rotation_rate.z);
% xlabel('Time (s)');
% ylabel('Rotation rate Z');
% 
% figure(4);
% plot(gps.logtime, gps.x);
% plot(est.logtime, est.rotation_rate.z);
% xlabel('Time (s)');
% ylabel('GPS X-axis');

% figure(5)
% plot3(gps.x, gps.y, gps.z)
% 
% figure(6)
% clf
% q = [est.orientation.q0 est.orientation.q1 est.orientation.q2 est.orientation.q3];
% [yaw, pitch, roll] = quat2angle(q); rad2deg([r p y])
% 
% plot(est.logtime, rad2deg(pitch));
% hold on
% plot(baro.logtime, baro.altitude,'r');
% legend('pitch', 'altitude');
% xlim([start_times(1) - 1, end_times(end) + 1]);