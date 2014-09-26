
% load the log

%dir = '2013-11-05-delta-crash/';
%filename = 'lcmlog_2013_11_05_00.mat';

dir = '2014-04-18-near-goalposts/mat/';
filename = 'pass1.mat';



dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

loadDeltawing


% get start and end times
[throttle_start, throttle_end] = FindActiveTimes(u.logtime, u.throttle, 1150);

% get altitude start and end times
[alt_start, alt_end] = FindActiveTimes(baro.logtime, baro.altitude, 7);
% now plot relevant things

start_times = throttle_start - 1;
end_times = throttle_end + 1;

figure(1)
clf
plot(baro.logtime, baro.altitude);
xlim([start_times(1) - 1, end_times(end) + 1]);
title('Barometric Altitude');
xlabel('Log time (s)');
ylabel('Altitude');

hold on

plot(baro.logtime, baro.airspeed.*.44704,'r');
title('Airspeed');
xlabel('Log time (s)');
ylabel('Airspeed / Altitude');
legend('Altitude', 'Airspeed');

figure(2)
plot(u.logtime, u.throttle);
xlim([start_times(1) - 1, end_times(end) + 1]);
xlabel('Time (s)')
ylabel('Throttle (pulse ms)');


figure(3)
plot(est.logtime, est.rotation_rate.z);
xlabel('Time (s)');
ylabel('Rotation rate Z');

figure(4);
plot(gps.logtime, gps.x);
plot(est.logtime, est.rotation_rate.z);
xlabel('Time (s)');
ylabel('GPS X-axis');

figure(5)
plot3(gps.x, gps.y, gps.z)

figure(6)
clf
q = [est.orientation.q0 est.orientation.q1 est.orientation.q2 est.orientation.q3];
[yaw, pitch, roll] = quat2angle(q); rad2deg([r p y])

plot(est.logtime, rad2deg(pitch));
hold on
plot(baro.logtime, baro.altitude,'r');
legend('pitch', 'altitude');
xlim([start_times(1) - 1, end_times(end) + 1]);