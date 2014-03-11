
% load the log

%dir = '2013-11-05-delta-crash/';
%filename = 'lcmlog_2013_11_05_00.mat';

dir = '2014-02-12-crash-on-takeoff/';
filename = 'lcmlog_2014_02_12_02.mat';



dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

loadDeltawing

% now plot relevant things

figure(1)
clf
plot(baro.logtime, baro.altitude);
title('Barometric Altitude');
xlabel('Log time (s)');
ylabel('Altitude');

hold on

plot(baro.logtime, baro.airspeed.*.44704,'r');
title('Airspeed');
xlabel('Log time (s)');
ylabel('Airspeed');
legend('Altitude', 'Airspeed');

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
[pitch, roll, yaw] = quat2angle(q, 'YXZ');

plot(est.logtime, pitch*10);
hold on
plot(baro.logtime, baro.altitude,'r');