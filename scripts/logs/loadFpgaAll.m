
clear
log_number = 9;
fpga_log_number = 4;

dir = '2014-10-13-fpga-flight2/mat/';
filename = ['log0' num2str(log_number) '_with_state_est_fpga_log_' num2str(fpga_log_number) '.mat'];
%filename = ['lcmlog_2014_10_13_0' num2str(log_number) '.mat'];



dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

loadDeltawing


%fpga = LoadFpgaStereo(['/home/abarry/MIT/RLG/logs/2014-10-10-fpga-flight1/boston_flight_logs/awesome' num2str(log_number) '/log.csv']);
fpga = LoadFpgaStereo(['/home/abarry/MIT/RLG/stereo-compare-paper/data/2014-10-13-fpgaflight' num2str(fpga_log_number) '/log.csv']);

% align FPGA and other data

% to align the data, we find the first timestamp that the FPGA recorded on
% then we find the nearest utime the u.utime data and that gives us a
% mapping to u.logtime.


[~, min_index] = min(abs(fpga.plane_time(1) - imu.utime));

time_diff_in_us = fpga.plane_time(1) - imu.utime(min_index);

time_diff_in_sec = time_diff_in_us * 1e-6;

nearby_logtime = imu.logtime(min_index);

% if time_diff is positive, it means that the first frame happened after
% the point on imu that we found.

% that means, to correct to logtime, we should add it

logtime_at_time_us_one = nearby_logtime + time_diff_in_sec;

fpga.logtime = fpga.time_us * 1e-6 + logtime_at_time_us_one - fpga.time_us(1)*1e-6;

clf
plot(imu.logtime, imu.accel.x)
hold on
plot(fpga.logtime, fpga.frame, 'r');
