% add path for the log loader

pass_number = 3;

start_frame_pass = [ 1801 852 696 ];
end_frame_pass = [ 2305 1119 1416 ];



addpath('../../scripts/logs');


dir = '../../logs/logs/2014-04-18-near-goalposts/bm-stereo/';
filename = ['pass' num2str(pass_number) '_bm.mat'];

loadDeltawing

% grab what we need from that load

bm_stereo = stereo_bm;

% dir = '../../logs/logs/2014-04-18-near-goalposts/bm-stereo/';
% filename = ['pass' num2str(pass_number) '_pushbroom.mat'];
% 
% loadDeltawing

clear STATE_ESTIMATOR_POSE battery_status stereoBmVals u
clear imu stereoReplayVals wind_groundspeed attitude est stereoVals 
clear wind_gspeed baro log stereo_bm wingeron_u baro_airspeed gps
clear servo_out stereo_replay battery gpsValues this_number


% everything is loaded and we have two main variables:
% bm_stereo and stereo_octomap

% we want to compare the distances from pushbroom to bm

% align based on frame number
frame_diff = bm_stereo.frame_number(1) - stereo_octomap.frame_number(1);

stereo_start = 1;
stereo_end = length(stereo_octomap.frame_number) - 1;

bm_start = 1;
bm_end = length(bm_stereo.frame_number) - 1;

if (frame_diff > 0)
  stereo_start = frame_diff+1;
  bm_start = 1;
  
elseif (frame_diff < 0)
  
  stereo_start = 1;
  bm_start = -frame_diff+1;
  
end

end_diff = bm_stereo.frame_number(end) - stereo_octomap.frame_number(end);

if (end_diff > 0)
  
  bm_end = bm_end - end_diff;
  
else
  
  stereo_end = stereo_end + end_diff;
  
  
end

% ok everything is aligned now.
% now figure out where we want to look
bm_stereo_aligned.frame_number = bm_stereo.frame_number(bm_start:bm_end);
stereo_octomap_aligned.frame_number = stereo_octomap.frame_number(stereo_start:stereo_end);

diff_start2 = start_frame_pass(pass_number) - bm_stereo_aligned.frame_number(1);

if (diff_start2 < 0)
  error('Requested start frame is earlier than we have data for.');
else
  bm_start = bm_start + diff_start2;
  stereo_start = stereo_start + diff_start2;
end


diff_end2 = bm_stereo_aligned.frame_number(end) - end_frame_pass(pass_number);

if (diff_end2 < 0)
  error('Requested end frame is later than we have data for.');
else
  bm_end = bm_end - diff_end2;
  stereo_end = stereo_end - diff_end2;
end


bm_stereo_aligned.frame_number = bm_stereo.frame_number(bm_start:bm_end);
stereo_octomap_aligned.frame_number = stereo_octomap.frame_number(stereo_start:stereo_end);

bm_stereo_aligned.x = bm_stereo.x(bm_start:bm_end, :);
bm_stereo_aligned.y = bm_stereo.y(bm_start:bm_end, :);
bm_stereo_aligned.z = bm_stereo.z(bm_start:bm_end, :);


stereo_octomap_aligned.x = stereo_octomap.x(stereo_start:stereo_end, :);
stereo_octomap_aligned.y = stereo_octomap.y(stereo_start:stereo_end, :);
stereo_octomap_aligned.z = stereo_octomap.z(stereo_start:stereo_end, :);

bm_stereo_aligned.frame_number(1:5)
stereo_octomap_aligned.frame_number(1:5)

bm_stereo_aligned.frame_number(end-5:end)
stereo_octomap_aligned.frame_number(end-5:end)

%%


distances = SmallestDistance(stereo_octomap_aligned.x, stereo_octomap_aligned.y, stereo_octomap_aligned.z, ...
  bm_stereo_aligned.x, bm_stereo_aligned.y, bm_stereo_aligned.z, 1);

%% process and plot

real_dists = distances(find(distances-1e8));
real_dists = real_dists(find(real_dists));




hist(real_dists,0:.1:ceil(max(real_dists)));
xlabel('Minimum separation (meters)')
ylabel('Number of pixels')
title(strrep(filename, '_','-'));
xlim([-1 10]);
grid on

