%% load everything

% add path for the log loader

pass_number = 1;

%start_frame_pass = [ 1772 675 559 ];
%end_frame_pass = [ 2528 1262 1594 ];
start_frame_pass = [ 1801 852 696 ];
end_frame_pass = [ 2305 1112 1416 ];



addpath('../../scripts/logs');


dir = '../../logs/logs/2014-04-18-near-goalposts/bm-stereo/';
filename = ['pass' num2str(pass_number) '_disp3_3.mat'];
%filename = ['pass' num2str(pass_number) '_disp3_random3.mat'];
%filename = ['pass' num2str(pass_number) '_fix_random2.mat'];

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


% load bounding box data
bbox = LoadBoundingBox(['box_clicking/pass' num2str(pass_number) '.csv']);

%% remove hits outside of bounding box for bm stereo


bm_stereo_boxed = ProcessBoundingBoxBmStereo(bm_stereo, bbox);


%% start processing

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
  error(['Requested start frame is earlier than we have data for (first frame is: ' num2str(bm_stereo_aligned.frame_number(1)) ').']);
else
  bm_start = bm_start + diff_start2;
  stereo_start = stereo_start + diff_start2;
end


diff_end2 = bm_stereo_aligned.frame_number(end) - end_frame_pass(pass_number);

if (diff_end2 < 0)
  error(['Requested end frame is later than we have data for (last frame is: ' num2str(bm_stereo_aligned.frame_number(end)) ').']);
else
  bm_end = bm_end - diff_end2;
  stereo_end = stereo_end - diff_end2;
end


bm_stereo_aligned.frame_number = bm_stereo.frame_number(bm_start:bm_end);
stereo_octomap_aligned.frame_number = stereo_octomap.frame_number(stereo_start:stereo_end);

bm_stereo_aligned.x = bm_stereo.x(bm_start:bm_end, :);
bm_stereo_aligned.y = bm_stereo.y(bm_start:bm_end, :);
bm_stereo_aligned.z = bm_stereo.z(bm_start:bm_end, :);

bm_stereo_aligned.utime = bm_stereo.utime(stereo_start:stereo_end);
bm_stereo_aligned.number_of_points = bm_stereo.number_of_points(stereo_start:stereo_end);
bm_stereo_aligned.video_number = bm_stereo.video_number(stereo_start:stereo_end);
bm_stereo_aligned.logtime = bm_stereo.logtime(stereo_start:stereo_end);





stereo_octomap_aligned.utime = stereo_octomap.utime(stereo_start:stereo_end);
stereo_octomap_aligned.number_of_points = stereo_octomap.number_of_points(stereo_start:stereo_end);
stereo_octomap_aligned.video_number = stereo_octomap.video_number(stereo_start:stereo_end);
stereo_octomap_aligned.logtime = stereo_octomap.logtime(stereo_start:stereo_end);

stereo_octomap_aligned.x = stereo_octomap.x(stereo_start:stereo_end, :);
stereo_octomap_aligned.y = stereo_octomap.y(stereo_start:stereo_end, :);
stereo_octomap_aligned.z = stereo_octomap.z(stereo_start:stereo_end, :);

stereo_octomap_aligned.frame_x = stereo_octomap.frame_x(stereo_start:stereo_end, :);
stereo_octomap_aligned.frame_y = stereo_octomap.frame_y(stereo_start:stereo_end, :);

bm_stereo_aligned.frame_number(1:5)
stereo_octomap_aligned.frame_number(1:5)

bm_stereo_aligned.frame_number(end-5:end)
stereo_octomap_aligned.frame_number(end-5:end)

%%


%stereo_octomap_filtered = FilterForInImage(stereo_octomap_aligned, 105, 376);
stereo_octomap_filtered = FilterForInImage(stereo_octomap_aligned, 132, 376, 51, 223);


distances = SmallestDistance(stereo_octomap_filtered.x, stereo_octomap_filtered.y, stereo_octomap_filtered.z, ...
  bm_stereo_aligned.x, bm_stereo_aligned.y, bm_stereo_aligned.z, 1.5);

%% process and plot

%real_dists = distances(find(distances-1e8));
real_dists = distances(find(distances));




hist(real_dists,0:.1:ceil(max(real_dists)));
xlabel('Minimum separation (meters)')
ylabel('Number of pixels')
title(strrep(filename, '_','-'));
xlim([-1 11]);
ylim([0 500]);
grid on

real_dists_array{pass_number} = real_dists;

%% sum up
figure(20)
real_sum = [];
for i = 1 : length(real_dists_array)
  real_sum = [real_sum; real_dists_array{i}];
end
hist(real_sum(:,1), 0:.1:ceil(max(real_sum(:,1))));
xlabel('Minimum separation (meters)')
ylabel('Number of pixels')
title('3 Passes');
xlim([-1 11]);
ylim([0 1500]);
grid on


