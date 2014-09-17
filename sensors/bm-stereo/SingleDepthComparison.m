% do single-depth comparisons

% clear
% 
% pass_number = 33;
% doFalseNeg = 0;
% sumfig = 20;
% enable_boxed = 0;
% 
% bm_depth_min = 4.7;
% bm_depth_max = 4.9;
% 
% new_dir = 2;

%% load everything

disp('Loading...');

start_frame_pass = zeros(1, 35);
end_frame_pass = zeros(1, 35);

start_frame_pass(1) = 1990;
start_frame_pass(2) = 989;
start_frame_pass(3) = 809;
start_frame_pass(6) = 62;

start_frame_pass(27) = 5313;
start_frame_pass(29) = 6111;
start_frame_pass(33) = 2851;


end_frame_pass(1) = 2046;
end_frame_pass(2) = 1112;
end_frame_pass(3) = 1416;
end_frame_pass(6) = 5630;

end_frame_pass(27) = 11198;
end_frame_pass(29) = 11962;

end_frame_pass(33) = 8794;



addpath('../../scripts/logs');


%dir = '../../logs/logs/2014-04-18-near-goalposts/bm-stereo/new/';
%filename = ['pass' num2str(pass_number) '.mat'];

if new_dir == 0
  dir = '../../logs/logs/2014-04-18-near-goalposts/bm-stereo/';
  filename = ['pass' num2str(pass_number) '_disp3_3.mat'];
elseif new_dir == 1
  dir = '../../logs/logs/2014-07-24-outside-stereo-test1/bm-stereo/';
  filename = ['pass' num2str(pass_number) '_log.mat'];
elseif new_dir == 2
  dir = '../../logs/logs/2014-07-24-outside-stereo-test2/bm-stereo/';
  filename = ['pass' num2str(pass_number) '_log.mat'];
  
end
%filename = ['pass' num2str(pass_number) '_disp3_random3.mat']/;
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

%% apply boxes

if enable_boxed == 1
  disp('Apply bounding boxes...');
  bbox = LoadBoundingBox(['box_clicking/pass' num2str(pass_number) '.csv']);


  bm_stereo_boxed = ProcessBoundingBoxBmStereo(bm_stereo, bbox);

  bm_stereo = bm_stereo_boxed;
end






%% crop bm_stero's depth

disp('Cropping depth...');

row_size = size(bm_stereo.x, 2);



bm_stereo_cropped.utime = bm_stereo.utime;
bm_stereo_cropped.frame_number = bm_stereo.frame_number;
bm_stereo_cropped.video_number = bm_stereo.video_number;
bm_stereo_cropped.logtime = bm_stereo.logtime;
% 

bm_stereo_cropped.x = zeros(size(bm_stereo.x));
bm_stereo_cropped.y = zeros(size(bm_stereo.y));
bm_stereo_cropped.z = zeros(size(bm_stereo.z));



for i = 1 : length(bm_stereo.frame_number)
  
  valid_ind = find(bm_stereo.z(i,:) > bm_depth_min & bm_stereo.z(i,:) < bm_depth_max);
  
  this_len = length(valid_ind);
  
  bm_stereo_cropped.number_of_points(i) = this_len;
  
  
  bm_stereo_cropped.x(i,:) = [ bm_stereo.x(i, valid_ind) zeros(1, row_size - this_len) ];
  bm_stereo_cropped.y(i,:) = [ bm_stereo.y(i, valid_ind) zeros(1, row_size - this_len) ];
  bm_stereo_cropped.z(i,:) = [ bm_stereo.z(i, valid_ind) zeros(1, row_size - this_len) ];
   
  if mod(i, 100) == 0
    disp(['Frame ' num2str(i) ' / ' num2str(length(bm_stereo.frame_number))]);
  end
  
end

%% align

disp('Aligning...');

% everything is loaded and we have two main variables:
% bm_stereo and stereo_octomap

% we want to compare the distances from pushbroom to bm

% align based on frame number


frame_diff = bm_stereo_cropped.frame_number(1) - stereo.frame_number(1);

stereo_start = 1;
stereo_end = length(stereo.frame_number) - 1;

bm_start = 1;
bm_end = length(bm_stereo_cropped.frame_number) - 1;

if (frame_diff > 0)
  stereo_start = frame_diff+1;
  bm_start = 1;
  
elseif (frame_diff < 0)
  
  stereo_start = 1;
  bm_start = -frame_diff+1;
  
end

end_diff = bm_stereo_cropped.frame_number(end) - stereo.frame_number(end);

if (end_diff > 0)
  
  bm_end = bm_end - end_diff;
  
else
  
  stereo_end = stereo_end + end_diff;
  
  
end

% ok everything is aligned now.
% now figure out where we want to look
bm_stereo_aligned.frame_number = bm_stereo_cropped.frame_number(bm_start:bm_end);
stereo_aligned.frame_number = stereo.frame_number(stereo_start:stereo_end);

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


bm_stereo_aligned.frame_number = bm_stereo_cropped.frame_number(bm_start:bm_end);
stereo_aligned.frame_number = stereo.frame_number(stereo_start:stereo_end);

bm_stereo_aligned.x = bm_stereo_cropped.x(bm_start:bm_end, :);
bm_stereo_aligned.y = bm_stereo_cropped.y(bm_start:bm_end, :);
bm_stereo_aligned.z = bm_stereo_cropped.z(bm_start:bm_end, :);

bm_stereo_aligned.utime = bm_stereo_cropped.utime(stereo_start:stereo_end);
bm_stereo_aligned.number_of_points = bm_stereo_cropped.number_of_points(stereo_start:stereo_end);
bm_stereo_aligned.video_number = bm_stereo_cropped.video_number(stereo_start:stereo_end);
bm_stereo_aligned.logtime = bm_stereo_cropped.logtime(stereo_start:stereo_end);





stereo_aligned.utime = stereo.utime(stereo_start:stereo_end);
stereo_aligned.number_of_points = stereo.number_of_points(stereo_start:stereo_end);
stereo_aligned.video_number = stereo.video_number(stereo_start:stereo_end);
stereo_aligned.logtime = stereo.logtime(stereo_start:stereo_end);

stereo_aligned.x = stereo.x(stereo_start:stereo_end, :);
stereo_aligned.y = stereo.y(stereo_start:stereo_end, :);
stereo_aligned.z = stereo.z(stereo_start:stereo_end, :);


bm_stereo_aligned.frame_number(1:5)
stereo_aligned.frame_number(1:5)

bm_stereo_aligned.frame_number(end-5:end)
stereo_aligned.frame_number(end-5:end)

%% process

%ComparePlot(bm_stereo_aligned, stereo_aligned);

distances = SmallestDistance(stereo_aligned.x, stereo_aligned.y, stereo_aligned.z, bm_stereo_aligned.x, bm_stereo_aligned.y, bm_stereo_aligned.z, 0, 100);
distances2 = SmallestDistance(bm_stereo_aligned.x, bm_stereo_aligned.y, bm_stereo_aligned.z, stereo_aligned.x, stereo_aligned.y, stereo_aligned.z, 0, 100);


real_dists = distances(find(distances));
real_dists2 = distances2(find(distances2));


%% plot
figure(1)
clf
hist(real_dists,0:.1:ceil(max(real_dists)));
xlabel('Minimum separation (meters)')
ylabel('Number of pixels')
title(strrep(filename, '_','-'));
xlim([-1 11]);
set(gca, 'XTickLabel',{'0','2','4','6','8','No Match'});
%ylim([0 500]);
grid on

if sum_false_pos == 1
  real_dists_array{pass_number} = real_dists;
end

figure(2)
clf
hist(real_dists2,0:.1:ceil(max(real_dists2)));
xlabel('Minimum separation (meters)')
ylabel('Number of pixels')
title(strrep(filename, '_','-'));
xlim([-1 11]);
set(gca, 'XTickLabel',{'0','2','4','6','8','No Match'});
%ylim([0 500]);
grid on

if sum_false_pos ~= 1
  real_dists_array{pass_number} = real_dists;
end



%% sum up

figure(sumfig)
real_sum = [];
for i = 1 : length(real_dists_array)
  real_sum = [real_sum; real_dists_array{i}];
end
hist(real_sum(:,1), 0:.1:ceil(max(real_sum(:,1))));
xlabel('Minimum separation (meters)')
ylabel('Number of pixels')

if doFalseNeg == 0
  title('')
else
  title('False Negative Check');
end

xlim([-1 11]);
set(gca, 'XTickLabel',{'0','2','4','6','8','No Match'});
%ylim([0 1500]);
grid on


%% compute statistics

% within 1 meter
one_meter = length(find(real_sum(:,1) < 1.0)) / length(real_sum(:,1));

disp([num2str(one_meter * 100) ' % within 1.0 meters'])


two_meters = length(find(real_sum(:,1) < 2.0)) / length(real_sum(:,1));

disp([num2str(two_meters * 100) ' % within 2.0 meters'])


five_meters = length(find(real_sum(:,1) >= 5.0)) / length(real_sum(:,1));

disp([num2str(five_meters * 100) ' % outside of 5.0 meters'])