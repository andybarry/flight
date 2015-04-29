
% load the log

%dir = 'sync/2013-10-23-delta-outside/';
%filename = 'lcmlog_1999_12_31_01.mat';

%filename2 = 'laptop_lcmlog_2013_04_17_00.mat';


clear STATE_ESTIMATOR_POSE battery_status stereoBmVals u
clear imu stereoReplayVals wind_groundspeed attitude est stereoVals 
clear wind_gspeed baro log stereo_bm wingeron_u airspeed sideslip gps
clear cpu_info_AAAZZZA cpu_info_odroid_gps1 cpu_info_odroid_cam1
clear cpu_info_odroid_gps2 cpu_info_odroid_cam2
clear log_info_odroid_gps1 log_info_odroid_cam1
clear log_info_odroid_gps2 log_info_odroid_cam2
clear log_info_AAAZZZA deltawing_u tvlqr_action
clear stereo_control
clear debug airspeed_unchecked altimeter
clear servo_out stereo_replay battery gpsValues stereo this_number
clear stereo_octomap stereoOctomapVals

load(strcat(dir, filename));
%load(strcat(dir, filename2));

if exist('cpu_info_odroid_gps1', 'var')
  cpu.gps.utime = cpu_info_odroid_gps1(:,1);
  cpu.gps.freq = cpu_info_odroid_gps1(:,2);
  cpu.gps.temp = cpu_info_odroid_gps1(:,3);
  cpu.gps.logtime = cpu_info_odroid_gps1(:,4);
  log.aircraft_number = 1;
  clear cpu_info_odroid_gps1
end

if exist('cpu_info_odroid_gps2', 'var')
  cpu.gps.utime = cpu_info_odroid_gps2(:,1);
  cpu.gps.freq = cpu_info_odroid_gps2(:,2);
  cpu.gps.temp = cpu_info_odroid_gps2(:,3);
  cpu.gps.logtime = cpu_info_odroid_gps2(:,4);
  log.aircraft_number = 2;
  clear cpu_info_odroid_gps2
end

if exist('cpu_info_odroid_cam1', 'var')
  cpu.cam.utime = cpu_info_odroid_cam1(:,1);
  cpu.cam.freq = cpu_info_odroid_cam1(:,2);
  cpu.cam.temp = cpu_info_odroid_cam1(:,3);
  cpu.cam.logtime = cpu_info_odroid_cam1(:,4);
  clear cpu_info_odroid_cam1
end

if exist('cpu_info_odroid_cam2', 'var')
  cpu.cam.utime = cpu_info_odroid_cam2(:,1);
  cpu.cam.freq = cpu_info_odroid_cam2(:,2);
  cpu.cam.temp = cpu_info_odroid_cam2(:,3);
  cpu.cam.logtime = cpu_info_odroid_cam2(:,4);
  clear cpu_info_odroid_cam2
end

% grab estimator values
if (exist('STATE_ESTIMATOR_POSE', 'var'))
  est.utime = STATE_ESTIMATOR_POSE(:,1);

  est.pos.x = STATE_ESTIMATOR_POSE(:,2);
  est.pos.y = STATE_ESTIMATOR_POSE(:,3);
  est.pos.z = STATE_ESTIMATOR_POSE(:,4);

  est.vel.x = STATE_ESTIMATOR_POSE(:,5);
  est.vel.y = STATE_ESTIMATOR_POSE(:,6);
  est.vel.z = STATE_ESTIMATOR_POSE(:,7);

  est.orientation.q0 = STATE_ESTIMATOR_POSE(:,8);
  est.orientation.q1 = STATE_ESTIMATOR_POSE(:,9);
  est.orientation.q2 = STATE_ESTIMATOR_POSE(:,10);
  est.orientation.q3 = STATE_ESTIMATOR_POSE(:,11);

  est.rotation_rate.x = STATE_ESTIMATOR_POSE(:,12);
  est.rotation_rate.y = STATE_ESTIMATOR_POSE(:,13);
  est.rotation_rate.z = STATE_ESTIMATOR_POSE(:,14);

  est.accel.x = STATE_ESTIMATOR_POSE(:,15);
  est.accel.y = STATE_ESTIMATOR_POSE(:,16);
  est.accel.z = STATE_ESTIMATOR_POSE(:,17);
  
  % get rpy
  this_rpy = quat2rpy_array(est.orientation.q0, est.orientation.q1, est.orientation.q2, est.orientation.q3);
  
  est.orientation.roll = this_rpy(:,1);
  est.orientation.pitch = this_rpy(:,2);
  est.orientation.yaw = this_rpy(:,3);
  
  est.est_frame = [est.pos.x, est.pos.y, est.pos.z, this_rpy, ...
    est.vel.x, est.vel.y, est.vel.z, ...
    est.rotation_rate.x, est.rotation_rate.y, est.rotation_rate.z];
  
  clear this_rpy;
  
  for i = 1 : size(est.est_frame, 1)
    est.drake_frame(i, :) = ConvertStateEstimatorToDrakeFrame(est.est_frame(i,:));
  end
  
  est.logtime = STATE_ESTIMATOR_POSE(:,18);
  
end
clear STATE_ESTIMATOR_POSE;

% battery values
battery.utime = battery_status(:,1);

battery.voltage = battery_status(:,2);
battery.amps_now = battery_status(:,3);
battery.milliamp_hours_total = battery_status(:,4);
battery.percent_remaining = battery_status(:,5);

battery.logtime = battery_status(:,6);
clear battery_status;


% imu values
imu.utime = attitude(:,1);
imu.device_time = attitude(:,2);
imu.gyro.x = attitude(:,3);
imu.gyro.y = attitude(:,4);
imu.gyro.z = attitude(:,5);

imu.mag.x = attitude(:,6);
imu.mag.y = attitude(:,7);
imu.mag.z = attitude(:,8);

imu.accel.x = attitude(:,9);
imu.accel.y = attitude(:,10);
imu.accel.z = attitude(:,11);

imu.logtime = attitude(:,18);
clear attitude;

%gps
gpsValues = gps;
clear gps;
gps.utime = gpsValues(:,1);
gps.gps_lock = gpsValues(:,2);
gps.longitude = gpsValues(:,3);
gps.latitude = gpsValues(:,4);
gps.elev = gpsValues(:,5);
gps.horizontal_accuracy = gpsValues(:,6);
gps.vertical_accuracy = gpsValues(:,7);
gps.num_staellites = gpsValues(:,8);
gps.speed = gpsValues(:,9);
gps.heading = gpsValues(:,10);
gps.x = gpsValues(:,11);
gps.y = gpsValues(:,12);
gps.z = gpsValues(:,13);
gps.gps_time = gpsValues(:,14);
gps.logtime = gpsValues(:,15);
clear gpsValues


% read servo configuration values
[rad_to_servo, servo_to_rad, servo_minmaxtrim] = ReadSimpleConfigServos(['/home/abarry/realtime/config/plane-odroid-gps' num2str(log.aircraft_number) '.cfg']);

% #servo_out  <class 'lcmt_deltawing_u.lcmt_deltawing_u'> :
% #[
% #1- timestamp
% #2- elevonL
% #3- elevonR
% #4- throttle
% #5- is_autonomous
% #6- video_record
% #7- log_timestamp
% #]


u.utime = servo_out(:,1);
u.elevonL = servo_out(:,2);
u.elevonR = servo_out(:,3);
u.throttle = servo_out(:,4);
u.is_autonomous = servo_out(:,5);
u.video_record = servo_out(:,6);
u.logtime = servo_out(:,7);

u.trim.zero.elevonL = servo_minmaxtrim.elevL_trim;
u.trim.zero.elevonR = servo_minmaxtrim.elevR_trim;

u.trim.flight.elevonL = servo_minmaxtrim.elevL_flight_trim;
u.trim.flight.elevonR = servo_minmaxtrim.elevR_flight_trim;

% throttle can exceed the top value, but don't do that since that confuses
% scripts down the line. Just max it out at the top value
u.throttle = min(u.throttle, servo_minmaxtrim.throttle_max);

u.rad.elevonL = servo_to_rad.elevL_slope .* u.elevonL + servo_to_rad.elevL_y_intercept;
u.rad.elevonR = servo_to_rad.elevR_slope .* u.elevonR + servo_to_rad.elevR_y_intercept;
u.rad.throttle = servo_to_rad.throttle_slope .* u.throttle + servo_to_rad.throttle_y_intercept;

u.rad.trim.zero.elevonL = servo_to_rad.elevL_slope .* u.trim.zero.elevonL + servo_to_rad.elevL_y_intercept;
u.rad.trim.zero.elevonR = servo_to_rad.elevR_slope .* u.trim.zero.elevonR + servo_to_rad.elevR_y_intercept;


u.rad.trim.flight.elevonL = servo_to_rad.elevL_slope .* u.trim.flight.elevonL + servo_to_rad.elevL_y_intercept;
u.rad.trim.flight.elevonR = servo_to_rad.elevR_slope .* u.trim.flight.elevonR + servo_to_rad.elevR_y_intercept;

clear servo_out

u.cmd.utime = deltawing_u(:,1);
u.cmd.elevonL = deltawing_u(:,2);
u.cmd.elevonR = deltawing_u(:,3);
u.cmd.throttle = deltawing_u(:,4);
u.cmd.is_autonomous = deltawing_u(:,5);
u.cmd.video_record = deltawing_u(:,6);

% ensure commands never exceed allowed values
u.cmd.elevonL = max(u.cmd.elevonL, servo_minmaxtrim.elevL_min);
u.cmd.elevonL = min(u.cmd.elevonL, servo_minmaxtrim.elevL_max);

u.cmd.elevonR = max(u.cmd.elevonR, servo_minmaxtrim.elevR_min);
u.cmd.elevonR = min(u.cmd.elevonR, servo_minmaxtrim.elevR_max);

u.cmd.throttle = max(u.cmd.throttle, servo_minmaxtrim.throttle_min);
u.cmd.throttle = min(u.cmd.throttle, servo_minmaxtrim.throttle_max);

u.cmd.rad.elevonL = servo_to_rad.elevL_slope .* u.cmd.elevonL + servo_to_rad.elevL_y_intercept;
u.cmd.rad.elevonR = servo_to_rad.elevR_slope .* u.cmd.elevonR + servo_to_rad.elevR_y_intercept;
u.cmd.rad.throttle = servo_to_rad.throttle_slope .* u.cmd.throttle + servo_to_rad.throttle_y_intercept;

u.cmd.rad.throttle = max(u.cmd.rad.throttle, 0);

u.cmd.logtime = deltawing_u(:,7);
clear deltawing_u




% stereo
% #stereo  <class 'lcmt_stereo.lcmt_stereo'> :
% #[
% #1- timestamp
% #2- number_of_points
% #3- frame_number
% #4- video_number
% #5- x(0)
% #5- y(0)
% #5- z(0)
% #5- log_timestamp
% #]
if (exist('stereo', 'var'))
  stereoVals = stereo;
  clear stereo

  stereo = LoadStereo(stereoVals);
  
end


if (exist('stereo_bm', 'var'))
  stereoBmVals = stereo_bm;
  clear stereo_bm

  stereo_bm = LoadStereo(stereoBmVals);
  
end


if (exist('stereo_replay', 'var'))
  stereoReplayVals = stereo_replay;
  clear stereo_repaly

  stereo_replay = LoadStereo(stereoReplayVals);
  
end

if (exist('stereo_octomap', 'var'))
  stereoOctomapVals = stereo_octomap;
  clear stereo_octomap

  stereo_octomap = LoadStereoWithXY(stereoOctomapVals);
  
end

%{

if (exist('stereo_bm'))
  stereoBmVals = stereo_bm;
  clear stereo_bm

  stereo_bm.utime = stereoBmVals(:,1);
  stereo_bm.number_of_points = stereoBmVals(:,2);
  stereo_bm.frame_number = stereoBmVals(:,3);
  stereo_bm.video_number = stereoBmVals(:,4);

  if (size(stereoBmVals, 2) < 6)
    % if we got no stereo hits, the array won't be initialized
    stereo_bm.x = [];
    stereo_bm.y = [];
    stereo_bm.z = [];
    stereo_bm.logtime = stereoBmVals(:,5);
  else
    stereo_bm.x = []; % TODO
    stereo_bm.y = []; % TODO
    stereo_bm.z = []; % TODO
    stereo_bm.logtime = stereoBmVals(:,5);
  end
end

if (exist('stereo_replay'))
  stereoReplayVals = stereo_replay;
  clear stereo_replay

  stereo_replay.utime = stereoReplayVals(:,1);
  stereo_replay.number_of_points = stereoReplayVals(:,2);
  stereo_replay.frame_number = stereoReplayVals(:,3);
  stereo_replay.video_number = stereoReplayVals(:,4);

  if (size(stereoReplayVals, 2) < 6)
    % if we got no stereo hits, the array won't be initialized
    stereo_replay.x = [];
    stereo_replay.y = [];
    stereo_replay.z = [];
    stereo_replay.logtime = stereoReplayVals(:,5);
  else
    stereo_replay.x = []; % TODO
    stereo_replay.y = []; % TODO
    stereo_replay.z = []; % TODO
    stereo_replay.logtime = stereoReplayVals(:,5);
  end
end

%}

sideslipValues = sideslip;
clear sideslip;
sideslip.utime = sideslipValues(:,1);
sideslip.sideslip = sideslipValues(:,4);
sideslip.logtime = sideslipValues(:,8);
clear sideslipValues


airspeedValues = airspeed;
clear airspeed;
airspeed.utime = airspeedValues(:,1);
airspeed.airspeed = airspeedValues(:,4);
airspeed.logtime = airspeedValues(:,8);
clear airspeedValues

airspeed_uncheckedValues = airspeed_unchecked;
clear airspeed_unchecked;
airspeed_unchecked.utime = airspeed_uncheckedValues(:,1);
airspeed_unchecked.airspeed = airspeed_uncheckedValues(:,4);
airspeed_unchecked.logtime = airspeed_uncheckedValues(:,8);
clear airspeed_uncheckedValues

%{
#altimeter  <class 'indexed_measurement_t.indexed_measurement_t'> :
#[
#1- utime
#2- state_utime
#3- measured_dim
#4- z_effective(1)
#5- z_indices(1)
#6- measured_cov_dim
#7- R_effective(1)
#8- log_timestamp
#]
%}

altimeterValues = altimeter;
clear altimeter
altimeter.utime = altimeterValues(:,1);
altimeter.altitude = altimeterValues(:,4);
altimeter.logtime = altimeterValues(:,8);
clear altimeterValues

% trajectory number
if (exist('trajnum', 'var'))
  trajnum.utime = trajectory_number(:,1);
  trajnum.trajnum = trajectory_number(:,2);
  trajnum.logtime = trajectory_number(:,3);
  clear trajectory_number
end

%{
#stereo-control  <class 'lcmt_stereo_control.lcmt_stereo_control'> :
#[
#1- timestamp
#2- stereo_control
#3- log_timestamp
#]
%}

if exist('stereo_control', 'var')
  stereo_controlValues = stereo_control;
  clear stereo_control
  stereo_control.utime = stereo_controlValues(:,1);
  stereo_control.stereo_control = stereo_controlValues(:,2);
  stereo_control.logtime = stereo_controlValues(:,3);
  
  clear stereo_controlValues;
end

%{
#tvlqr-action  <class 'lcmt_tvlqr_controller_action.lcmt_tvlqr_controller_action'> :
#[
#1- timestamp
#2- trajectory_number
#3- log_timestamp
#]
%}

if exist('tvlqr_action', 'var')
  tvlqr.utime = tvlqr_action(:,1);
  tvlqr.trajectory_number = tvlqr_action(:,2);
  tvlqr.logtime = tvlqr_action(:,3);
  clear tvlqr_action
end





% optotrak
if (exist('wingeron_x_quat', 'var'))
  optotrak.utime = wingeron_x_quat(:,1);
  optotrak.sec = (optotrak.utime - optotrak.utime(1)) / 1000;
  optotrak.number_of_rigid_bodies = wingeron_x_quat(:,2);

  optotrak.x = wingeron_x_quat(:,3);
  optotrak.y = wingeron_x_quat(:,4);
  optotrak.z = wingeron_x_quat(:,5);

  optotrak.q0 = wingeron_x_quat(:,6);
  optotrak.qx = wingeron_x_quat(:,7);
  optotrak.qy = wingeron_x_quat(:,8);
  optotrak.qz = wingeron_x_quat(:,9);

  optotrak.logtime = wingeron_x_quat(:,10);
  clear wingeron_x_quat

end

if exist('cpu_info_AAAZZZA', 'var')
  cpu.laptop.utime = cpu_info_AAAZZZA(:,1);
  cpu.laptop.freq = cpu_info_AAAZZZA(:,2);
  cpu.laptop.temp = cpu_info_AAAZZZA(:,3);
  cpu.laptop.logtime = cpu_info_AAAZZZA(:,4);
  clear cpu_info_AAAZZZA
end



%{
#log-info-AAAZZZA  <class 'lcmt_log_size.lcmt_log_size'> :
#[
#1- timestamp
#2- log_number
#3- log_size
#4- disk_space_free
#5- log_timestamp
#]
%}

if exist('log_info_odroid_gps2', 'var')
  log.info.gps.utime = log_info_odroid_gps2(:,1);
  log.info.gps.log_number = log_info_odroid_gps2(:,2);
  log.info.gps.log_size = log_info_odroid_gps2(:,3);
  log.info.gps.disk_free = log_info_odroid_gps2(:,4);
  log.info.gps.logtime = log_info_odroid_gps2(:,5);
  clear log_info_odroid_gps2
end

if exist('log_info_odroid_gps1', 'var')
  log.info.gps.utime = log_info_odroid_gps1(:,1);
  log.info.gps.log_number = log_info_odroid_gps1(:,2);
  log.info.gps.log_size = log_info_odroid_gps1(:,3);
  log.info.gps.disk_free = log_info_odroid_gps1(:,4);
  log.info.gps.logtime = log_info_odroid_gps1(:,5);
  clear log_info_odroid_gps1
end

if exist('log_info_odroid_cam1', 'var')
  log.info.cam.utime = log_info_odroid_cam1(:,1);
  log.info.cam.log_number = log_info_odroid_cam1(:,2);
  log.info.cam.log_size = log_info_odroid_cam1(:,3);
  log.info.cam.disk_free = log_info_odroid_cam1(:,4);
  log.info.cam.logtime = log_info_odroid_cam1(:,5);
  clear log_info_odroid_cam1
end

if exist('log_info_odroid_cam2', 'var')
  log.info.cam.utime = log_info_odroid_cam2(:,1);
  log.info.cam.log_number = log_info_odroid_cam2(:,2);
  log.info.cam.log_size = log_info_odroid_cam2(:,3);
  log.info.cam.disk_free = log_info_odroid_cam2(:,4);
  log.info.cam.logtime = log_info_odroid_cam2(:,5);
  clear log_info_odroid_cam2
end

if exist('log_info_AAAZZZA', 'var')
  log.info.laptop.utime = log_info_AAAZZZA(:,1);
  log.info.laptop.log_number = log_info_AAAZZZA(:,2);
  log.info.laptop.log_size = log_info_AAAZZZA(:,3);
  log.info.laptop.disk_free = log_info_AAAZZZA(:,4);
  log.info.laptop.logtime = log_info_AAAZZZA(:,5);
  clear log_info_AAAZZZA
end



if exist('debug', 'var')
  debugValues = debug;
  clear debug;
  debug.utime = debugValues(:,1);
  debug.logtime = debugValues(:,2);
  
  clear debugValues
end

% get the run number
log.number = filename(end-5:end-4);

mypath = strsplit([dir filename], '/');
log.name = mypath{end};
clear mypath;



%{

#cpu-info-AAAZZZA  <class 'lcmt_cpu_info.lcmt_cpu_info'> :
#[
#1- timestamp
#2- cpu_freq
#3- cpu_temp
#4- log_timestamp
#]

#log-info-AAAZZZA  <class 'lcmt_log_size.lcmt_log_size'> :
#[
#1- timestamp
#2- log_number
#3- log_size
#4- disk_space_free
#5- log_timestamp
#]

#cpu-info-odroid-cam2  <class 'lcmt_cpu_info.lcmt_cpu_info'> :
#[
#1- timestamp
#2- cpu_freq
#3- cpu_temp
#4- log_timestamp
#]

#log-info-odroid-cam2  <class 'lcmt_log_size.lcmt_log_size'> :
#[
#1- timestamp
#2- log_number
#3- log_size
#4- disk_space_free
#5- log_timestamp
#]

#cpu-info-odroid-gps2  <class 'lcmt_cpu_info.lcmt_cpu_info'> :
#[
#1- timestamp
#2- cpu_freq
#3- cpu_temp
#4- log_timestamp
#]

#log-info-odroid-gps2  <class 'lcmt_log_size.lcmt_log_size'> :
#[
#1- timestamp
#2- log_number
#3- log_size
#4- disk_space_free
#5- log_timestamp
#]

#gps  <class 'gps_data_t.gps_data_t'> :
#[
#1- utime
#2- gps_lock
#3- longitude
#4- latitude
#5- elev
#6- horizontal_accuracy
#7- vertical_accuracy
#8- numSatellites
#9- speed
#10- heading
#11- xyz_pos(3)
#14- gps_time
#15- log_timestamp
#]

#battery-status  <class 'lcmt_battery_status.lcmt_battery_status'> :
#[
#1- timestamp
#2- voltage
#3- amps_now
#4- milliamp_hours_total
#5- percent_remaining
#6- log_timestamp
#]

#altimeter  <class 'indexed_measurement_t.indexed_measurement_t'> :
#[
#1- utime
#2- state_utime
#3- measured_dim
#4- z_effective(1)
#5- z_indices(1)
#6- measured_cov_dim
#7- R_effective(1)
#8- log_timestamp
#]

#airspeed-unchecked  <class 'indexed_measurement_t.indexed_measurement_t'> :
#[
#1- utime
#2- state_utime
#3- measured_dim
#4- z_effective(1)
#5- z_indices(1)
#6- measured_cov_dim
#7- R_effective(1)
#8- log_timestamp
#]

#sideslip  <class 'indexed_measurement_t.indexed_measurement_t'> :
#[
#1- utime
#2- state_utime
#3- measured_dim
#4- z_effective(1)
#5- z_indices(1)
#6- measured_cov_dim
#7- R_effective(1)
#8- log_timestamp
#]

#airspeed  <class 'indexed_measurement_t.indexed_measurement_t'> :
#[
#1- utime
#2- state_utime
#3- measured_dim
#4- z_effective(1)
#5- z_indices(1)
#6- measured_cov_dim
#7- R_effective(1)
#8- log_timestamp
#]

#debug  <class 'lcmt_debug.lcmt_debug'> :
#[
#1- utime
#2- log_timestamp
#]

#servo_out  <class 'lcmt_deltawing_u.lcmt_deltawing_u'> :
#[
#1- timestamp
#2- elevonL
#3- elevonR
#4- throttle
#5- is_autonomous
#6- video_record
#7- log_timestamp
#]

#attitude  <class 'ins_t.ins_t'> :
#[
#1- utime
#2- device_time
#3- gyro(3)
#6- mag(3)
#9- accel(3)
#12- quat(4)
#16- pressure
#17- rel_alt
#18- log_timestamp
#]

#STATE_ESTIMATOR_POSE  <class 'pose_t.pose_t'> :
#[
#1- utime
#2- pos(3)
#5- vel(3)
#8- orientation(4)
#12- rotation_rate(3)
#15- accel(3)
#18- log_timestamp
#]

#deltawing_u  <class 'lcmt_deltawing_u.lcmt_deltawing_u'> :
#[
#1- timestamp
#2- elevonL
#3- elevonR
#4- throttle
#5- is_autonomous
#6- video_record
#7- log_timestamp
#]
                                
#stereo-control  <class 'lcmt_stereo_control.lcmt_stereo_control'> :
#[
#1- timestamp
#2- stereo_control
#3- log_timestamp
#]

#tvlqr-action  <class 'lcmt_tvlqr_controller_action.lcmt_tvlqr_controller_action'> :
#[
#1- timestamp
#2- trajectory_number
#3- log_timestamp
#]

%}