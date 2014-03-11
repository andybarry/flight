
% load the log

%dir = 'sync/2013-10-23-delta-outside/';
%filename = 'lcmlog_1999_12_31_01.mat';

%filename2 = 'laptop_lcmlog_2013_04_17_00.mat';


load(strcat(dir, filename));
%load(strcat(dir, filename2));

% grab estimator values
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

est.logtime = STATE_ESTIMATOR_POSE(:,18);

% battery values
battery.utime = battery_status(:,1);

battery.voltage = battery_status(:,2);
battery.amps_now = battery_status(:,3);
battery.milliamp_hours_total = battery_status(:,4);
battery.percent_remaining = battery_status(:,5);

battery.logtime = battery_status(:,6);

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

baro.utime = baro_airspeed(:,1);
baro.airspeed = baro_airspeed(:,2);
baro.altitude = baro_airspeed(:,3);
baro.temperature = baro_airspeed(:,4);
baro.logtime = baro_airspeed(:,5);

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


% 
% stereo
% if (exist('stereo'))
%   stereoVals = stereo;
%   clear stereo
% 
%   stereo.timestamp = stereoVals(:,1);
%   stereo.number_of_points = stereoVals(:,2);
%   stereo.frame_number = stereoVals(:,3);
% 
%   if (size(stereo, 2) < 6)
%     if we got no stereo hits, the array won't be initialized
%     stereo.x = [];
%     stereo.y = [];
%     stereo.z = [];
%     stereo.logtime = stereoVals(:,4);
%   else
%     stereo.x = stereoVals(:,4);
%     stereo.y = stereoVals(:,5);
%     stereo.z = stereoVals(:,6);
%     stereo.logtime = stereoVals(:,7);
%   end
% end
%  



% wind_groundspeed

if (exist('wind_groundspeed'))
  wind_gspeed.utime = wind_groundspeed(:,1);
  wind_gspeed.airspeed = wind_groundspeed(:,2);
  wind_gspeed.estimated_ground_speed = wind_groundspeed(:,3);
  wind_gspeed.wind_x = wind_groundspeed(:,4);
  wind_gspeed.wind_y = wind_groundspeed(:,5);
  wind_gspeed.wind_z = wind_groundspeed(:,6);

  wind_gspeed.logtime = wind_groundspeed(:,7);
end


% baro_airspeed
baro.utime = baro_airspeed(:,1);
baro.airspeed = baro_airspeed(:,2);
baro.baro_altitude = baro_airspeed(:,3);
baro.temperature = baro_airspeed(:,4);
baro.logtime = baro_airspeed(:,5);

% trajectory number
if (exist('trajnum'))
  trajnum.utime = trajectory_number(:,1);
  trajnum.trajnum = trajectory_number(:,2);
  trajnum.logtime = trajectory_number(:,3);
end

% optotrak
if (exist('wingeron_x_quat'))
  optotrak.timestamp = wingeron_x_quat(:,1);
  optotrak.sec = (optotrak.timestamp - optotrak.timestamp(1)) / 1000;
  optotrak.number_of_rigid_bodies = wingeron_x_quat(:,2);

  optotrak.x = wingeron_x_quat(:,3);
  optotrak.y = wingeron_x_quat(:,4);
  optotrak.z = wingeron_x_quat(:,5);

  optotrak.q0 = wingeron_x_quat(:,6);
  optotrak.qx = wingeron_x_quat(:,7);
  optotrak.qy = wingeron_x_quat(:,8);
  optotrak.qz = wingeron_x_quat(:,9);

  optotrak.logtime = wingeron_x_quat(:,10);

end