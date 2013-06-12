
% load the log

dir = '2013-04-17-holodeck-bike/attempt1/';
filename = 'stereo_fix_u2_lcmlog_2013_04_17_02.mat';

filename2 = 'laptop_lcmlog_2013_04_17_00.mat';


load(strcat(dir, filename));
load(strcat(dir, filename2));

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

% stereo
stereoVals = stereo;
clear stereo
stereo.timestamp = stereoVals(:,1);
stereo.number_of_points = stereoVals(:,2);
stereo.frame_number = stereoVals(:,3);
stereo.x = stereoVals(:,4);
stereo.y = stereoVals(:,5);
stereo.z= stereoVals(:,6);

stereo.logtime = stereoVals(:,7);

% wind_groundspeed
wind_gpeed.utime = wind_groundspeed(:,1);
wind_gpeed.airspeed = wind_groundspeed(:,2);
wind_gpeed.estimatedGroundSpeed = wind_groundspeed(:,3);
wind_gpeed.wind_x = wind_groundspeed(:,4);
wind_gpeed.wind_y = wind_groundspeed(:,5);
wind_gpeed.wind_z = wind_groundspeed(:,6);

wind_gpeed.logtime = wind_groundspeed(:,7);

% baro_airspeed
baro.utime = baro_airspeed(:,1);
baro.airspeed = baro_airspeed(:,2);
baro.baro_altitude = baro_airspeed(:,3);
baro.temperature = baro_airspeed(:,4);
baro.logtime = baro_airspeed(:,5);

% trajectory number
trajnum.utime = trajectory_number(:,1);
trajnum.trajnum = trajectory_number(:,2);
trajnum.logtime = trajectory_number(:,3);


% optotrak
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
