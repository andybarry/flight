% load the data

dir = '2014-04-18-near-goalposts/mat/';
filename = 'pass1.mat';



dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

loadDeltawing



% get start and end times
[throttle_start, throttle_end] = FindActiveTimes(u.logtime, u.throttle, 1150);

% get altitude start and end times
[alt_start, alt_end] = FindActiveTimes(baro.logtime, baro.altitude, 7);

start_times = throttle_start;
end_times = throttle_end;


% build a vector that includes all time samples
time_all = sort([est.logtime; u.logtime]);

% get the average dt
id_dt = mean(diff(time_all));

% build a vector with uniform sample times

id_t = start_times(1):id_dt:end_times(1);


% build the state vector

id_y_raw = [ est.pos.x, est.pos.y, est.pos.z, ...
  est.orientation.yaw,  est.orientation.pitch, est.orientation.roll, ...
  est.vel.x, est.vel.y, est.vel.z, ...
  est.rotation_rate.z, est.rotation_rate.y, est.rotation_rate.x ];


% build the input vector

id_u_raw = [ u.throttle, u.elevonL, u.elevonR ];


% take first order holds on all of them

id_y_foh = foh(est.logtime, id_y_raw');
id_y = ppval(id_y_foh, id_t)';

id_u_foh = foh(u.logtime, id_u_raw');
id_u = ppval(id_u_foh, id_t)';


fit_data = iddata(id_y, id_u, id_dt);