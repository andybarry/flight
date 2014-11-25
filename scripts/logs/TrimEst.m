function est = TrimEst(start_time, end_time, est_in)

  est.utime = TrimTimes(start_time, end_time, est_in.logtime, est_in.utime);
  
  est.pos.x = TrimTimes(start_time, end_time, est_in.logtime, est_in.pos.x);
  est.pos.y = TrimTimes(start_time, end_time, est_in.logtime, est_in.pos.y);
  est.pos.z = TrimTimes(start_time, end_time, est_in.logtime, est_in.pos.z);
  
  est.vel.x = TrimTimes(start_time, end_time, est_in.logtime, est_in.vel.x);
  est.vel.y = TrimTimes(start_time, end_time, est_in.logtime, est_in.vel.y);
  est.vel.z = TrimTimes(start_time, end_time, est_in.logtime, est_in.vel.z);
  
  est.orientation.q0 = TrimTimes(start_time, end_time, est_in.logtime, est_in.orientation.q0);
  est.orientation.q1 = TrimTimes(start_time, end_time, est_in.logtime, est_in.orientation.q1);
  est.orientation.q2 = TrimTimes(start_time, end_time, est_in.logtime, est_in.orientation.q2);
  est.orientation.q3 = TrimTimes(start_time, end_time, est_in.logtime, est_in.orientation.q3);
  
  est.orientation.yaw = TrimTimes(start_time, end_time, est_in.logtime, est_in.orientation.yaw);
  est.orientation.pitch = TrimTimes(start_time, end_time, est_in.logtime, est_in.orientation.pitch);
  est.orientation.roll = TrimTimes(start_time, end_time, est_in.logtime, est_in.orientation.roll);
  
  est.rotation_rate.x = TrimTimes(start_time, end_time, est_in.logtime, est_in.rotation_rate.x);
  est.rotation_rate.y = TrimTimes(start_time, end_time, est_in.logtime, est_in.rotation_rate.y);
  est.rotation_rate.z = TrimTimes(start_time, end_time, est_in.logtime, est_in.rotation_rate.z);
  
  est.accel.x = TrimTimes(start_time, end_time, est_in.logtime, est_in.accel.x);
  est.accel.y = TrimTimes(start_time, end_time, est_in.logtime, est_in.accel.y);
  est.accel.z = TrimTimes(start_time, end_time, est_in.logtime, est_in.accel.z);
  
  est.logtime = TrimTimes(start_time, end_time, est_in.logtime, est_in.logtime);

end