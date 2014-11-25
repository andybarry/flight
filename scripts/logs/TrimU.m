function u = TrimU(start_time, end_time, u_in)

  u.utime = TrimTimes(start_time, end_time, u_in.logtime, u_in.utime);
  u.throttle = TrimTimes(start_time, end_time, u_in.logtime, u_in.throttle);
  u.elevonL = TrimTimes(start_time, end_time, u_in.logtime, u_in.elevonL);
  u.elevonR = TrimTimes(start_time, end_time, u_in.logtime, u_in.elevonR);
  u.is_autonomous = TrimTimes(start_time, end_time, u_in.logtime, u_in.is_autonomous);
  u.video_record = TrimTimes(start_time, end_time, u_in.logtime, u_in.video_record);
  u.elevonL_command = TrimTimes(start_time, end_time, u_in.logtime, u_in.elevonL_command);
  u.elevonR_command = TrimTimes(start_time, end_time, u_in.logtime, u_in.elevonR_command);
  u.throttle_command = TrimTimes(start_time, end_time, u_in.logtime, u_in.throttle_command);
  u.logtime = TrimTimes(start_time, end_time, u_in.logtime, u_in.logtime);

end