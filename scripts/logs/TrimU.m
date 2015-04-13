function u = TrimU(start_time, end_time, u_in)

  u.utime = TrimTimes(start_time, end_time, u_in.logtime, u_in.utime);
  u.throttle = TrimTimes(start_time, end_time, u_in.logtime, u_in.throttle);
  u.elevonL = TrimTimes(start_time, end_time, u_in.logtime, u_in.elevonL);
  u.elevonR = TrimTimes(start_time, end_time, u_in.logtime, u_in.elevonR);
  u.is_autonomous = TrimTimes(start_time, end_time, u_in.logtime, u_in.is_autonomous);
  u.video_record = TrimTimes(start_time, end_time, u_in.logtime, u_in.video_record);
  
  u.rad.elevonL = TrimTimes(start_time, end_time, u_in.logtime, u_in.rad.elevonL);
  u.rad.elevonR = TrimTimes(start_time, end_time, u_in.logtime, u_in.rad.elevonR);
  u.rad.throttle = TrimTimes(start_time, end_time, u_in.logtime, u_in.rad.throttle);
  
  u.cmd.utime = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.cmd.utime);
  u.cmd.throttle = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.cmd.throttle);
  u.cmd.elevonL = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.cmd.elevonL);
  u.cmd.elevonR = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.cmd.elevonR);
  u.cmd.is_autonomous = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.cmd.is_autonomous);
  u.cmd.video_record = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.cmd.video_record);
  
  u.cmd.rad.elevonL = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.rad.elevonL);
  u.cmd.rad.elevonR = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.rad.elevonR);
  u.cmd.rad.throttle = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.rad.throttle);
  
  
  u.cmd.logtime = TrimTimes(start_time, end_time, u_in.cmd.logtime, u_in.cmd.logtime);
  
  u.logtime = TrimTimes(start_time, end_time, u_in.logtime, u_in.logtime);

end