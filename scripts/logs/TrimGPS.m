function gps = TrimGPS(start_time, end_time, gps_in)

  gps.utime = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.utime);
  
  gps.gps_lock = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.gps_lock);
  
  gps.longitude = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.longitude);
  gps.latitude = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.latitude);
  gps.elev = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.elev);
  
  gps.horizontal_accuracy = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.horizontal_accuracy);
  gps.vertical_accuracy = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.vertical_accuracy);
  gps.num_staellites = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.num_staellites);

  gps.speed = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.speed);
  gps.heading = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.heading);
  gps.x = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.x);  
  gps.y = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.y);
  gps.z = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.z);
  
  
  
  gps.gps_time = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.gps_time);
  
  gps.logtime = TrimTimes(start_time, end_time, gps_in.logtime, gps_in.logtime);

end