function wind_gspeed = TrimWindGspeed(start_time, end_time, wind_gspeed_in)

  wind_gspeed.utime = TrimTimes(start_time, end_time, wind_gspeed_in.logtime, wind_gspeed_in.utime);
  wind_gspeed.airspeed = TrimTimes(start_time, end_time, wind_gspeed_in.logtime, wind_gspeed_in.airspeed);
  
  wind_gspeed.estimated_ground_speed = TrimTimes(start_time, end_time, wind_gspeed_in.logtime, wind_gspeed_in.estimated_ground_speed);
  
  wind_gspeed.wind_x = TrimTimes(start_time, end_time, wind_gspeed_in.logtime, wind_gspeed_in.wind_x);
  wind_gspeed.wind_y = TrimTimes(start_time, end_time, wind_gspeed_in.logtime, wind_gspeed_in.wind_y);
  wind_gspeed.wind_z = TrimTimes(start_time, end_time, wind_gspeed_in.logtime, wind_gspeed_in.wind_z);

  
  wind_gspeed.logtime = TrimTimes(start_time, end_time, wind_gspeed_in.logtime, wind_gspeed_in.logtime);

end