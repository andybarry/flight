function airspeed = TrimAirspeed(start_time, end_time, airspeed_in)

  airspeed.utime = TrimTimes(start_time, end_time, airspeed_in.logtime, airspeed_in.utime);
  airspeed.airspeed = TrimTimes(start_time, end_time, airspeed_in.logtime, airspeed_in.airspeed);
  
  airspeed.logtime = TrimTimes(start_time, end_time, airspeed_in.logtime, airspeed_in.logtime);

end