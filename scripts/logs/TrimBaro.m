function baro = TrimBaro(start_time, end_time, baro_in)

  baro.utime = TrimTimes(start_time, end_time, baro_in.logtime, baro_in.utime);
  baro.airspeed = TrimTimes(start_time, end_time, baro_in.logtime, baro_in.airspeed);
  
  baro.altitude= TrimTimes(start_time, end_time, baro_in.logtime, baro_in.altitude);
  baro.temperature = TrimTimes(start_time, end_time, baro_in.logtime, baro_in.temperature);
  baro.baro_altitude = TrimTimes(start_time, end_time, baro_in.logtime, baro_in.baro_altitude);

  
  baro.logtime = TrimTimes(start_time, end_time, baro_in.logtime, baro_in.logtime);

end