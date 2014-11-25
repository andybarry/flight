function battery = TrimBattery(start_time, end_time, battery_in)

  battery.utime = TrimTimes(start_time, end_time, battery_in.logtime, battery_in.utime);
  battery.voltage = TrimTimes(start_time, end_time, battery_in.logtime, battery_in.voltage);
  
  battery.amps_now = TrimTimes(start_time, end_time, battery_in.logtime, battery_in.amps_now);
  battery.milliamp_hours_total = TrimTimes(start_time, end_time, battery_in.logtime, battery_in.milliamp_hours_total);
  battery.percent_remaining = TrimTimes(start_time, end_time, battery_in.logtime, battery_in.percent_remaining);

  
  battery.logtime = TrimTimes(start_time, end_time, battery_in.logtime, battery_in.logtime);

end