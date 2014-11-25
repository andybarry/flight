function imu = TrimIMU(start_time, end_time, imu_in)

  imu.utime = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.utime);
  imu.device_time = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.device_time);
  
  imu.gyro.x = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.gyro.x);
  imu.gyro.y = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.gyro.y);
  imu.gyro.z = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.gyro.z);
  
  imu.mag.x = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.mag.x);
  imu.mag.y = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.mag.y);
  imu.mag.z = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.mag.z);
  
  imu.accel.x = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.accel.x);
  imu.accel.y = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.accel.y);
  imu.accel.z = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.accel.z);
  
  imu.logtime = TrimTimes(start_time, end_time, imu_in.logtime, imu_in.logtime);

end