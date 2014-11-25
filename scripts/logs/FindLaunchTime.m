function t_start = FindLaunchTime(imu)
  % Finds first spike in x-accleration
  %
  % @param imu imu structure from logs
  %
  % @retval time (from logtime) that launch is at
  
  
  g = 9.81;
  
  launch_g = 7*g;

  
  if max(imu.accel.x) < launch_g
    error('Launch of > 7 Gs not detected.');
  end
  
  idx = find(imu.accel.x > launch_g);
  
  % take the first
  
  t_start = imu.logtime(idx(1));
    

end