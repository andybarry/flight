% generate a trajectory to use for sysid


zoh_t = [0, 2, 3, 100];

zoh_array = [0, 0, 0, 0     % throttle
             0, 0, 0, 0     % rudder
             0, 0, 0, 0     % elevator
             0, 45, -45, 0   % aileron left
             0, 45, -45, 0   % aileron right
             ];


pp = zoh(zoh_t, zoh_array);

pptraj = PPTrajectory(pp);