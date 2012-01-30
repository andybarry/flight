% generate a trajectory to use for sysid


zoh_t = [0, .25, .5, 100];

%{
zoh_array = [0, 0, -255, 0     % throttle
             0, 0, 0, 0     % rudder
             0, 0, 0, 0     % elevator
             0, 25, -25, 0   % aileron left
             0, 25, -25, 0   % aileron right
             ];

%}         
zoh_array = [0, 0, -255, 0     % throttle
             0, 0, 0, 0     % rudder
             0, 0, 0, 0     % elevator
             0, 45, -45, 0   % aileron left
             0, 45, -45, 0   % aileron right
             ];


pp = zoh(zoh_t, zoh_array);

pptraj = PPTrajectory(pp);