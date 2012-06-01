%tstart = 00;
%tend = 120;

t = wingeron_x(:,9);

% discover stop of throttle
t_u = wingeron_u(:,8);
throttle = wingeron_u(:,2);

% find all the points where the throttle drops to zero
dthrottle = diff(throttle);
ind = find(dthrottle < 0);
tend = t_u(ind(end)) + 1;
tstart = tend - 2;

x = wingeron_x(:,3);
y = wingeron_x(:,4);
z = wingeron_x(:,5);

yaw = wingeron_x(:,8); %%% ROLL AND YAW SWITCH FROM OPTOTRAK
pitch = wingeron_x(:,7);
roll = wingeron_x(:,6);

figure(1)
clf
plot(t,x,'bx-')
hold on
plot(t,y,'gx');
plot(t,z,'rx');
legend('x','y','z');
xlabel('Time (s)');
ylabel('Position (mm)');
title('wingeron x positions');
ylim([-5000 5000]);

xlim([tstart tend]);



figure(2)
clf
plot(t, yaw.*180/pi, 'bx')
hold on
plot(t, pitch.*180/pi,'gx-');
plot(t, roll.*180/pi,'rx');

legend('yaw','pitch','roll');
ylim([-200 200]);
xlabel('Time (s)');
ylabel('Angle (deg)');
title('wingeron x angles');


xlim([tstart tend]);

figure(3)
clf

ailR = wingeron_u(:,7);
plot(t_u, throttle);
hold on
plot(t_u, ailR, 'xr')
legend('Thottle','Aileron Right');
title('wingeron u');
xlim([tstart tend]);
