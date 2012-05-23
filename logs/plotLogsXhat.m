tstart = 00;
tend = 100;

t = wingeron_xhat(:,14);

x = wingeron_xhat(:,5);
y = wingeron_xhat(:,6);
z = wingeron_xhat(:,7);

roll = wingeron_xhat(:,2);
pitch = wingeron_xhat(:,3);
yaw = wingeron_xhat(:,4);

figure(1)
clf
plot(t,x,'bx-')
hold on
plot(t,y,'gx-');
plot(t,z,'rx-');
legend('x','y','z');
xlabel('Time (s)');
ylabel('Position (m)');
title('wingeron xhat positions');
ylim([-5 5]);

xlim([tstart tend]);

%return;

figure(2)
clf
plot(t, yaw.*180/pi, 'bx-')
hold on
plot(t, pitch.*180/pi,'gx-');
plot(t, roll.*180/pi,'rx-');

legend('yaw','pitch','roll');
ylim([-200 200]);
xlabel('Time (s)');
ylabel('Angle (deg)');
title('wingeron xhat angles');


xlim([tstart tend]);

figure(3)
clf
throttle = wingeron_u(:,2);
ailR = wingeron_u(:,7);
t_u = wingeron_u(:,8);
plot(t_u, throttle);
hold on
plot(t_u, ailR, 'xr-')
legend('Thottle','Aileron Right');
title('wingeron u');
