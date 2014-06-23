%tstart = 00;
%tend = 100;

t = wingeron_xhat(:,14);

% discover stop of throttle
t_u = wingeron_u(:,8);
throttle = wingeron_u(:,2);

% find all the points where the throttle drops to zero
dthrottle = diff(throttle);
ind = find(dthrottle < 0);
if (isempty(ind))
    tend = 2;
else
    tend = t_u(ind(end)) + 1;
end
tstart = tend - 2;
[~, tcrossind] = min(abs(x - 1000));

tcross = t(tcrossind);

x = wingeron_xhat(:,5);
y = wingeron_xhat(:,6);
z = wingeron_xhat(:,7);

roll = wingeron_xhat(:,2);
pitch = wingeron_xhat(:,3);
yaw = wingeron_xhat(:,4);

thisFig = figure(4);
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
set(thisFig, 'Position', [20 1 560 420])

%return;

thisFig = figure(5);
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
set(thisFig, 'Position', [1+560 1 560 420])

thisFig = figure(6);
%clf
throttle = wingeron_u(:,2);
ailR = wingeron_u(:,7);
t_u = wingeron_u(:,8);
%plot(t_u-tstart, throttle);

elevator = wingeron_u(:,5);
rudder = wingeron_u(:,4);
t_u_ret = wingeron_u_return_gumstix(:,8);
ailR_ret = wingeron_u_return_gumstix(:,7);
ailL = wingeron_u(:,6);
ailL_ret = wingeron_u_return_gumstix(:,6);

elevator0 = elevator(tcrossind-50);
rudder0 = rudder(tcrossind-50);
ailL0 = ailL(tcrossind-50);
ailR0 = ailR(tcrossind-50);
throttle0 = throttle(tcrossind-50);

umsg = struct();
umsg.rudder = rudder-rudder0;
umsg.elevator = elevator-elevator0;
umsg.aileronLeft = ailL-ailL0;
umsg.aileronRight = ailR-ailR0;

umsgDeg = commandsToDegrees(umsg);

hold on
cl6 = plot(t_u-tcross, umsgDeg.aileronRight, 'xr-');
%plot(t_u_ret, ailR_ret, 'xk-');
%plot(t_u, ailL, 'xb-')
%plot(t_u, ailL-ailR, 'xm-');
%legend('Thottle','AilR', 'AiR Return', 'AilDiff');
%title('wingeron u');
xlim([0 tend]);
box on

%set(thisFig, 'Position', [850 1 560 420])


figure(17)
cl17=plot(t_u-tcross, 100/255*(throttle-throttle0),'rx-');
hold on

figure(18)
cl18=plot(t_u-tcross, umsgDeg.elevator,'rx-');
hold on

figure(19)
cl19=plot(t_u-tcross, umsgDeg.rudder,'rx-');
hold on

figure(20)
cl20=plot(t_u-tcross, umsgDeg.aileronLeft,'rx-');
hold on