% make plot for thesis
% comment out loadLog.m's load for the .mat and the close all / megaclear


megaclear
load ../controllers/matlab_tvlqr/rollTrajOptJune1_30z_73.mat


for i=[44:49]
    thisFile = ['sync/jun1/lcmlog_2012_06_02_' num2str(i) '.mat'];
    
    disp(['Plotting ' thisFile ' ...']);
    
    load(thisFile)
    
    loadLog
    
    figure(100);
    
    plot(t,wingeron_xhat(:,11)+wingeron_xhat(:,12)+wingeron_xhat(:,13));
    
    
    
    xlim([tstart tend-1]);
    
    
end

thisFile = ['sync/jun1/lcmlog_2012_06_02_21.mat'];

disp(['Plotting ' thisFile ' ...']);

load(thisFile)

plotLogs
%plotLogsXhat
[~, istart] = min(abs(t-tstart));
[~, iend] = min(abs(t-tend));

[~, tcrossind] = min(abs(x - 1000));
tcross = t(tcrossind);


% open loop trajectory = 21

thisFig = figure(6);

throttle = wingeron_u(:,2);
ailR = wingeron_u(:,7);
t_u = wingeron_u(:,8);
%plot(t_u-tstart, throttle,'g*-');

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
ol6 = plot(t_u-tcross, umsgDeg.aileronRight, 'bx-');
%plot(t_u_ret, ailR_ret, 'xk-');
%plot(t_u, ailL, 'xb-')
%plot(t_u, ailL-ailR, 'xm-');
%legend('Thottle','AilR', 'AiR Return', 'AilDiff');
%title('wingeron u');
%legend([ol6 cl6],'Open Loop', 'Closed Loop');
legend([ol6 cl6], 'Open Loop','Closed Loop','Position','SouthEast');
axis([-.31 .77 -80 80]);

set(thisFig, 'Position', [850 1 560 420])
grid on
xlabel('Time (s)')
ylabel('Commanded Deflection (deg)');
%saveasAll('knifeEdgeUtapes',20);
saveasAll('figures/plots/knife-edge/ailRComparison',30)

figure(17)
ol17 = plot(t_u-tcross, 100*(throttle-throttle0)/255,'bx-');
hold on
axis([-.2 1.2 -10 260])
legend([ol17 cl17], 'Open Loop','Closed Loop');
axis([-.31 .77 0 100]);
grid on
xlabel('Time (s)')
ylabel('Throttle (%)');
saveasAll('figures/plots/knife-edge/throttleComparison',30)

figure(18)
ol18=plot(t_u-tcross, umsgDeg.elevator,'bx-');
hold on
%legend([ol18 cl18], 'Open Loop','Closed Loop');
axis([-.31 .77 -80 80]);
grid on
xlabel('Time (s)')
ylabel('Commanded Deflection (deg)');
saveasAll('figures/plots/knife-edge/elevatorComparison',30)

figure(19)
ol19=plot(t_u-tcross, umsgDeg.rudder,'bx-');
hold on
%legend([ol19 cl19], 'Open Loop','Closed Loop');
axis([-.31 .77 -80 80]);
grid on
xlabel('Time (s)')
ylabel('Commanded Deflection (deg)');
saveasAll('figures/plots/knife-edge/rudderComparison',30)

figure(20)
ol20=plot(t_u-tcross, umsgDeg.aileronLeft,'bx-');
hold on
%legend([ol20 cl20], 'Open Loop','Closed Loop');
axis([-.31 .77 -80 80]);
grid on
xlabel('Time (s)')
ylabel('Commanded Deflection (deg)');
saveasAll('figures/plots/knife-edge/ailLComparison',30)


thisFig = figure(7);

ol7 = plot(x(istart:iend)/1000,z(istart:iend)/1000,'kx');
axis([-1 5 -pi/2 pi/2])
xlabel('X (m)')
ylabel('Z (m)');
legend([ref7 ol7 cl7], 'Reference','Open Loop','Closed Loop');
set(gca,'YDir', 'reverse');
set(thisFig, 'Position', [20+560 1 560 420])
%saveasAll('knifeEdgeZvsX',20);

thisFig = figure(8);
xs = xtraj.eval(xtraj.getBreaks);
ol8 = plot(x(istart:iend)/1000,y(istart:iend)/1000,'kx');
axis([-1 5 -pi/2 pi/2])
xlabel('X (m)')
ylabel('Y (m)');
legend([ref8 ol8 cl8],'Reference','Open Loop','Closed Loop');
set(thisFig, 'Position', [20 1 560 420])
axis([-.5 5.5 -.5 2.3]);
%saveasAll('knifeEdgeYvsX',20);



thisFig = figure(9);
openLoopHandle = plot(x(istart:iend)/1000,roll(istart:iend)*180/pi,'kx');
axis([-1 5 -90 90])
xlabel('X (m)')
ylabel('Roll (deg)');
legend([refHandle openLoopHandle closedLoopHandle], 'Reference','Open Loop','Closed Loop','Location','NorthWest');
set(thisFig, 'Position', [850 1 560 420])

axis([-.5 5.25 -35 125])
%saveasAll('knifeEdgeRollvsX', 20);

disp('done.')

%% note: to do horz, just use loadlog