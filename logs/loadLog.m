%close all
megaclear
close all

load jun1/lcmlog_2012_06_03_41.mat
    
plotLogs
plotLogsXhat

thisFig = figure(7);
load ../controllers/matlab_tvlqr/rollTrajOptJune2_horz_2.mat
xs = xtraj.eval(xtraj.getBreaks);
plot(xs(1,:),xs(3,:),'bo')
hold on
plot(x,z,'ro')
axis([-1 5 -pi/2 pi/2])
xlabel('x (m)')
ylabel('z (m)');
set(thisFig, 'Position', [20+560 1 560 420])

thisFig = figure(8);
load ../controllers/matlab_tvlqr/rollTrajOptJune2_horz_2.mat
xs = xtraj.eval(xtraj.getBreaks);
plot(xs(1,:),xs(2,:),'bo')
hold on
plot(x,y,'ro')
axis([-1 5 -pi/2 pi/2])
xlabel('x (m)')
ylabel('y (m)');
set(thisFig, 'Position', [20 1 560 420])

thisFig = figure(9);
plot(xs(1,:),xs(4,:)*180/pi,'bo')
hold on
plot(x,roll*180/pi,'ro')
axis([-1 5 -90 90])
xlabel('x (m)')
ylabel('roll (deg)');
set(thisFig, 'Position', [850 1 560 420])