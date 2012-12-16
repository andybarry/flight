%close all
%megaclear
%close all

% successful vertical trajectory
%load sync/jun1/lcmlog_2012_06_02_44.mat
%load ../controllers/matlab_tvlqr/rollTrajOptJune1_30z_73.mat

% successful horizontal trajectory
%load sync/jun1/lcmlog_2012_06_02_48.mat
%load ../controllers/matlab_tvlqr/rollTrajOptJune1_30z_73.mat

% successful horizontal trajectory
%load sync/jun1/lcmlog_2012_06_03_41.mat
%load ../controllers/matlab_tvlqr/rollTrajOptJune2_horz_2.mat    



plotLogs
plotLogsXhat

[~, istart] = min(abs(t-tstart));
[~, iend] = min(abs(t-tend));


thisFig = figure(7);

xs = xtraj.eval(xtraj.getBreaks);
ref7 = plot(xs(1,:),xs(3,:),'b-','LineWidth',2);
hold on
cl7 = plot(x(istart:iend)/1000,z(istart:iend)/1000,'rx');
axis([-1 5 -pi/2 pi/2])
xlabel('X (m)')
ylabel('Z (m)');
%legend('Reference Trajectory','Closed Loop Trajectory');
set(gca,'YDir', 'reverse');
set(thisFig, 'Position', [20+560 1 560 420])

thisFig = figure(8);
xs = xtraj.eval(xtraj.getBreaks);
ref8 = plot(xs(1,:),xs(2,:),'b-','LineWidth',2);
hold on
cl8 = plot(x(istart:iend)/1000,y(istart:iend)/1000,'rx');
axis([-1 5 -pi/2 pi/2])
xlabel('X (m)')
ylabel('Y (m)');
%legend('Reference Trajectory','Closed Loop Trajectory');
set(thisFig, 'Position', [20 1 560 420])

thisFig = figure(9);
refHandle = plot(xs(1,:),xs(4,:)*180/pi,'b-','LineWidth',2);
hold on
closedLoopHandle = plot(x(istart:iend)/1000,roll(istart:iend)*180/pi,'rx');
axis([-1 5 -90 90])
xlabel('X (m)')
ylabel('Roll (deg)');
%legend('Reference','Closed Loop');
set(thisFig, 'Position', [850 1 560 420])
%%
thisFig = figure(10);

[~, tFireIndex] = min(abs(x-1));
tFire = t(tFireIndex);
tref = xtraj.getBreaks() + tFire;
ref10 = plot(tref,xs(4,:)*180/pi,'bx-');
hold on
cl10 = plot(t,roll*180/pi,'rx-');
axis([tstart tend -90 90])
xlabel('Time (s)')
ylabel('Roll (deg)');
legend('Reference Trajectory','Closed Loop Trajectory');
%set(thisFig, 'Position', [850 1 560 420])


%plotTrajectories
