
addpath('../../MillionTrajs'); % for trajectory plotting


% build the state vector
state = [x y z yaw pitch roll]';
stateRef = xs;

% start time is 58.26
% end time is 58.76

%startT = 58.01;
%endT = 58.76;

startT = tstart;
endT = tend - .5;

[~, startIndex] = min(abs(t - startT));
[~, endIndex] = min(abs(t - endT));


state = state(:, startIndex:endIndex);



figure(25)
clf
plotTraj(state, .1)


 
% now plot the reference one

% swap yaw / roll in the stateRef
refYaw = stateRef(6,:);
refPitch = stateRef(4,:);

stateRef(4,:) = refYaw;
stateRef(6,:) = refPitch;

plotTraj(stateRef, .1);


xlabel('x');
ylabel('y');
zlabel('z');
set(gca, 'CameraPosition', 1e4*[-1.5771   -0.9326    0.3965]);
drawnow
axis equal

set(gca,'ZDir', 'reverse');
