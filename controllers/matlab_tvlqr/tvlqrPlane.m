p = PlanePlant;
%load rollHandTunedMay19.mat
Q = diag([10 10 10 50 10 10 1 1 1 1 1 1]); R = 50*diag([0.1 1 1 1 1]);

options.grad_method = 'numerical';
tv = tvlqr(p,xtraj,utrajdeg,Q,R,Q,options);

%  m = ModelVisualizer();
% 
% sys = cascade(feedback(p,tv),m);
% xtrajcl = simulate(sys,[0 0.6],x0);

% t0 = linspace(0,15,41);
% utraj = PPTrajectory(foh(t0,0.8*t0.*sin(4*t0)));
% sys = cascade(utraj,p);
% xtraj = simulate(sys,[0 10],[0;0;0;0]);
% v.playback(xtraj);



