p = PlanePlant;
%load rollHandTunedMay19.mat
%Q = diag([10 10 10 100 10 10 1 1 1 1 1 1]); R = 0.2*diag([0.1 1 1 1 1]);
% COSTS FOR MAY30-31 GOOD: Q = diag([10 10 10 300 10 10 1 1 1 1 1 1]); R = 0.2*diag([0.1 1 1 0.5 0.5]);
%Q = 10000*diag([10 10 10 300 10 10 1 1 1 1 1 1]); R = 10000*0.2*diag([0.1 1 1 0.06 0.06]);

%Q = diag([10 10 10 300 10 10 1 1 1 5.2 1 1]); R = 0.2*diag([0.1 1 1 0.4 0.4]);

%Q = 10*diag([10 10 10 100 10 10 1 1 1 4 1 1]); R = 10*0.2*diag([0.1 1 1 0.3 0.3]);

Q = 10*diag([10 10 10 100 10 10 1 1 1 4.2 1 1]); R = 10*0.2*diag([0.1 1 1 0.3 0.3]);

%FINAL COSTS FOR HORIZONTAL AND VERTICAL FLIGHT:
%Q = 10*diag([10 10 10 100 10 10 1 1 1 4.2 1 1]); R = 10*0.2*diag([0.1 1 1 0.3 0.3]);




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



