p = PlanePlant;
load rollHandTunedMay19.mat
%t_sample = 0:0.1:1;
%inputs = [zeros(3,length(t_sample));1*ones(1,length(t_sample))];
%u = PPTrajectory(spline(t_sample,inputs));
u = utrajdeg;
t_sample = u.getBreaks;
sys = cascade(u,p);
%InitialStates = 0.0*randn(12,1);
%InitialStates(7) = 5;
x0 = [1 0 -0.03 -0.05 0.06 0.02 6 0.6 -1 1 -6.3 -3];
InitialStates = x0; %xs(:,1);

disp('simulating')
xtrajs = sys.simulate([0 0.5],InitialStates);

xss = xtrajs.eval(xtrajs.getBreaks);



