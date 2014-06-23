load ../controllers/matlab_tvlqr/rollTrajOptJune1_30z_73.mat
us = utrajdeg.eval(utrajdeg.getBreaks);

myT = t_u_ret;

inds = 1:length(myT);

ind = inds(abs(myT - 70.68) < 0.01)
ind = ind(end)


figure(7)
clf;
plot(myT(ind:ind+50)-myT(ind),ailL_ret(ind:ind+50),'bx-')
hold on
plot(utrajdeg.getBreaks,us(4,:),'rx-')
legend('u ret','u tape');