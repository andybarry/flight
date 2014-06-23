
xlim(AX(1), [start_times(1)- 10, end_times(end)+20]);
xlim(AX(2), [start_times(1)- 10, end_times(end)+20]);

ylim(AX(2), [-20 120]);

xlabel('Time (s)');

ylabel(AX(2), 'Throttle (%)')
title([log.name '.' log.number]);

grid(AX(1), 'on')