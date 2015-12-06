% make ground plot
clear
close all

doFalseNeg = 0;
sumfig = 20;
enable_boxed = 0;
enable_random = 0;

bm_depth_min = 4.55;
bm_depth_max = 5.15;

sum_false_pos = 1;


%%

pass_number = 6;
new_dir = 1;

SingleDepthComparison


pass_number = 27;
new_dir = 2;

SingleDepthComparison


pass_number = 29;
new_dir = 2;

SingleDepthComparison


pass_number = 33;
new_dir = 2;

SingleDepthComparison

%% create cdf like plot


figure(22)
clf
real_sum_sorted = sort(real_sum);

% so now I have a sorted vector
% compute a vector that is the percentages of values at this point
percent_vals = 0:1/length(real_sum_sorted):1-1/length(real_sum_sorted);

plot(real_sum_sorted, percent_vals,'b-')

xlabel('Separation (meters)')
ylabel('Fraction of Pixels')
title('');
xlim([0 6]);
%set(gca, 'XTickLabel',{'0','1','2','3','4','5','No Match'});
grid on

