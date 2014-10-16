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
