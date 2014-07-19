% generate plots for stereo paper
clear
close all

passes = [ 1 2 3 ];

doFalseNeg = 0;
enable_boxed = 1;

figstart = 0
sumfig = 20;

for i = 1 : length(passes)
  
  pass_number = i;
  
  figure(i+figstart)
  clf
  doComparisons;
  
  
  
end

%%
clear

enable_boxed = 1;
passes = [ 1 2 3 ];
doFalseNeg = 1;
figstart = 0
sumfig = 21;
for i = 1 : length(passes)
  
  pass_number = i;
  
  figure(3+i+figstart)
  clf
  doComparisons;
  
end

