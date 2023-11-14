function [optimalSolution,optimalLoss] = calcOptimalColorComb(recordedHueAndVariance, count, nVars)
%% Iteratively solving for the optimal result using genetic algorithms.

%%
index = [1:count-1];

% the number of variables
%nVars = 5;

%lower and upper bounds
LB = ones(1, nVars);
UB = (size(index,2))*ones(1, nVars);

%variables with integer constraints (all in this case)
IntCon = 1:nVars;

% options
options = gaoptimset('PlotFcns',{@gaplotbestf,@gaplotbestindiv});

% run the GA solver
[optimalSolution,  optimalLoss]= ga(@(idx)AdaptiveGenPattern.objFunc(idx,recordedHueAndVariance), nVars, [], [], [], [], LB, UB, [], IntCon, options);
end

