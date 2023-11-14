function [totalLoss] = objFunc(idx,AllVecHue)
%% Objecive function 

%%
C = nchoosek(1:size(idx,2),2);
totalLoss = 0;

for i=1:size(C,1)
    currIdx1 = C(i, 1);
    currIdx2 = C(i, 2);
    totalLoss = totalLoss + AdaptiveGenPattern.calDegreeOfIntersection(AllVecHue(idx(currIdx1)).weight, AllVecHue(idx(currIdx2)).weight);
end

% add a penalty to make each variable not equal to each other
totalLoss = totalLoss + (1e+10) * (size(idx,2) - numel(unique(idx))); 
end

