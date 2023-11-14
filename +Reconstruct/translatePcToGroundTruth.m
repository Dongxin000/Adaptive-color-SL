function [newPC] = translatePcToGroundTruth(pcGroundRef, pcGroundCurrent)
%% Translate point cloud to ground truth position.

%%
temp = [sum(pcGroundRef.XLimits)/2, sum(pcGroundRef.YLimits)/2, sum(pcGroundRef.ZLimits)/2];
t = repmat(temp,[size(pcGroundCurrent.Location, 1), 1]);
new3d = pcGroundCurrent.Location + t;

% output
newPC = pointCloud(new3d);
end

