function [newPc, meanTranslate] = translatePcToOriginPoint(ptCloud)
%% Translate point cloud to origin point.

%%
meanX = mean(ptCloud.Location(:,1));
meanY = mean(ptCloud.Location(:,2));
meanZ = mean(ptCloud.Location(:,3));

newX = ptCloud.Location(:,1) - meanX;
newY = ptCloud.Location(:,2) - meanY;
newZ = ptCloud.Location(:,3) - meanZ;

pts3d = [newX,newY,newZ];
newPc  = pointCloud(pts3d);

meanTranslate = [meanX, meanY, meanZ];
end