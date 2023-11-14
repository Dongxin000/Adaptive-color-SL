function [pcGroundTruthTemp, pcCalibAligned, rmse, meanTranslateParam1, meanTranslateParam2, tform] = alignPcToGroundTruth(pcGroundTruth, pcCalib, alignParam)
%% Align reconstructed point cloud to ground truth point cloud.

%% Step1. coarse registration
% unified unit
pt3d = (pcGroundTruth.Location).*1000;

pt3d(:,2) = -pt3d(:,2);
pt3d(:,3) = -pt3d(:,3);
pcGroundTruthTemp = pointCloud(pt3d);

pcGroundTruthTemp = Reconstruct.rotatePC(pcGroundTruthTemp, alignParam(1, 2), alignParam(1, 3), alignParam(1, 4));

% translate to origin point
[pcGroundRef, meanTranslateParam1] = Reconstruct.translatePcToOriginPoint(pcGroundTruthTemp);
[pcCalibCurrent, meanTranslateParam2] = Reconstruct.translatePcToOriginPoint(pcCalib);

if(alignParam(1,1))
    % translate the point cloud to the position of ground truth to facilitate icp
    pcCalibCurrent = Reconstruct.translatePcToGroundTruth(pcGroundRef, pcCalibCurrent);
end

%% step2. iterative closest point (ICP)
% downsample and denoise to save time and memory
gridSize = 0.01;

fixed = pcdownsample(pcdenoise(pcGroundRef,'Threshold',5), 'gridAverage', gridSize);
moving = pcdownsample(pcdenoise(pcCalibCurrent,'Threshold',5),  'gridAverage', gridSize);

% register pcCalib to fixed pcGroundTruth
[tform, ~, rmse] = pcregistericp(moving, fixed,'Extrapolate', true, 'MaxIterations', 100);

% disp(['ICP root mean square error: ',num2str(rmse)]);

% align current to ref
pcCalibAligned = pctransform(pcCalibCurrent, tform);

%% step3. translate
originPt3d = pcCalibAligned.Location;
originPt3d(:,1) = originPt3d(:,1) + meanTranslateParam1(1);
originPt3d(:,2) = originPt3d(:,2) + meanTranslateParam1(2);
originPt3d(:,3) = originPt3d(:,3) + meanTranslateParam1(3);

pcCalibAligned = pointCloud(originPt3d);

%% show result
% figure;pcshowpair(pcCalibAligned, pcGroundTruthTemp)

end

