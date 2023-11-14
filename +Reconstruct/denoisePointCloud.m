function [newPc] = denoisePointCloud(pcCalib, param, camW, camH)
%% Denoise reconstructed point cloud.

%%
pts3d = double(pcCalib.Location);
pts3d = pts3d(pts3d(:,3)>0,:);
pts2d = cv.projectPoints(pts3d, [0,0,0], [0,0,0], param.camK, 'DistCoeffs', param.camKc);
imD = zeros(camH, camW);
pts2d = round(pts2d);
inlierIdx = pts2d(:,1)<=camW & pts2d(:,2)<=camH & pts2d(:,1)>0 & pts2d(:,2)>0;
pts2d = pts2d(inlierIdx,:);
pts3d = pts3d(inlierIdx,:);
imD(sub2ind([camH, camW], pts2d(:,2), pts2d(:,1))) = pts3d(:,3);

% find depth image mask
imMask = false(size(imD));
imMask(imD > 0) = 1;
se = strel('disk', 5);
% imMask = imclose(imopen(imMask,  se), se);
imMask = imclose(imMask, se);
imMask = imfill(imMask, 'holes');
imMask = bwareafilt(imMask, 1);
imMask = imgaussfilt(double(imMask), 7) > 0.2; % smooth edges
% fs(imMask);

%% align reconstructed point cloud with ground truth point cloud using icp

% downsample and denoise to save time and memory
% gridSize = 0.1;
% fixed = pcdownsample(pcdenoise(pcGroundTruth, 'Threshold',5), 'gridAverage', gridSize);
% % moving = pcdenoise(pcdownsample(pointCloud(pts3dInterp), 'gridAverage', gridSize));
% moving = pcdownsample(pcdenoise(pcCalib,'Threshold',5), 'gridAverage', gridSize);
% 
% % register pcCalib to fixed pcGroundTruth
% [tform, ~, ~] = pcregistericp(moving, fixed,'Extrapolate', true, 'MaxIterations', 50);
% pcCalibAligned = pctransform(pcCalib, tform);
% figure;pcshowpair(fixed, pcCalibAligned)

[~, ~, ~, pcInlierIdx] = Reconstruct.filterPointCloud(pcCalib, imMask, param, camW, camH);

location = pcCalib.Location;
newLocation = location(pcInlierIdx,:,:);
newPc = pointCloud(newLocation);
end