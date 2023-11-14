function [] = visualizePointCloud(pcCalib, pcGroundTruth, param, camW, camH, verbose, figName, meanTranslateParam1, meanTranslateParam2, tform, alignParam)
%% Visualize point cloud for differet methods mentioned in paper.

%%
if(nargin < 7)
    figName = '';
end

%%
% convert point cloud to XYZ map
% imPC = zeros(camH, camW, 3);
% imPC(:,:,1) = (reshape(pcGroundTruth.Location(:,1), camW, camH))';
% imPC(:,:,2) = (reshape(pcGroundTruth.Location(:,2), camW, camH))';
% imPC(:,:,3) = (reshape(pcGroundTruth.Location(:,3), camW, camH))';
% imPC = double(imPC);

% pts3dGT = double(pcGroundTruth.Location);
% pts3dGT = pts3dGT(pts3dGT(:,3)>0,:);
% pts2dGT = cv.projectPoints(pts3dGT, [0,0,0], [0,0,0], param.camK, 'DistCoeffs', param.camKc);
% imD = zeros(camH, camW);
% pts2dGT = round(pts2dGT);
% inlierIdx = pts2dGT(:,1)<=camW & pts2dGT(:,2)<=camH & pts2dGT(:,1)>0 & pts2dGT(:,2)>0;
% pts2dGT = pts2dGT(inlierIdx,:);
% pts3dGT = pts3dGT(inlierIdx,:);
% imD(sub2ind([camH, camW], pts2dGT(:,2), pts2dGT(:,1))) = pts3dGT(:,3);

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

%% interpolate point cloud
rp = regionprops(imMask, 'boundingbox');
desity = 4;
npX = rp.BoundingBox(3)*desity;
npY = rp.BoundingBox(4)*desity;

% xRange contains lower and higher bounds
xRange = double(pcCalib.XLimits);
xDiff = xRange(1) - xRange(2);
xStep = -xDiff / npX;

yRange = double(pcCalib.YLimits);
yDiff = yRange(1) - yRange(2);
yStep = -yDiff/npY;

% use griddata to interpolate a mesh from given point cloud
% pts3d = pcCalibAligned.Location;
[xq,yq] = meshgrid(xRange(1):xStep:xRange(2), yRange(1):yStep:yRange(2));
zq = griddata(pts3d(:,1),pts3d(:,2),pts3d(:,3), xq, yq,'cubic');

% pcInterp is the interpolated denser point cloud
pcInterp = pointCloud([xq(:), yq(:), zq(:)]);

% finer registration
% moving = pcdownsample(pcdenoise(pcInterpAligned,'Threshold',5), 'gridAverage', gridSize);
% [tform, ~, ~] = pcregrigid(moving, fixed,'Extrapolate', true);
% pcInterpAligned = pctransform(pcInterpAligned, tform);
% 

% add the carefully adjusted mask
% imMask = imread('C:\Users\hp\Desktop\koutu\david18\imMask.png');
% imMask = logical(imMask);
[~, ~, ~, pcInlierIdx] = Reconstruct.filterPointCloud(pcInterp, imMask, param, camW, camH);
pcOutlierIdx = setdiff(1:size(xq,1)*size(xq,2), pcInlierIdx);

%% align reconstructed point cloud with ground truth point cloud
[pcGroundRef] = Reconstruct.translatePcToOriginPoint(pcGroundTruth);

pt3d = pcInterp.Location;

pt3d(:,1) = pt3d(:,1) - meanTranslateParam2(1);
pt3d(:,2) = pt3d(:,2) - meanTranslateParam2(2);
pt3d(:,3) = pt3d(:,3) - meanTranslateParam2(3);

pcInterpGroundTruthPosition = pointCloud(pt3d);

if(alignParam(1, 1))
    % translate the point cloud to the position of ground truth to facilitate icp
    pcInterpGroundTruthPosition = Reconstruct.translatePcToGroundTruth(pcGroundRef, pcInterpGroundTruthPosition);
end

pcCalibAligned = pctransform(pcInterpGroundTruthPosition, tform);

originPt3d = pcCalibAligned.Location;
originPt3d(:,1) = originPt3d(:,1) + meanTranslateParam1(1);
originPt3d(:,2) = originPt3d(:,2) + meanTranslateParam1(2);
originPt3d(:,3) = originPt3d(:,3) + meanTranslateParam1(3);

pcInterpAligned = pointCloud(originPt3d);

% pcInterpAlignedInier = pointCloud(originPt3d(pcInlierIdx,:));

visPC = pcInterpAligned;
visPC = Reconstruct.rotatePC(visPC, alignParam(1, 2), alignParam(1, 3), alignParam(1, 4));

newXq = visPC.Location(:,1);
newYq = visPC.Location(:,2);
newZq = visPC.Location(:,3);

newXq = reshape(newXq, size(xq,1), size(xq,2));
newYq = reshape(newYq, size(yq,1), size(yq,2));
newZq = reshape(newZq, size(zq,1), size(zq,2));

%% visualize point cloud
if(verbose)
    figure('Name', figName);
    h = surf(newXq, newYq, newZq, 'FaceColor', 'interp', 'FaceLighting', 'gouraud'); % square mesh
%     h = trisurf(delaunay(xq,yq),xq, yq, zq, 'FaceColor', 'interp', 'FaceLighting', 'gouraud'); % triangular mesh
    
    % remove interpolated meth that are out of ground truth range
    h.XData(pcOutlierIdx) = nan;
    h.YData(pcOutlierIdx) = nan;
    h.ZData(pcOutlierIdx) = nan;
    h.CData(pcOutlierIdx) = nan;
    
    h.EdgeColor = 'none';
%     h.EdgeColor = [0.3,0.3,0.3];
    h.LineStyle = ':';
    colormap parula
    
    % title
    title(figName)
    caxis([0 8])
%     colorbar
    
    % remove ticks
    set(gca,'xtick',[])
    set(gca,'xticklabel',[])
    set(gca,'ytick',[])
    set(gca,'yticklabel',[])
    set(gca,'ztick',[])
    set(gca,'zticklabel',[])
    
    % set figure size
   % set(gcf,'position',[0,0, 300, 400]);
    
    % font
    set(gca, 'FontSize', 20);
    daspect([1 1 1]);
    axis vis3d tight
    
    rotate3d on
    box on
    set(gca, 'visible', 'off'); % ge trid of ugly box edges
end

light('Position',[0 1 -1]);
material dull % metal, shiny, default

%% calculte 3D alignment error using knn search
[~, err] = knnsearch(pcGroundTruth.Location, pcInterpAligned.Location);

% visualize errors as pseudocolor
if(verbose)
    errOutlierIdx = isnan(err);
    errInlierIdx = ~errOutlierIdx;
    h.ZData(errOutlierIdx) = nan;
    h.CData(errInlierIdx) = err(errInlierIdx); % use err as color  
%      h.CData(errInlierIdx) = abs(err(errInlierIdx)); % use abs err as color 
end

% remove nan values
err = err(~isnan(err));
end