function reconstructGUI(app)
%% Reconstruction function for the app.
% Reconstruct 3d points from the Gray-coded structured light images.

%% License
% ACADEMIC OR NON-PROFIT ORGANIZATION NONCOMMERCIAL RESEARCH USE ONLY
% Copyright (c) 2018 Bingyao Huang
% All rights reserved.

% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% If you publish results obtained using this software, please cite our paper.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

%% Options
% TODO: 1. depthGT. 2. mesh. 3. denoise point cloud. 4. stitch point cloud
dataRoot = app.dataRoot;
dataName = app.dataName;

calibInfo = Calibration.loadCalibInfo(fullfile(dataRoot, dataName, 'calib'));
dataFolder = fullfile(dataRoot, dataName);
% imgIdx = app.ReconImageTree.SelectedNodes;

selectedNode = app.ReconImageTree.SelectedNodes;
if(contains(selectedNode.Text, 'img'))
    selectedSet = selectedNode.Parent.Text;
    selectedTextureID = str2num(selectedNode.Text(end-1:end)); % for point cloud color
elseif(contains(selectedNode.Text, 'Set'))
    selectedSet = selectedNode.Text;
    selectedTextureID = 2; % for point cloud color (default is the dark image)
else
    uialert(app.ProCamCalibUIFigure, 'Can only select Setxx','Error');
end
reconSetDir = fullfile(dataFolder, 'recon', selectedSet);

stereoParams = app.stereoParams;

% debug option, enable for visuals/figures
verbose = app.reconOption.verbose;

% close previous waitbar
delete(findall(0,'type','figure','tag','TMWWaitbar'));

% start waitbar
msg = 'Extracting structured light nodes from camera image...';
waitBarHandle = waitbar(0.3, msg, 'Name', 'Reconstructing...');
set(findall(waitBarHandle, '-not', {'Type', 'AnnotationPane'}), 'Units', 'normalized');
waitBarHandle.Position(3) = 0.3;
disp(msg)

%% Step 1: Extract SL points from selected image
camW = calibInfo.camW;
camH = calibInfo.camH;
prjW = calibInfo.prjW;
prjH = calibInfo.prjH;

imCam = readAllImgs(reconSetDir);
[camPts, prjPts, imMax, imMin] = Reconstruct.decodeGrayPattern(imCam, prjW, prjH, app.reconOption.thresh);


sl.camPts2d = camPts;
sl.prjPts2d = prjPts;

cv.FileStorage(fullfile(reconSetDir, 'sl.yml'), sl);

% verify SL matches
if(verbose)
    
    % figure;showMatchedFeatures(0.3*ones(prjH, prjW,3), imCam(:,:,:,1), prjPts(idx,:), camPts(idx,:), 'montage');
    imC = 0.5*imCam(:,:,:,13) + 0.5*imCam(:,:,:,35);
    %imP = SL.warpCamImgToPrj(imC, camPts, prjPts, prjH, prjW);
    slPts.cam = camPts;
    slPts.prj = prjPts;   
    imP = SL.warpCamImgToPrj(imC, slPts, prjH, prjW);
%     imP = SL.warpCamImgToPrj(imC, camPts, prjPts, prjH, prjW, true, [256, 256]); % crop and resize
    figure; title('Camera-captured image warped to projector view');
    idx = randsample(length(prjPts), 5);
    showMatchedFeatures(imP, imC, prjPts(idx,:), camPts(idx,:), 'montage');

%     imC = imread(fullfile(app.dataRoot, app.dataName, 'calib/colorGrid01.png'));
%     imP = SL.warpCamImgToPrj(imC, slPts, prjH, prjW);
%     idx = randsample(length(prjPts), 10);
%     figure;showMatchedFeatures(imP, imC, prjPts(idx,:), camPts(idx,:), 'montage');
 end

% extract matched color grid Nodes' coords in camera and projector images
msg = 'Matching structured light pattern...';
waitbar(0.45, waitBarHandle, msg);
disp(msg);

% [camNodes, prjNodes, Nodes, Edges] = ImgProc.getMatchedNodes(imLightName, imColorName, [], prjW, prjH, verbose);

%% triangulate nodes
msg = 'Triangulating node coordinates...';
waitbar(0.5, waitBarHandle, msg);
disp(msg);

% intrinsics
camK = stereoParams.camK;
camKc = stereoParams.camKc;

prjK = stereoParams.prjK;
prjKc = stereoParams.prjKc;

% extrinsics
R = stereoParams.R;
T = stereoParams.T;
F = stereoParams.F;

% VERY IMPORTANT!!!!, undistort camera image points first
% camNodesUndistort = ImgProc.cvUndistortPoints(camNodes, camK, camKc);
% prjNodesUndistort = ImgProc.cvUndistortPoints(prjNodes, prjK, prjKc);

camPtsUndistort = ImgProc.cvUndistortPoints(camPts, camK, camKc);
prjPtsUndistort = ImgProc.cvUndistortPoints(prjPts, prjK, prjKc);

% remove epipolar outliers
[inlierIdx, d] = Reconstruct.findEpipolarInliers(F, camPtsUndistort, prjPtsUndistort, app.reconOption.epiThresh, verbose);
camPtsUndistort = camPtsUndistort(inlierIdx,:);
prjPtsUndistort = prjPtsUndistort(inlierIdx,:);
    
% triangulate Node's 3d coordinates
pts3d = Reconstruct.triangulatePoints(camK, prjK, R, T, camPtsUndistort, prjPtsUndistort);

% visualize reconstructed 3d Nodes
% Reconstruct.visualizePts3d(pts3d, R, T, 'Point Cloud');
figure; pcshow(pointCloud(pts3d)); title('Point Cloud');
% fig = uifigure;
% ax = axes(parent=fig);
% pcshow(pointCloud(pts3d), parent=ax); 
% title('Point Cloud');


%% interpolate point cloud
msg = 'Colorizing point cloud...';
waitbar(0.8, waitBarHandle, msg);
disp(msg);

% use Nayar's method to get mask
b = 0.9; % projector back light stength (for mask use a large b, for real direct/indirect separation, use a smaller b)
imDirect = (imMax - imMin)/(1-b); % direct light image
% imIndirect = 2*(imMin-b*imMax)/(1-b*b); % indirect (global) light image
% fm({imDirect, imIndirect});
    
imDirectSmooth = cv.GaussianBlur(imDirect, 'KSize', [3, 3], 'SigmaX', 1.5);
% fs(imDirectSmooth > graythresh(imDirectSmooth));
imMask = imDirectSmooth > graythresh(imDirectSmooth);
imTexture = ImgProc.maskImage(imCam(:,:,:,selectedTextureID), imMask); 

if(app.InterpolatePointCloudCheckBox.Value)
    ptCloudOut = Reconstruct.interpolatePtCloud(camK, camKc, pointCloud(pts3d), imTexture, imMask, verbose);
else
    ptCloudOut = Reconstruct.colorizePtCloud(camK, camKc, pts3d, imTexture, imMask);
end

figure;
h = pcshow(ptCloudOut, 'MarkerSize', 50);
title('(Interpolated) Colored point cloud');

% save colorized point cloud
pcFileName = fullfile(dataFolder, 'recon', ['pointCloud_', selectedSet, '.ply']);
pcwrite(ptCloudOut, pcFileName);


%% depth and normal maps
msg = 'Converting point cloud to depth and normal maps...';
waitbar(0.9, waitBarHandle, msg);
disp(msg);

% imN = Reconstruct.pointCloudToNormalMap(camK, camKc, ptCloudOut, imMask);
imD = Reconstruct.pointCloudToDepthMap(camK, camKc, pts3d, imMask);

if(app.InterpolatePointCloudCheckBox.Value)
    % imD = fillmissing(imD, 'spline', 'MissingLocations', (imD==0)&imMask).*imMask;
    imD = fillmissing(imD, 'movmedian', 20, 'MissingLocations', (imD==0)&imMask).*imMask;
end
fsc(imD); title('Depth map');

imN = Reconstruct.depthMapToNormalMap(camK, imD, imMask);
fs(imN); title('Normal map');


%% Finished
waitbar(1.0, waitBarHandle, 'Reconstruction done!');
close(waitBarHandle);
% uiconfirm(app.ProCamCalibUIFigure,['Point Cloud saved to ', pcFileName], 'Reconstruction complete!', 'Options', {'OK'},'icon','success');
end
