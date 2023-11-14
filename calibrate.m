%% A single-shot camera and projector calibration system for imperfect planar targets
% Calibrates the camera and projector using single-shot
% colored structured light.

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

%%
clear; close all

%% Options
dataRoot = 'data';
dataName = 'calibration-11-13-17';

calibInfo = Calibration.loadCalibInfo(fullfile(dataRoot, dataName));
calibInfo.dataName = dataName;

% folder where extracted checkerboard corners are stored
cornerDir = fullfile(calibInfo.path, 'matlabCorners');

% debug option, enable for visuals/figures
verbose = 1;

% checkerboard corners will be extracted and saved by the script.
% If no entry in calib-info.yml file has been modified since the last
% calibration, you can set the two flags to true to speed up recalibration.
% But you must set them to false if you change sets, camera, projector or
% checkerobard settings in calib-info.yml.
useExistingCamCorners = 0;
useExistingPrjCorners = 0;

format short

%% Step 1: Get checkerboard corners from camera image
if (~useExistingCamCorners)
    disp('Extracting checkerboard corners from camera image...');
    
    % 1. Get the checkerboard points for the selected calibInfo.sets from camera image
    [camCorners, usedImIdx] = Calibration.getCameraCorners(calibInfo);
    
    % 2. Save the camera corners, and load them to eliminate rounding errors.
    Calibration.saveCorners(camCorners, 'cam_', cornerDir, calibInfo.sets, usedImIdx);
    camCorners = camCorners(:, :, usedImIdx);
    
    % 3. Eliminate any potentially unused calibInfo.sets during checkerboard detection
    calibInfo.sets = calibInfo.sets(usedImIdx);
    calibInfo.numSets = numel(calibInfo.sets);
else
    % Read camCorners
    disp('Reading existing camera corners...');
    camCorners = Calibration.readCorners('cam_', cornerDir, calibInfo);    
end

%% Step 2: Calibrate camera, (skip 1 if using existing camera corners)
disp('Calibrating camera using checkerboard corners...');

% 1. Generate world corners
modelCornersCell = Calibration.generateModelCorners(calibInfo);

% 2. Convert camCorners to (mex)OpenCV format
camCornersCell = squeeze(mat2cell(camCorners, calibInfo.numCorners, 2, ones(1, calibInfo.numSets)))';

% 3. Calibrate camera, only used to warp nodes to model space (not init guess)
camParams = Calibration.calibrateInitGuess(modelCornersCell, camCornersCell, calibInfo);

%% Step 3: Calculate projector nodes (Xp) for proposed method

if (~useExistingPrjCorners)
    % 1. Get SL nodes from specified image sets
    disp('Extracting and saving grid nodes and warped projector corners...');
    [nodesCell, ~] = Calibration.getNodesAndPrjCorners(calibInfo, camParams, camCorners, verbose);
    
    % 2. Save projector points and node pairs
    cv.FileStorage(fullfile(calibInfo.path, 'nodePairs.yml'), nodesCell);
else
    disp('Reading existing node coordinates...');
    nodesCell = cv.FileStorage(fullfile(calibInfo.path, 'nodePairs.yml')).nodePairs;
end

%% Step 4. Warp node points (Xc) in camera image to model space (Xm)
disp('Warpping nodes to model space (Xc to Xm) for proposed method...');
% refer to paper to understand Xm, Xc, Xp
Xm = []; % node points in model space
Xc = []; % node points in camera image space
Xp = []; % node points in projector image space

% 1. Undistorted node points in camera image space
XcUndistort = cell(1, calibInfo.numSets);

% 2. Warp Xc to model space to get Xm
for i = 1:calibInfo.numSets
    % according to Zhang's method the homography Hmc is:
    % H = lambda*K*[r1, r2, t], where r1, r2 are 1st and 2nd column of R
    R = cv.Rodrigues(camParams.rVecs{i});
    Hmc = camParams.camK * [R(:, 1:2), camParams.tVecs{i}];
    lambda = 1 / norm(inv(camParams.camK) * Hmc(:, 1));
    Hmc = lambda * Hmc;
    
    % undistort grid points in camera image space
    Xc{i} = nodesCell{i}(:, 1:2);
    Xp{i} = nodesCell{i}(:, 3:4);
    
    % undistort grid points in camera image
    XcUndistort{i} = ImgProc.cvUndistortPoints(Xc{i}, camParams.camK, camParams.camKc);
    
    % transform camera image grid points to white board model space using
    % H, then use calibrated tvecs and rvecs to transform grid points from
    % model space to world space.
    % NOTE: the cb corners have to be undistorted
    curXm = ImgProc.applyHomography(XcUndistort{i}, inv(Hmc));
    curXm = [curXm, zeros(length(curXm), 1)];
    Xm{i} = curXm;
end

%% Step 5. Calibrate camera and projector using the proposed method
disp('[Proposed] Performing camera-projector calibration using BA...');

% 1. Use 'BA' option to specify bundle adjustment on Xm
stereoParams = Calibration.stereoCalibrate(Xm, Xc, Xp, calibInfo.camImgSize, calibInfo.prjImgSize, 'BA');
stereoParams.sets = calibInfo.sets;
stereoParams.dataName = calibInfo.dataName;

% 2. Calculate reprojection errors
stereoParams = Calibration.calcReprojectionError(stereoParams, stereoParams.modelPts, stereoParams.camImgPts, stereoParams.prjImgPts);

%3. Display calibration results
stereoParams

% debug
if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParams.worldPts'), stereoParams.R, stereoParams.T, 'Proposed calibration: Xm ');
end

%% Step 6. Save calibration data
calibFileName = fullfile(calibInfo.resultDir, ['proposed' , '.yml']);
cv.FileStorage(calibFileName, stereoParams);
disp(['Calibration data saved to ', calibFileName]);