function calibrateGUI(app)
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

%% Options
% turn on log
logFileName = fullfile(app.logDir, [datestr(now, 'yy-mm-dd-_HH-MM-SS'), '.txt']);
diary(logFileName);
diary on;
            
dataRoot = app.dataRoot;
dataName = app.dataName;

calibInfo = Calibration.loadCalibInfo(fullfile(dataRoot, dataName, 'calib'));
calibInfo.adapFolderName = fullfile(dataRoot, dataName, 'adap');

calibInfo.dataName = dataName;
calibInfo.sets = app.calibOption.sets;
calibInfo.sqSize = app.calibOption.sqSize;

% folder where extracted checkerboard corners are stored
cornerDir = fullfile(calibInfo.path, 'matlabCorners');

% debug option, enable for visuals/figures
verbose = app.calibOption.verbose;

% use matlab parfor to speedup calibration
useParallel = app.calibOption.useParallel;

% distortion model (has distortion or not)
useDistortion = app.calibOption.useDistortion;

% adaptive option
adaptiveOption = 0

hueLabel = load(fullfile(dataRoot, dataName, 'adap\hueLabel.mat'));
hueLabel = hueLabel.hueLabel;

% if(adaptiveOption == 1)
%     calibInfo.numColor{1} = 4;
%     calibInfo.numColor{2} = 5;
% else
%     calibInfo.numColor{1} = 4;
%     calibInfo.numColor{2} = 4;
% end

format short

% close previous waitbar
delete(findall(0,'type','figure','tag','TMWWaitbar'));

% start waitbar
msg = 'Extracting checkerboard corners from camera image...';
waitBarHandle = waitbar(0, msg, 'Name', 'Calibrating...');
set(findall(waitBarHandle, '-not', {'Type', 'AnnotationPane'}), 'Units', 'normalized');
waitBarHandle.Position(3) = 0.3;

%% Step 1: Get checkerboard corners from camera image
disp(msg);

% 1. Get the checkerboard points for the selected calibInfo.sets from camera image
[camCorners, usedImIdx] = Calibration.getCameraCorners(calibInfo);

if(nnz(usedImIdx) < 3)
    uialert(app.ProCamCalibUIFigure, ['Unable to extract checkerboard corners in set(s): ','[', num2str(calibInfo.sets(usedImIdx == 0))',']. Click AutoSeg to see details'], 'Not enough sets')
    return;
end

% 2. Save the camera corners, and load them to eliminate rounding errors.
Calibration.saveCorners(camCorners, 'cam_', cornerDir, calibInfo.sets, usedImIdx);
camCorners = camCorners(:, :, usedImIdx);

% 3. Eliminate any potentially unused calibInfo.sets during checkerboard detection
calibInfo.sets = calibInfo.sets(usedImIdx);
calibInfo.numSets = numel(calibInfo.sets);


%% Step 2: Calibrate camera, (skip 1 if using existing camera corners)
msg = 'Calibrating camera using checkerboard corners...';
waitbar(0.1, waitBarHandle, msg);
disp(msg);

% 1. Generate world corners
modelCornersCell = Calibration.generateModelCorners(calibInfo);

% 2. Convert camCorners to (mex)OpenCV format
camCornersCell = squeeze(mat2cell(camCorners, calibInfo.numCorners, 2, ones(1, calibInfo.numSets)))';

% 3. Calibrate camera, only used to warp nodes to model space (not init guess)
camParams = Calibration.calibrateInitGuess(modelCornersCell, camCornersCell, calibInfo);

%% Step 3: Calculate projector nodes (Xp) for proposed method
% 1. Get SL nodes from specified image sets
msg = 'Extracting and saving grid nodes and warped projector corners...';
waitbar(0.2, waitBarHandle, msg);
disp(msg);

%[nodesCell, ~] = Calibration.getNodesAndPrjCorners(calibInfo, camParams, camCorners, verbose, useParallel,useDeepLearningModel);
[nodesCell, ~] = Calibration.getNodesAndPrjCorners(calibInfo, camParams, camCorners, adaptiveOption, hueLabel, verbose, useParallel);

% 2. Save projector points and node pairs
cv.FileStorage(fullfile(calibInfo.path, 'nodePairs.yml'), nodesCell);

%% Step 4. Warp node points (Xc) in camera image to model space (Xm)
msg = 'Warpping nodes to model space (Xc to Xm) for proposed method...';
waitbar(0.3, waitBarHandle, msg);
disp(msg);

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
    % TODO: check undistortion error by fitting line with checkerboard if not
    % good, do not undistort.
    XcUndistort{i} = ImgProc.cvUndistortPoints(Xc{i}, camParams.camK, camParams.camKc);
%     XcUndistort{i} = Xc{i}; 
    
    % transform camera image grid points to white board model space using
    % H, then use calibrated tvecs and rvecs to transform grid points from
    % model space to world space.
    % NOTE: the cb corners have to be undistorted
    curXm = ImgProc.applyHomography(XcUndistort{i}, inv(Hmc));
    curXm = [curXm, zeros(length(curXm), 1)];
    Xm{i} = curXm;
end

%% Step 5. Calibrate camera and projector using the proposed method
msg = 'Performing camera-projector calibration using bundle adjustment (BA)...';
waitbar(0.9, waitBarHandle, msg);
disp(msg);

% 1. Use 'BA' option to specify bundle adjustment on Xm
stereoParams = Calibration.stereoCalibrate(Xm, Xc, Xp, calibInfo.camImgSize, calibInfo.prjImgSize, 'BA', useDistortion);
stereoParams.sets = calibInfo.sets;
stereoParams.dataName = calibInfo.dataName;

% 2. Calculate reprojection errors
stereoParams = Calibration.calcReprojectionError(stereoParams, stereoParams.modelPts, stereoParams.camImgPts, stereoParams.prjImgPts);
app.stereoParams = stereoParams;

%3. Plot calibration results
Calibration.showReprojectionErrors(app.stereoParams, calibInfo);

% debug
Reconstruct.visualizePts3d(cell2mat(stereoParams.worldPts'), stereoParams.R, stereoParams.T, 'ProCams extrinsics and reconstructed SL nodes');

%% Step 6. Save calibration data
% if(useDeepLearningModel ~= 1)
%     calibFileFullName = fullfile(calibInfo.resultDir, ['calibration' , '.yml']);
% else
%     calibFileFullName = fullfile(calibInfo.resultDir, ['calibrationUsingDeepLearningModel' , '.yml']);
% end
if(adaptiveOption == 1)
    calibFileFullName = fullfile(calibInfo.calibResultDir, ['adaptive_Calibration' , '.yml']);
else
    calibFileFullName = fullfile(calibInfo.calibResultDir, ['calibration' , '.yml']);
end

cv.FileStorage(calibFileFullName, stereoParams);
waitbar(1.0, waitBarHandle, 'Calibration data saved');
% fprintf('Calibration data saved to %s\n', calibFileFullName);

%% update app's text area, display results and save to log
app.updateCalibrationResultText(calibFileFullName, stereoParams);

calibInfo
% app.stereoParams
fprintf('%s\n', app.CalibrationResultsTextArea.Value{:})

close(waitBarHandle);
uiconfirm(app.ProCamCalibUIFigure,['Calibration data saved to ', calibFileFullName], 'Calibration complete!', 'Options', {'OK'},'icon','success');
diary off;
end