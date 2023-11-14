function [stereoParamsPR, modelCornersCell, camCornersCell, camParams] = singleShotCalibration(calibInfo, adaptiveOption, verbose)
%% single-shot calibration, more details please refer to B. Huang, Y. Tang, S. Ozdemir, and H. Ling. A fast and flexible projector-camera calibration system. IEEE Transactions on Automation Science and Engineering, 2020.

%%
% folder where extracted checkerboard corners are stored
cornerDir = fullfile(calibInfo.path, 'matlabCorners');

% debug option, enable for visuals/figures
%verbose = 1;

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

% find opencv checkerboard corners and matlab checkerboard corners matches (for MT method)
% imLightNames = ImgProc.getImageNames(calibInfo.path, 'light');
% imLightNames = imLightNames(calibInfo.sets);
% [cornersCV, ret] = cv.findChessboardCorners(rgb2gray(imread(imLightNames{1})), calibInfo.boardSize-1);
% cornersCV = cell2mat(cornersCV');
% [idx, err] = knnsearch(cornersCV, camCorners(:,:,1));


%% Step 2: Calibrate camera, (skip 1 if using existing camera corners)
disp(' ');
disp('Calibrating camera using checkerboard corners...');

% 1. Generate world corners
modelCornersCell = Calibration.generateModelCorners(calibInfo);

% 2. Convert camCorners to (mex)OpenCV format
camCornersCell = squeeze(mat2cell(camCorners, calibInfo.numCorners, 2, ones(1, calibInfo.numSets)))';

% 3. Calibrate camera, only used to warp nodes to model space (not init guess)
camParams = Calibration.calibrateInitGuess(modelCornersCell, camCornersCell, calibInfo);

%% Step 3: Calculate projector points for global homography and nodes for proposed method
disp(' ');
disp('Extracting and saving grid nodes and warped projector corners...');

if (~useExistingPrjCorners)
    % 1. Get projector corners using matching & homography
    [nodesCell, prjCornersGH] = Calibration.getNodesAndPrjCorners(calibInfo, camParams, camCorners, adaptiveOption, verbose);
    
    % 2. Save projector points and node pairs
    Calibration.saveCorners(prjCornersGH, 'proj_', cornerDir, calibInfo.sets, ones(calibInfo.numSets,1));
    cv.FileStorage(fullfile(calibInfo.path, 'nodePairs.yml'), nodesCell);
else
    disp('Reading existing warped projector corners and nodes...');
    prjCornersGH = Calibration.readCorners('proj_', cornerDir, calibInfo);
    nodesCell = cv.FileStorage(fullfile(calibInfo.path, 'nodePairs.yml')).nodePairs;
end

% %% Step 4: Read Moreno & Taubin warped checkerboard corners and SL node coordinates
% disp(' ');
% disp('Reading existing Moreno & Taubin warped projector corners and nodes...');
% 
% % 1. read Moreno & Taubin checkerboard corners warped by local
% % homographies. (MT stands for their method related data)
% cornerDirMT = fullfile(calibInfo.path, 'MT');
% prjCornersMT = Calibration.readCorners('proj_', cornerDirMT, calibInfo);
% 
% % change order to match matlab checkerboard corner orders
% % prjCornersMT = prjCornersMT(idx,:,:);
% 
% if (verbose)
%     for i = 1:calibInfo.numSets
%         figure;
%         hold on;
%         plot(prjCornersGH(:, 1, i), prjCornersGH(:, 2, i), 'ro'); hold on
%         plot(prjCornersMT(:, 1, i), prjCornersMT(:, 2, i), 'b+');
%         title(['Set ', num2str(calibInfo.sets(i)),...
%             ': warped checkerboard corners in projector image, red is global homography, blue is Moreno & Taubin' ]);
%         hold off;
%    end
% end

%% debug
% camCornersMT = Calibration.readCorners('cam_', cornerDirMT, calibInfo);
% i=1;
% figure;
% for j = 1:size(camCorners,1)
%     plot(camCorners(j, 1, i), camCorners(j, 2, i), 'ro'); text(camCorners(j, 1, i), camCorners(j, 2, i),num2str(j)); hold on
%     plot(camCornersMT(j, 1, i), camCornersMT(j, 2, i), 'b+');text(camCornersMT(j, 1, i), camCornersMT(j, 2, i),num2str(j));
% end
% 
% figure;
% tst =  camCorners(idx,:,:);
% for j = 1:size(camCorners,1)
%     plot(tst(j, 1, i), tst(j, 2, i), 'ro'); text(tst(j, 1, i), tst(j, 2, i),num2str(j)); hold on
%     plot(camCornersMT(j, 1, i), camCornersMT(j, 2, i), 'b+');text(camCornersMT(j, 1, i), camCornersMT(j, 2, i),num2str(j));
% end


%% Step 4. Warp node points (Xc) in camera image to model space (Xm)
disp(' ');
disp('Warpping nodes to model space (Xc to Xm) for proposed method...');
% refer to paper to understand Xm, Xc, Xp
Xm = []; % node points in model space
Xc = []; % node points in camera image space
Xp = []; % node points in projector image space

% 1. Undistorted node points in camera image space
xcUndistort = cell(1, calibInfo.numSets);

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
    xcUndistort{i} = ImgProc.cvUndistortPoints(Xc{i}, camParams.camK, camParams.camKc);
    
    % transform camera image grid points to white board model space using
    % H, then use calibrated tvecs and rvecs to transform grid points from
    % model space to world space.
    % NOTE: the cb corners have to be undistorted
    curXm = ImgProc.applyHomography(xcUndistort{i}, inv(Hmc));
    curXm = [curXm, zeros(length(curXm), 1)];
    Xm{i} = curXm;
    
    % random sample points to reduce computation and ram
%     k = 500;
%     if(length(Xm{i}) > k)
%         idx = randsample(length(Xm{i}), k);
%         Xm{i} = Xm{i}(idx,:);
%         Xc{i} = Xc{i}(idx,:);
%         Xp{i} = Xp{i}(idx,:);
%     end
    
end


%% Step 5. Calibrate camera and projector using the proposed method (PR)
disp(' ');
disp('[Proposed] Performing camera-projector calibration using BA...');

% 1. Use 'BA' option to specify bundle adjustment on Xm
stereoParamsPR = Calibration.stereoCalibrate(Xm, Xc, Xp, calibInfo.camImgSize, calibInfo.prjImgSize, 'BA');
stereoParamsPR.sets = calibInfo.sets;
stereoParamsPR.dataName = calibInfo.dataName;

% 2. Calculate reprojection errors
stereoParamsPR = Calibration.calcReprojectionError(stereoParamsPR, stereoParamsPR.modelPts, stereoParamsPR.camImgPts, stereoParamsPR.prjImgPts);

% 3. Display calibration results
stereoParamsPR

% debug
if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParamsPR.worldPts'), stereoParamsPR.R, stereoParamsPR.T, 'Proposed calibration: Xm ');
end
end
