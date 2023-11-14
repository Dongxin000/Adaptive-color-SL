%% Compares calibration and reconstruction under different imaging settings and ambient conditions of methods mentioned in the paper.

%% Suffix:
% MT: Moreno & Taubin method
% HBY: Huang method
% DG: degraded proposed method (w/o MAP-based color detection algorithm)
% PR: proposed method
% e.g., stereoParamsPR: stereo parameters obtained using proposed method.

%% Options
% clc;
clear; close all

verbose = 0;

dataRoot = 'data\calibAndReconExp'; 

%%% different imaging settings
dataName = 'Setting1'; % Setting1
% dataName = 'Setting2';% Setting2
% dataName = 'Setting3'; % Setting3
% dataName = 'Setting4'; % Setting4
% dataName = 'Setting5'; % Setting5
% dataName = 'Setting6'; % Setting6
% dataName = 'Setting7'; % Setting7

%%% different ambient light conditions
% dataName = 'Light1'; % Light1
% dataName = 'Light2'; % Light2 
% dataName = 'Light3; % Light3
% dataName = 'Light4'; % Light3
% dataName = 'Light5'; % Light5
% dataName = 'Light6'; % Light6

% load calibration info
calibInfo = Calibration.loadCalibInfo(fullfile(dataRoot, dataName));
calibInfo.dataName = dataName;
calibInfo.adapFolderName = fullfile(dataRoot, dataName, 'adap');

% load the color label for 256-bin hue
hueLabel = load(fullfile(dataRoot, dataName, 'adap\hueLabel.mat'));
hueLabel = hueLabel.hueLabel;

%% Calibrate Huang's method
% set path
calibInfo.path = fullfile(dataRoot, dataName, 'Huang');

% set adaptive option: 0 represents Huang method, e.g., fixed color SL pattern & Otsu color detection
adaptiveOption = 0;

% single-shot calibrate
[stereoParamsHBY, modelCornersCell] = Calibration.singleShotCalibration(calibInfo, adaptiveOption, 'Huang', hueLabel, verbose);

% calculate color detection accuracy 
stereoParamsHBY.colorDetectionAccuracy = AdaptiveGenPattern.calColorDetectionAccuracy(calibInfo, 'huang');

%% Calibrate Proposed method w/o MAP-based color detection algorithm (DG for short)
% set path
calibInfo.path = fullfile(dataRoot, dataName, 'Proposed');

% set adaptive option: 2 represents not using the MAP-based color detection algorithm, e.g., adaptive color SL pattern & otsu color detection
adaptiveOption = 2;

% single shot calibrate
[stereoParamsDG, ~] = Calibration.singleShotCalibration(calibInfo, adaptiveOption, 'Proposed w/o MAP', hueLabel, verbose);

% calculate color detection accuracy 
stereoParamsDG.colorDetectionAccuracy = AdaptiveGenPattern.calColorDetectionAccuracy(calibInfo, 'degraded_proposed');

%% Calibrate Proposed method
% set path
calibInfo.path = fullfile(dataRoot, dataName, 'Proposed');

% set adaptive option: 1 represents proposed method, e.g., adaptive color SL pattern & MAP-based color detection algorithm
adaptiveOption = 1;

% single shot calibrate
[stereoParamsPR, ~] = Calibration.singleShotCalibration(calibInfo, adaptiveOption, 'Proposed', hueLabel, verbose);

% calculate color detection accuracy 
stereoParamsPR.colorDetectionAccuracy = AdaptiveGenPattern.calColorDetectionAccuracy(calibInfo, 'proposed');

%% Calibrate Moreno & Taubin (MT for short)
% Read Moreno & Taubin warped checkerboard corners and SL node coordinates
calibInfo.path = fullfile(dataRoot, dataName);

disp(' ');
disp('Reading existing Moreno & Taubin warped projector corners and nodes...');

% 1. read Moreno & Taubin checkerboard corners warped by local  
% homographies. (MT stands for their method related data)
cornerDirMT = fullfile(calibInfo.path, 'MT');

camCornersMT = Calibration.readCorners('cam_', cornerDirMT, calibInfo);
camCornersMT = ImgProc.correctMTCornersToMatlab(camCornersMT, calibInfo.boardSize);

prjCornersMT = Calibration.readCorners('proj_', cornerDirMT, calibInfo);
prjCornersMT = ImgProc.correctMTCornersToMatlab(prjCornersMT, calibInfo.boardSize);
% change order to match matlab checkerboard corner orders
% prjCornersMT = prjCornersMT(idx,:,:);

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

% calibrate
disp(' ');
disp('[Moreno & Taubin] Performing camera-projector calibration using Moreno & Taubin method...');

%1. Calibrate camera, only used to warp nodes to model space (not init guess)
camCornersCellMT = squeeze(mat2cell(camCornersMT, calibInfo.numCorners, 2, ones(1, calibInfo.numSets)))';
camParamsMT = Calibration.calibrateInitGuess(modelCornersCell, camCornersCellMT, calibInfo);

% 2. Calibrate projector using local homography warped checkerboard corners
prjCornersCellMT = squeeze(mat2cell(prjCornersMT, calibInfo.numCorners, 2, ones(1, calibInfo.numSets)))';
prjParamsMT = Calibration.calibrateInitGuess(modelCornersCell, prjCornersCellMT, calibInfo);

% 3. Stereo calibration using OpenCV(Mod)
stereoParamsMT = Calibration.calibrateStereoInitGuess(modelCornersCell, camCornersCellMT, prjCornersCellMT, camParamsMT, prjParamsMT, calibInfo);
stereoParamsMT.sets = calibInfo.sets;
stereoParamsMT.dataName = calibInfo.dataName;
stereoParamsMT.isValid = 1;

% 4. Calculate reprojection errors
stereoParamsMT = Calibration.calcReprojectionError(stereoParamsMT, modelCornersCell, camCornersCellMT, prjCornersCellMT);

% 5. display calibration results
% stereoParamsMT

% debug
% if (verbose)
%     Reconstruct.visualizePts3d(cell2mat(stereoParamsMT.worldPts'), stereoParamsMT.R, stereoParamsMT.T, 'Moreno & Taubin calibration: Xm ');
% end


%%  Save all calibration data
calTypes = {'moreno_taubin', 'huang', 'degraded_proposed', 'proposed'};

cv.FileStorage(fullfile(calibInfo.calibResultDir, [calTypes{1}, '.yml']), stereoParamsMT);
cv.FileStorage(fullfile(calibInfo.calibResultDir, [calTypes{2}, '.yml']), stereoParamsHBY);
cv.FileStorage(fullfile(calibInfo.calibResultDir, [calTypes{3}, '.yml']), stereoParamsDG);
cv.FileStorage(fullfile(calibInfo.calibResultDir, [calTypes{4}, '.yml']), stereoParamsPR);

disp(' ');
disp(['Calibration data saved to ', calibInfo.calibResultDir]);

%% Compare calibration reprojection errors
algNames = {'Moreno & Taubin', 'Huang', 'Proposed w/o MAP', 'Proposed'};
reprojErrs = zeros(length(calTypes), 3);

for i=1:length(calTypes)
    curCalType = calTypes{i};
    param = cv.FileStorage(fullfile(calibInfo.calibResultDir, [curCalType, '.yml']));
    
    % calibration reprojection error
    reprojErrs(i,1) = param.camReprojErr;
    reprojErrs(i,2) = param.prjReprojErr;
    reprojErrs(i,3) = param.stereoReprojErr;
end

colorAndSegInfo = zeros(2, 3);

for i=2:length(calTypes)
    curCalType = calTypes{i};
    param = cv.FileStorage(fullfile(calibInfo.calibResultDir, [curCalType, '.yml']));
    
    colorAndSegInfo(i-1,1) = param.colorDetectionAccuracy;
    colorAndSegInfo(i-1,2) = param.numNodes(1);
    colorAndSegInfo(i-1,3) = param.numNodes(2);
end

% display color detection ,deteced Nodes and decoded nodes
colorAndSegInfoTable = array2table(colorAndSegInfo,...
    'VariableNames',{'color detection accuracy', 'extracted nodes','decoded nodes'},...
    'RowNames', algNames(2:4));

disp(' ');
disp('Color detection accuracy, extracted and decoded nodes:'); 
disp(colorAndSegInfoTable);

% reprojection error
calibTable = array2table(reprojErrs,...
    'VariableNames',{'camera','projector','stereo'},...
    'RowNames', algNames);

disp(' ');
disp('Calibration reprojection errors:'); 
disp(calibTable);

% save results
writetable(calibTable, fullfile(calibInfo.calibResultDir, ['calibResult.xlsx']), "WriteRowNames",true);
writetable(colorAndSegInfoTable, fullfile(calibInfo.calibResultDir, ['colorAndSegResult.xlsx']), "WriteRowNames",true);

%% Compare 3D reconstruction
% reconstruct different objects
reconNames=strvcat('David', 'Girl', 'Box');
for i=1:3
    % load reconstruction info
    reconInfo = Reconstruct.loadGrayCodeReconInfo(fullfile(dataRoot, dataName));
    reconName = strip(reconNames(i,:));

    disp(['Reconstructing ', reconName, '...']);
    
    % set recon id
    if(strcmp('David', reconName))
        reconSet = num2str(reconInfo.DavidReconId);
    elseif(strcmp('Girl', reconName))
        reconSet = num2str(reconInfo.GirlReconId);
    elseif(strcmp('Box', reconName))
        reconSet = num2str(reconInfo.BoxReconId);
    end
    
    % reconSet = '39'; % no need to write as %02d format
    linkedFigs = Calibration.compareIpadProData(reconSet, calTypes, algNames, reconName, calibInfo, reconInfo);
    close
    %  close all
end
