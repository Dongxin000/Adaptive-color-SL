%% Compares single-shot reconstruction and merged single-shot-per-pose reconstruction of methods mentioned in the paper.

%% Suffix:
% HBY: Huang method
% DG: degraded proposed method (w/o MAP-based color detection algorithm)
% PR: proposed method

%% Options
% clc
clear
close all
dataRoot = 'data\singeShotReconExp'; 

dataName = 'singleShotRecon_settingA'; % a carefully imaging setting
% dataName = 'singleShotRecon_settingB'; % an extreme imaging setting 

% name of reconstructed object
reconName = 'Box';

% load calibration info
calibInfo = Calibration.loadCalibInfo(fullfile(dataRoot, dataName, 'singleShotRecon'));
calibInfo.adapFolderName = fullfile(dataRoot, dataName, 'adap');
dataPath = calibInfo.path;

% load reconstruction info
reconInfo = Reconstruct.loadSingleShotReconInfo(fullfile(dataRoot, dataName));

if(strcmp('David', reconName))
    singleShotIdx = reconInfo.idxDavid(1);
    start = reconInfo.idxDavid(2);
    finish = reconInfo.idxDavid(3);
    
    downSampleGridSize = 9.5;
    
    % denoise parameters
    denoiseParamPR = reconInfo.denoiseParamDavidPR;
    denoiseParamDG = reconInfo.denoiseParamDavidABL;
    alignParam = reconInfo.alignDavid;
    
    % denoise threshold
    option.t = 15;
elseif(strcmp('Fan', reconName))
    singleShotIdx = reconInfo.idxFan(1);
    start = reconInfo.idxFan(2);
    finish = reconInfo.idxFan(3);   
    
    downSampleGridSize = 8.5;
    
    % denoise parameters
    denoiseParamPR = reconInfo.denoiseParamFanPR;
    denoiseParamDG = reconInfo.denoiseParamFanABL;
    alignParam = reconInfo.alignFan;
    
    % denoise threshold
    option.t = 10;
elseif(strcmp('Box', reconName))
    singleShotIdx = reconInfo.idxBox(1);
    start = reconInfo.idxBox(2);
    finish = reconInfo.idxBox(3);   
    
    downSampleGridSize = 10;
    
    % denoise parameters
    denoiseParamPR = reconInfo.denoiseParamBoxPR;
    denoiseParamDG = reconInfo.denoiseParamBoxABL;
    alignParam = reconInfo.alignBox;
    
    % denoise threshold
    option.t = 5;
end

option.target = reconName;

if(strcmp('singleShotRecon_settingA', dataName))
    % SettingA  image data idx
    imgDataIdx = [1 18 19; 2 20 21; 3 22 23; 4 24 25; 5 26 27; 6 28 29; 7 30 31; 8 32 33; 9 34 35; 10 36 37; 11 38 39; 12 40 41; 13 42 43; 14 44 45; 15 47 46; 16 49 48; 17 51 50; 18 53 52; 19 55 54; 20 57 56; 21 59 58; 22 61 60; 23 63 62; 24 65 64; 25 67 66; 26 69 68; 27 71 70; 28 73 72; 29 75 74];
else
    % SettingB image data idx
    imgDataIdx = [1 17 18; 2 19 20; 3 21 22; 4 23 24; 5 25 26; 6 27 28; 7 29 30; 8 31 32; 9 33 34; 10 35 36; 11 37 38; 12 39 40; 13 42 41; 14 44 43; 15 46 45; 16 48 47; 17 50 49; 18 52 51; 19 54 53; 20 56 55; 21 58 57; 22 60 59; 23 62 61; 24 64 63; 25 66 65; 26 68 67; 27 70 69; 28 72 71; 29 74 73];
end

reconIdxGrayCode = int2str(imgDataIdx(singleShotIdx, 1));
imgIdxHBY = imgDataIdx(singleShotIdx, 2);
imgIdxPR = imgDataIdx(singleShotIdx, 3);

% use denoise option
useDenoise = 0;

% use existing mask option
useExistingMask = 1;

% 
count = 0;

% ground truth point cloud
pcGT = pcread(fullfile('data', 'ipadProRecon', [reconName,'.ply']));

% interpolate option
InterpolateOption = 0;

% get stereo params
stereoParams = cv.FileStorage(fullfile(calibInfo.calibResultDir, ['adaptive_Calibration.yml']));

% get hue label
hueLabel = load(fullfile(dataRoot, dataName, 'adap\hueLabel.mat'));
hueLabel = hueLabel.hueLabel;

% debug option, enable for visuals/figures
verbose = 0;

% use edge pixels option
useEdge = 0;

%
epiThresh = 1;

%% Huang: single-shot reconstruction based huang fixed pattern & Otsu color detection
% the number of pattern's colors
calibInfo.numColor(1) = 4;
calibInfo.numColor(2) = 4;

adaptiveOption = 0;

ptCloudSingleShotOutHuang = Reconstruct.singleShotReconstruction(imgIdxHBY, stereoParams, useExistingMask, useEdge, adaptiveOption, epiThresh, calibInfo, dataPath, dataRoot, dataName, verbose, hueLabel);

%% Proposed: single-shot reconstruction based proposed adaptive pattern & MAP-based detection
% the number of pattern's colors
calibInfo.numColor(1) = 4;
calibInfo.numColor(2) = 5;

adaptiveOption = 1;

ptCloudSingleShotOutPR = Reconstruct.singleShotReconstruction(imgIdxPR, stereoParams, useExistingMask, useEdge, adaptiveOption, epiThresh, calibInfo, dataPath, dataRoot, dataName, verbose, hueLabel);

%%  Proposed w/o MAP: single-shot reconstruction based adaptive pattern & Otsu color detection
calibInfo.numColor(1) = 4;
calibInfo.numColor(2) = 5;

adaptiveOption = 2;

ptCloudSingleShotOutDG = Reconstruct.singleShotReconstruction(imgIdxPR, stereoParams, useExistingMask, useEdge, adaptiveOption, epiThresh, calibInfo, dataPath, dataRoot, dataName, verbose, hueLabel);

%%  Calculate align error
calTypes = {'huang', 'degraded_proposed', 'proposed'};

pcCalib = [];

pcCalib{1} = ptCloudSingleShotOutHuang;
pcCalib{2} = ptCloudSingleShotOutDG;
pcCalib{3} = ptCloudSingleShotOutPR;

singleShotAlignErr = zeros(size(length(calTypes), 1), 7);

maxErr = 50;
cMap = parula(256);
errMap = linspace(0,maxErr,256);

vecErr = [];
for i = 1:3
    [newpcRS, currPcCalib, IcpRmse, ~] = Reconstruct.alignPcToGroundTruth(pcGT, pcCalib{i}, alignParam);

    [~, vecErr{i}] = knnsearch(newpcRS.Location, currPcCalib.Location);
    
    [errColor, errInlierIdx] = Reconstruct.getColorUsingErr(vecErr{i}, maxErr, errMap, cMap);
    
    rmse = vecErr{i};
    singleShotAlignErr(i,1) = length(pcCalib{i}.Location(errInlierIdx,:));
    singleShotAlignErr(i,2) = min(rmse);
    singleShotAlignErr(i,3) = max(rmse);
    singleShotAlignErr(i,4) = mean(rmse);
    singleShotAlignErr(i,5) = median(rmse);
    singleShotAlignErr(i,6) = std(rmse);
    singleShotAlignErr(i,7) = IcpRmse;
    ax(i) = gca;

%     % set color using error
%     [errColor, errInlierIdx] = Reconstruct.getColorUsingErr(vecErr{i}, maxErr, errMap, cMap);
% 
%     % color point cloud
%     pcCalib{i}.Color = errColor;

    % create point cloud figure
%     figure('Name' ,[calTypes{i}]);
%     pcshow(pcCalib{i}.Location(errInlierIdx,:), errColor(errInlierIdx,:,:), MarkerSize = 60)

    % set background color to white
%     set(gca,'color','w');
%     set(gcf,'color','w');
%     set(gca, 'XColor', [1 1 1], 'YColor', [1 1 1], 'ZColor', [1 1 1])
end

% hlink = linkprop(ax,{'CameraPosition','CameraUpVector'});

%% Merge point clouds
for k=start:finish
    close all
    reconIdxGrayCode = int2str(imgDataIdx(k, 1));
    imgIdx = imgDataIdx(k, 3);
    
    count = count+1;

    %%% Proposed
    adaptiveOption = 1;

    meanTranslateParam1 = [];
    meanTranslateParam2 = [];
    inTform = [];
    
    ptCloudSingleShotOutPR = Reconstruct.singleShotReconstruction(imgIdx, stereoParams, useExistingMask, useEdge, adaptiveOption, epiThresh, calibInfo, dataPath, dataRoot, dataName, verbose, hueLabel);
    
    % denoise
    [ptCloudSingleShotOutPR, meanTranslateParam1, meanTranslateParam2,  inTform] = Reconstruct.singleShotReconDenoisePc(calibInfo, stereoParams, dataRoot, dataName, reconIdxGrayCode, pcGT, ptCloudSingleShotOutPR, meanTranslateParam1, meanTranslateParam2, denoiseParamPR(count, :), inTform, alignParam, option.t);

    % save point cloud
    pcFileName = fullfile(calibInfo.reconResultDir, strcat([sprintf('%02d_SingleShot', imgIdx)], '_', 'proposed.ply'));
    pcwrite(ptCloudSingleShotOutPR, pcFileName);
    
    %%% Proposed w/o MAP
    adaptiveOption = 2;
    
    ptCloudSingleShotOutDG = Reconstruct.singleShotReconstruction(imgIdx, stereoParams, useExistingMask, useEdge, adaptiveOption, epiThresh, calibInfo, dataPath, dataRoot, dataName, verbose, hueLabel);
    
    % denoise
    [ptCloudSingleShotOutDG, ~, ~,  ~] = Reconstruct.singleShotReconDenoisePc(calibInfo, stereoParams, dataRoot, dataName, reconIdxGrayCode, pcGT, ptCloudSingleShotOutDG, meanTranslateParam1, meanTranslateParam2, denoiseParamDG(count, :), inTform, alignParam, option.t);

    % save point cloud
    pcFileName = fullfile(calibInfo.reconResultDir, strcat([sprintf('%02d_SingleShot', imgIdx)], '_', 'dg_proposed.ply'));
    pcwrite(ptCloudSingleShotOutDG, pcFileName);
end

mergeAlignErr = zeros(size(2, 1), 7);

disp(['Merging point cloud...']);
mergeAlignErr(1,:) = Reconstruct.mergePC(imgDataIdx, start, finish, pcGT, alignParam, downSampleGridSize, reconName, 'dg_proposed', calibInfo);
close all

disp(['Merging point cloud...']);
mergeAlignErr(2,:) = Reconstruct.mergePC(imgDataIdx, start, finish, pcGT, alignParam, downSampleGridSize, reconName, 'proposed', calibInfo);
close all

%% Summarize error
alignErr = zeros(size(5, 1), 7);

alignErr(1:2, :) = singleShotAlignErr(1:2, :);
alignErr(3, :) = mergeAlignErr(1,:);
alignErr(4, :) = singleShotAlignErr(3, :);
alignErr(5, :) = mergeAlignErr(2,:);

algNames = {'Huang', 'Ours w/o MAP', 'Ours w/o MAP (merged)', 'Ours', 'Ours (merged)'};

% display results
metricsTable = array2table(alignErr,...
    'VariableNames',{'numPts', 'min','max','mean', 'median','std', 'ICPErr'},...
    'RowNames', algNames);
disp('3D reconstructed and RealSense captured point cloud 3D alignment error:'); 
disp(metricsTable);

% save results
writetable(metricsTable, fullfile(calibInfo.reconResultDir, [reconName, '_Merged.xlsx']), "WriteRowNames",true);
