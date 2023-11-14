function [reconInfo] = loadGrayCodeReconInfo(reconPath)
%% Load gray code SL reconstruction information.

%%

% load calibration info
reconInfo = cv.FileStorage(fullfile(reconPath, 'recon-info.yml'));

% id
reconId = cell2mat(reconInfo.reconId);
reconInfo.DavidReconId = reconId(1);
reconInfo.GirlReconId = reconId(2);
reconInfo.BoxReconId = reconId(3);

% alignment paramter for David, girl and box
reconInfo.alignDavid = cell2mat(reconInfo.alignDavid);
reconInfo.alignGirl = cell2mat(reconInfo.alignGirl);
reconInfo.alignBox = cell2mat(reconInfo.alignBox);
end

