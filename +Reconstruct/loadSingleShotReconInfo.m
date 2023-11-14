function [reconInfo] = loadSingleShotReconInfo(reconPath)
%% Load single-shot reconstruction information.

%%

% load calibration info
reconInfo = cv.FileStorage(fullfile(reconPath, 'recon-info.yml'));

% alignment paramter for David, girl and box
reconInfo.alignDavid = cell2mat(reconInfo.alignDavid);
reconInfo.alignFan = cell2mat(reconInfo.alignFan);
reconInfo.alignBox = cell2mat(reconInfo.alignBox);

% get idx
reconInfo.idxDavid = cell2mat(reconInfo.idxDavid);
reconInfo.idxFan = cell2mat(reconInfo.idxFan);
reconInfo.idxBox = cell2mat(reconInfo.idxBox);

% denoise param for Proposed
reconInfo.denoiseParamDavidPR = cell2mat(reconInfo.denoiseParamDavidPR);
reconInfo.denoiseParamDavidPR = reshape(reconInfo.denoiseParamDavidPR, 3, numel(reconInfo.denoiseParamDavidPR)/3)';

reconInfo.denoiseParamFanPR = cell2mat(reconInfo.denoiseParamFanPR);
reconInfo.denoiseParamFanPR = reshape(reconInfo.denoiseParamFanPR, 3, numel(reconInfo.denoiseParamFanPR)/3)';

reconInfo.denoiseParamBoxPR = cell2mat(reconInfo.denoiseParamBoxPR);
reconInfo.denoiseParamBoxPR = reshape(reconInfo.denoiseParamBoxPR, 3, numel(reconInfo.denoiseParamBoxPR)/3)';

% denoise param for Proposed w/o MAP
reconInfo.denoiseParamDavidABL = cell2mat(reconInfo.denoiseParamDavidABL);
reconInfo.denoiseParamDavidABL = reshape(reconInfo.denoiseParamDavidABL, 3, numel(reconInfo.denoiseParamDavidABL)/3)';

reconInfo.denoiseParamFanABL = cell2mat(reconInfo.denoiseParamFanABL);
reconInfo.denoiseParamFanABL = reshape(reconInfo.denoiseParamFanABL, 3, numel(reconInfo.denoiseParamFanABL)/3)';

reconInfo.denoiseParamBoxABL = cell2mat(reconInfo.denoiseParamBoxABL);
reconInfo.denoiseParamBoxABL = reshape(reconInfo.denoiseParamBoxABL, 3, numel(reconInfo.denoiseParamBoxABL)/3)';

end

