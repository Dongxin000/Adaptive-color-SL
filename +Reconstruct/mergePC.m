function [alignErr] = mergePC(idx, start, finish, pcGT, alignParam, DownSampleGridSize, reconName, flag, calibInfo)
%% Merge point clouds from different views.

%% Merge PC
j = idx(start, 3);
pcGroundRef = pcread(fullfile(calibInfo.reconResultDir, strcat(num2str(j), ['_SingleShot_', flag, '.ply' ])));

[~, pcGroundRef, ~, ~, ~, ~] = Reconstruct.alignPcToGroundTruth(pcGT, pcGroundRef, alignParam);

for k=start+1:finish
    j = idx(k, 3);
    pcGroundCurrent = pcread(fullfile(calibInfo.reconResultDir, strcat(num2str(j), ['_SingleShot_', flag, '.ply'])));
    [~, pcGroundCurrent, ~, ~, ~, ~] = Reconstruct.alignPcToGroundTruth(pcGT, pcGroundCurrent, alignParam);
    
    gridSize = 0.01;

    fixed = pcdownsample(pcdenoise(pcGroundRef,'Threshold',5), 'gridAverage', gridSize);
    moving = pcdownsample(pcdenoise(pcGroundCurrent,'Threshold',5),  'gridAverage', gridSize);

    % register pcCalib to fixed pcGroundTruth
    [tform, ~, rmse] = pcregistericp(moving, fixed,'Extrapolate', true, 'MaxIterations', 100);

    % disp(['ICP root mean square error: ',num2str(k),'___', num2str(rmse)]);

    % align current to ref
    pcCalibAligned = pctransform(pcGroundCurrent, tform);
    
    mergeSize = 0.001;
    pcGroundRef = pcmerge(pcGroundRef, pcCalibAligned,mergeSize);
end


%% Downsample
pcPRMerge = pcdownsample(pcGroundRef, 'gridAverage', DownSampleGridSize);

c = repmat(im2uint8([0,0,1]),pcPRMerge.Count,1);
pcPRMerge.Color = c;

figure
pcshow(pcPRMerge,MarkerSize = 60)
% set background color to white
set(gca,'color','w');
set(gcf,'color','w');
set(gca, 'XColor', [1 1 1], 'YColor', [1 1 1], 'ZColor', [1 1 1])

pcwrite(pcPRMerge,fullfile(calibInfo.reconResultDir, [reconName, '_pcPRMerge_downsample_' , flag, '.ply']));

%% Calculate Error
pcCalib{1} = pcPRMerge;

calTypes = {'Proposed'};

alignErr = zeros(size(length(2), 1), 6);

maxErr = 10;
cMap = parula(256);
errMap = linspace(0,maxErr,256);

vecErr = [];
for i = 1:1
    [newpcRS, currPcCalib, IcpRmse, ~] = Reconstruct.alignPcToGroundTruth(pcGT, pcCalib{i}, alignParam);
    
    [~, vecErr{i}] = knnsearch(newpcRS.Location, currPcCalib.Location);
    
    % set color using error
    [errColor, errInlierIdx] = Reconstruct.getColorUsingErr(vecErr{i}, maxErr, errMap, cMap);
    
    % color point cloud
    % currPcCalib.Color = errColor;
    
    % currPcCalib = Reconstruct.rotatePC(currPcCalib, 0,20, 0);
    % create point cloud figure
    figure('Name' ,[calTypes{i}]);
    
    pcshow(currPcCalib.Location(errInlierIdx,:), errColor(errInlierIdx,:,:))
    
    % set background color to white
    set(gca,'color','w');
    set(gcf,'color','w');
    set(gca, 'XColor', [1 1 1], 'YColor', [1 1 1], 'ZColor', [1 1 1])
    
    rmse = vecErr{i};
    alignErr(i,1) = length(pcCalib{i}.Location(errInlierIdx,:));
    alignErr(i,2) = min(rmse);
    alignErr(i,3) = max(rmse);
    alignErr(i,4) = mean(rmse);
    alignErr(i,5) = median(rmse);
    alignErr(i,6) = std(rmse);
    alignErr(i,7) = IcpRmse;
    ax(i) = gca;
end

hlink = linkprop(ax,{'CameraPosition','CameraUpVector'});

end

