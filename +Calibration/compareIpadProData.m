function [hlink] = compareIpadProData(reconId, calTypes, algNames, reconName, calibInfo, reconInfo)
%% Compare reconstructed point cloud with iPad Pro captured point cloud.  

%%

%%
% disp(' ');
disp('Comparing reconstrcuted 3D object with ipad pro captured 3D object, please wait...')

%% load gray-coded SL data
imageFolder = fullfile(calibInfo.path, 'MT', ['Set', sprintf('%02d', str2num(reconId))]);
imds = imageDatastore(imageFolder, 'LabelSource', 'foldernames', 'IncludeSubfolders',true);

% set threshold
if(strcmp('David', reconName))
    threshold = reconInfo.threshDavid;
    alignParam = reconInfo.alignDavid;
elseif(strcmp('Girl', reconName))
    threshold = reconInfo.threshGirl;
    alignParam = reconInfo.alignGirl;
elseif(strcmp('Box', reconName))
    threshold = reconInfo.threshBox;
    alignParam = reconInfo.alignBox;
elseif(strcmp('Fan', reconName))
    threshold = reconInfo.threshFan;
    alignParam = reconInfo.alignFan;
end

% decode gray-coded SL data
Reconstruct.decodeGrayPatterns(imds.Files, imageFolder, calibInfo.prjW, calibInfo.prjH, threshold);

%% Load groudtruth data
ipadProReconName = fullfile('data', 'ipadProRecon', [reconName, '.ply']);
calibReconName = fullfile(calibInfo.path, 'MT', ['Set', sprintf('%02d', str2num(reconId))], 'sl.yml');

% ipadPro reconstructed point cloud
if(exist(ipadProReconName, 'file'))
    pcGT = pcread(ipadProReconName);
else
    error(['File ', ipadProReconName, ' does not exist!'])
end

%% Load calibration data and SL point pairs
alignErr = zeros(size(calTypes, 1), 5);

% cam and prj SL point pairs generated by Moreno & Taubin software
if(exist(calibReconName, 'file'))
    slPtsPair = cv.FileStorage(calibReconName);
else
    error(['File ', calibReconName, ' does not exist!'])
end

% maxErr = 50;
% cMap = parula(256);
% errMap = linspace(0, maxErr, 256);

pcCalib = [];
for i=1:length(calTypes)    
    curCalType = calTypes{i};
    param = cv.FileStorage(fullfile(calibInfo.calibResultDir, [curCalType, '.yml']));
    
    if(param.isValid)
        % reconstruct
        currPcCalib = triangulatePointPairs(param, slPtsPair);

        % denoise point cloud
        currPcCalib = Reconstruct.denoisePointCloud(currPcCalib, param, calibInfo.camW, calibInfo.camH);
        currPcCalib = pcdenoise(currPcCalib,'NumNeighbors',100);
        pcCalib{i} = pcdenoise(currPcCalib,'NumNeighbors',50);

        % align reconstructed point cloud to groundtruth using ICP
        try
            % align reconstructed point cloud to ground truth point cloud
            % using icp
            [newpcGT, pcCalib{i}, IcpRmse, meanTranslateParam1, meanTranslateParam2, tform] = Reconstruct.alignPcToGroundTruth(pcGT, pcCalib{i}, alignParam);

            % calculate error between reconstructed point cloud with ground
            % truth point cloud
            [~, vecErr{i}] = knnsearch(newpcGT.Location, pcCalib{i}.Location);
            
            % Reconstruct.visualizePointCloud(pcCalib{i}, newpcRS, param, calibInfo.camW, calibInfo.camH, 1, algNames{i}, meanTranslateParam1, meanTranslateParam2, tform, alignParam);
        catch
            disp([reconName, ' Alignment error!']);
            vecErr{i} = inf;
            IcpRmse = inf;
        end
    else
        disp([reconName, ' alignment error!']);
        vecErr{i} = inf;
        IcpRmse = inf;
    end
    
    % get different metrics
    rmse = vecErr{i};
    alignErr(i,1) = min(rmse);
    alignErr(i,2) = max(rmse);
    alignErr(i,3) = mean(rmse);
    alignErr(i,4) = median(rmse);
    alignErr(i,5) = std(rmse);
    alignErr(i,6) = IcpRmse ;
    ax(i) = gca;
end

% link the plots
% hlink = linkprop(ax,{'CameraPosition','CameraUpVector', 'xlim', 'ylim', 'zlim'});
hlink = linkprop(ax,{'CameraPosition','CameraUpVector'});

%% Display results in table

% 3d alignment error
metricsTable = array2table(alignErr,...
    'VariableNames',{'min','max','mean', 'median','std', 'ICPErr'},...
    'RowNames', algNames);
disp('3D reconstructed and RealSense captured point cloud 3D alignment error:'); 
disp(metricsTable);

writetable(metricsTable, fullfile(calibInfo.reconResultDir, [sprintf('%02d', str2num(reconId)), '_ErrAddAbl_final.xlsx']), "WriteRowNames",true);

% save point cloud
% for i =1:4
%     pcwrite(pcCalib{i}, fullfile(calibInfo.reconResultDir, [sprintf('%02d', str2num(reconId)),'_',calTypes{i}, '.ply']));
% end

%% Export figures
% save figures
% SAVE_FIG = 0;
% 
% n = 1;
% figIds = findobj('Type', 'figure');
% % sort according to number
% [val, idx] = sort([figIds.Number]);
% figIds = figIds(idx);
% 
% if(SAVE_FIG)
% 
%     % export as png format
%     exportPath = 'doc\recon\';
%     for i=1:length(figIds)
%         curFig = figure(figIds(i));
%         figName = strrep(curFig.Name, '/', ' ');
%         figName = [num2str(n),  '_', strrep(figName, ' ', '_')];
%         savefig(curFig, [exportPath, figName, '.fig']);
%         export_fig(curFig, [exportPath, figName, '.png'], '-transparent'); % you need to install export_fig package
%         n = n + 1;
%     end
% end

end

%% Compute point cloud using calibrated params and 2d point pairs
function [ptCloud] = triangulatePointPairs(param, ptsYML)

% undistort points
camPtsImg = ImgProc.cvUndistortPoints(ptsYML.camPts2d, param.camK, param.camKc);
prjPtsImg = ImgProc.cvUndistortPoints(ptsYML.prjPts2d, param.prjK, param.prjKc);

% triangulate
[ptCloud, reprjErr] = Reconstruct.pointCloudFromPts2d(camPtsImg, prjPtsImg, param.camK, param.prjK, param.R, param.T);
end

