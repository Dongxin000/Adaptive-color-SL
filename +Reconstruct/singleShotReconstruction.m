function [ptCloudSingleShotOut] = singleShotReconstruction(imgIdx, stereoParams, useExistingMask, useEdge, adaptiveOption, epiThresh, calibInfo, dataPath, dataRoot, dataName, verbose, hueLabel)
%% Single shot per pose reconstruction.

%%
% start waitbar
msg = 'Extracting structured light nodes from camera image...';
disp(msg)

%% Step 1: Extract SL points from selected image
imSceneName = fullfile(dataPath, sprintf('scene%02d.png', imgIdx));
imColorName = fullfile(dataPath, sprintf('colorGrid%02d.png', imgIdx));
imPatternName = fullfile(dataPath, 'pattern.png');

currCompHsv = load(fullfile(calibInfo.adapFolderName, 'currCompHsv.mat'));
currCompHsv = currCompHsv.currCompHsv;

% generate color grid pattern if it does not exist
if(~exist(imPatternName, 'file'))
    imwrite(ImgProc.genStructuredLight(calibInfo.prjW, calibInfo.prjH, adaptiveOption, currCompHsv, calibInfo.numColor), imPatternName);
end

%% Find matched nodes
camW = calibInfo.camW;
camH = calibInfo.camH;
prjW = calibInfo.prjW;
prjH = calibInfo.prjH;

imMaskName = fullfile(dataRoot, dataName, 'singleShotRecon', ['objectROI',sprintf('scene%02d.png', imgIdx)]);

if(useExistingMask)
    if(exist(imMaskName, 'file'))
        imMask = imread(imMaskName);
    else
        disp([imMaskName, ' does not exist. Check off Use Existing Mask!'], 'File not found');
        return;
    end
else
    f = figure;
    imshow(imread(imSceneName));title('Draw ROI by clicking and dragging left mouse button')
    h = drawfreehand(f.Children);

    if(isvalid(h))
        imMask = createMask(h);
        cv.imwrite(fullfile(dataRoot, dataName, 'calib', ['objectROI',sprintf('scene%02d.png', imgIdx)]), im2uint8(imMask));
        delete(f);
    end
end

if(verbose)
    fs(imMask);
end

% extract matched color grid Nodes' coords in camera and projector images
msg = 'Matching structured light pattern...';
disp(msg);

[camNodes, prjNodes, Nodes, Edges] = ImgProc.getMatchedNodes(imSceneName, imColorName, [], prjW, prjH, calibInfo.numColor, adaptiveOption, hueLabel, calibInfo.ablParam, verbose);

%% triangulate nodes
msg = 'Triangulating node coordinates...';
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
camNodesUndistort = ImgProc.cvUndistortPoints(camNodes, camK, camKc);
prjNodesUndistort = ImgProc.cvUndistortPoints(prjNodes, prjK, prjKc);

% remove epipolar outliers
[inlierIdx, d] = Reconstruct.findEpipolarInliers(F, camNodesUndistort, prjNodesUndistort, epiThresh, verbose);
camNodesUndistort = camNodesUndistort(inlierIdx,:);
prjNodesUndistort = prjNodesUndistort(inlierIdx,:);

% % draw camera and projector points epipolar lines
if(verbose)
    imCamUndistort = cv.undistort(imread(imColorName), camK, camKc);
    imPrjUndistort = cv.undistort(imread(imPatternName), prjK, prjKc);
    
    % draw epipolar lines
    ImgProc.drawEpipolarLine(F, camNodesUndistort, imCamUndistort, imPrjUndistort);
    ImgProc.drawEpipolarLine(F', prjNodesUndistort, imPrjUndistort, imCamUndistort);
end

% triangulate Node's 3d coordinates
nodePts3d = Reconstruct.triangulatePoints(camK, prjK, R, T, camNodesUndistort, prjNodesUndistort);

% denoise using pcdenoise
% if(useDenoise)
%     pcCalib = pointCloud(nodePts3d);
%     %pcCalib = pcdenoise(pcCalib,'NumNeighbors',10);
%   %  pcCalib = pcdenoise(pcCalib,'NumNeighbors',5);
%     nodePts3d = pcCalib.Location;
% end
% visualize reconstructed 3d Nodes
Reconstruct.visualizePts3d(nodePts3d, R, T, 'Node points only');

%% triangulate Edges

% reconstruct edge points under projector distortion assumption and it works well
if(useEdge)
    msg = 'Triangulating edge coordinates...';
    disp(msg);

    camEdges = [];
    prjEdges = [];
    
    for i=1:numel(Edges)
        curEdge = Edges(i);
        if(numel(curEdge.nodes) ~= 2)
            continue;
        end
        
        node1 = Nodes(curEdge.nodes(1));
        node2 = Nodes(curEdge.nodes(2));
        
        p1 = [node1.activeCol, node1.activeRow];
        p2 = [node2.activeCol, node2.activeRow];
        
        if(any( [p1, p2] < 0))
            continue;
        end
        
        % two nodes's cam coords
        c1 = ImgProc.cvUndistortPoints(node1.Centroid, camK, camKc);
        c2 = ImgProc.cvUndistortPoints(node2.Centroid, camK, camKc);
        
        % two nodes's prj coords, (should be undistorted)
        p1 = ImgProc.cvUndistortPoints(p1, prjK, prjKc);
        p2 = ImgProc.cvUndistortPoints(p2, prjK, prjKc);
        
        % edge points' row and col
        [rows, cols] = ind2sub([camH, camW], curEdge.PixelIdxList);
        
        % linear interpolate edge pixels' projector coord
        for k = 1:length(rows)
            curCamEdgePt2d = [cols(k), rows(k)];
            %             d1 = abs(curCamEdgePt2d - c1);
            %             d2 = abs(curCamEdgePt2d - c2);
            %             s = d1./(d1+d2);
            %             curPrjEdgePt2d = (1-s).*p1 + s.*p2;
            d1 = norm(curCamEdgePt2d - c1);
            d2 = norm(curCamEdgePt2d - c2);
            s = d1/(d1+d2);
            curPrjEdgePt2d = (1-s)*p1 + s*p2;
            curPrjEdgePt2d = curPrjEdgePt2d(1:2); % convert back to non-homogenious
            prjEdges = [prjEdges; curPrjEdgePt2d];
            camEdges = [camEdges; curCamEdgePt2d];
        end
    end
    
    camEdgesUndistort = camEdges;
    prjEdgesUndistort = prjEdges;
    
    % remove epipolar outliers
    [inlierIdx, d] = Reconstruct.findEpipolarInliers(F, camEdgesUndistort, prjEdgesUndistort, epiThresh, verbose);
    camEdgesUndistort = camEdgesUndistort(inlierIdx,:);
    prjEdgesUndistort = prjEdgesUndistort(inlierIdx,:);
    
    % triangulate
    edgePts3d = Reconstruct.triangulatePoints(camK, prjK, R, T, camEdgesUndistort, prjEdgesUndistort);
    
    % visualize
    Reconstruct.visualizePts3d([nodePts3d; edgePts3d], R, T, 'Both Node and Edge points');
end

%% interpolate point cloud
imROI = ImgProc.maskImage(imread(imSceneName), imMask);

ptCloudSingleShotOut = Reconstruct.colorizePtCloud(camK, camKc, nodePts3d, imROI, imMask);

% if(useDenoise)
%       % count1 = ptCloudSingleShotOut.Count
%       ok = pcdenoise(ptCloudSingleShotOut,'NumNeighbors', 20);
%       ptCloudSingleShotOut = pcdenoise(ptCloudSingleShotOut,'NumNeighbors', 60, 'Threshold', 0.1);
%       %ptCloudSingleShotOut = pcdenoise(ptCloudSingleShotOut,'NumNeighbors',3);
%       % count2 = ptCloudSingleShotOut.Count
%       
% %       ok = pcdenoise(ptCloudSingleShotOut,'NumNeighbors',2);
% %       ok = pcdenoise(ok,'NumNeighbors',3);
% %       count1 = ptCloudSingleShotOut.Count
% %       count2 = ok.Count
% %       figure; pcshow(ok, 'MarkerSize', 150)
% %           set(gca,'color','w');
% %         set(gcf,'color','w');
% %     set(gca, 'XColor', [1 1 1], 'YColor', [1 1 1], 'ZColor', [1 1 1])
%     
%       % ptCloudSingleShotOut = pcdenoise(ptCloudSingleShotOut,'NumNeighbors',5);
% end

% if(InterpolateOption)
%     msg = 'Interpolating point cloud...';
%     disp(msg);
% 
%     ptCloudSingleShotOutInterpolate = Reconstruct.interpolatePtCloud(camK, camKc, pointCloud(ptCloudSingleShotOut.Location), imROI, imMask, verbose);
% 
%     % ptCloudInterp = Reconstruct.interpolatePtCloud(camK, camKc, pointCloud(nodePts3d), imROI, imMask, verbose);
%     % ptCloudInterp = Reconstruct.interpolatePtCloud(camK, camKc, pointCloud([nodePts3d; edgePts3d]), imROI, imMask, verbose);
% else
%     ptCloudSingleShotOutInterpolate = ptCloudSingleShotOut;
% end

% % denoise
% [newpcRS, currPcCalib, IcpRmse, ~] = Reconstruct.alignPointCloud(pcRS, ptCloudSingleShotOut, target);
% [~, vecErr] = knnsearch(newpcRS.Location, currPcCalib.Location);
% idx = find(vecErr>=0);
% err = sort(vecErr,'descend')
% vecErr(find(vecErr >5))
% ptCloudSingleShotOut = pointCloud(ptCloudSingleShotOut.Location(idx, :, :));

% pcwrite(ptCloudSingleShotOut, pcFileName);
% pcwrite(ptCloudSingleShotOutInterpolate, pcInterpolateFileName);


% figure; pcshow(nodePts3d); title('Point Cloud');
% ptCould = [];
% ptCould{1} = ptCloudSingleShotOut;
% ptCould{2} = ptCloudSingleShotOut;
% 
% for i =1:1
%     figure;
%     h = pcshow(ptCould{i}, 'MarkerSize', 150);
%     title('Interpolated colored point cloud')
% 
%     % set background color to white
%     set(gca,'color','w');
%     set(gcf,'color','w');
%     set(gca, 'XColor', [1 1 1], 'YColor', [1 1 1], 'ZColor', [1 1 1])
%     ax(i) = gca;
% end
% 
% hlink = linkprop(ax,{'CameraPosition','CameraUpVector'});

% [~, ~, ~, pcInlierIdx] = Reconstruct.filterPointCloud(ptCloudSingleShotOut, imMask, stereoParams, camW, camH);
% pt3d = ptCloudSingleShotOut.Location;
% newPt3d = pt3d(pcInlierIdx,:,:);
% ptCloudSingleShotOut = pointCloud(newPt3d);
end

