function [pcOut, meanTranslateParam1, meanTranslateParam2,  inTform] = singleShotReconDenoisePc(calibInfo, stereoParams, dataRoot, dataName, reconId, pcGT, ptCloudSingleShotOutPR, meanTranslateParam1, meanTranslateParam2, option, inTform, alignParam, denoiseThreshold)
%% Denoise single-shot reconstructed point cloud.

%%
% decode gray patterns
if(option(1))
    imageFolder = fullfile(dataRoot, dataName,  'grayCodeRecon', ['Set', sprintf('%02d', str2num(reconId))]);
    imds = imageDatastore(imageFolder, 'LabelSource', 'foldernames', 'IncludeSubfolders',true);

    % SL.decodeGrayPatternsMexMake
    threshold = option(2);
    Reconstruct.decodeGrayPatterns(imds.Files, imageFolder, calibInfo.prjW, calibInfo.prjH, threshold);

    % reconstruction using sl points
    calibReconName = fullfile(dataRoot, dataName, 'grayCodeRecon', ['Set', sprintf('%02d', str2num(reconId))], 'sl.yml');
    slPtsPair = cv.FileStorage(calibReconName);
    ptCloudMT = triangulatePointPairs(stereoParams, slPtsPair);

    % denoise point cloud
    ptCloudMT = pcdenoise(ptCloudMT,'NumNeighbors',100);
    ptCloudMT = pcdenoise(ptCloudMT,'NumNeighbors',50);

    [~, ~, ~, meanTranslateParam1, meanTranslateParam2,  inTform] = Reconstruct.alignPcToGroundTruth(pcGT, ptCloudMT, alignParam);
end

% align point cloud to target point cloud to denoise point cloud
if(option(1) || option(3))
    pt3d = (pcGT.Location).*1000;
    pt3d(:,2) = -pt3d(:,2);
    pt3d(:,3) = -pt3d(:,3);
    pcGroundTruthTemp = pointCloud(pt3d);

    newpcRS = Reconstruct.rotatePC(pcGroundTruthTemp, alignParam(1, 2), alignParam(1, 3), alignParam(1, 4));
    
    originPt3d = ptCloudSingleShotOutPR.Location;
    originPt3d(:,1) = originPt3d(:,1) - meanTranslateParam2(1);
    originPt3d(:,2) = originPt3d(:,2) - meanTranslateParam2(2);
    originPt3d(:,3) = originPt3d(:,3) - meanTranslateParam2(3);
    
    pc = pointCloud(originPt3d);
    
    if(option(3))
        [pcGroundRef] = Reconstruct.translatePcToOriginPoint(newpcRS);
        pc = Reconstruct.translatePcToGroundTruth(pcGroundRef, pc);
    end
   
    pcTemp = pctransform(pc, inTform);
    
    originPt3d = pcTemp.Location;
    originPt3d(:,1) = originPt3d(:,1) + meanTranslateParam1(1);
    originPt3d(:,2) = originPt3d(:,2) + meanTranslateParam1(2);
    originPt3d(:,3) = originPt3d(:,3) + meanTranslateParam1(3);

    pcAligned = pointCloud(originPt3d);
else
    [newpcRS, pcAligned, ~, meanTranslateParam1, meanTranslateParam2,  inTform] = Reconstruct.alignPcToGroundTruth(pcGT, ptCloudSingleShotOutPR, alignParam);
end

[~, vecErr] = knnsearch(newpcRS.Location, pcAligned.Location);

% denoise
idx = find(vecErr <= denoiseThreshold);
% err = sort(vecErr,'descend')
vecErr(find(vecErr > denoiseThreshold));
pcOut = pointCloud(ptCloudSingleShotOutPR.Location(idx, :, :));

%% Compute point cloud using calibrated params and 2d point pairs
function [ptCloud] = triangulatePointPairs(param, ptsYML)

% undistort points
camPtsImg = ImgProc.cvUndistortPoints(ptsYML.camPts2d, param.camK, param.camKc);
prjPtsImg = ImgProc.cvUndistortPoints(ptsYML.prjPts2d, param.prjK, param.prjKc);

% triangulate
[ptCloud, reprjErr] = Reconstruct.pointCloudFromPts2d(camPtsImg, prjPtsImg, param.camK, param.prjK, param.R, param.T);

end
end

