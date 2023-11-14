function imD = pointCloudToDepthMap(camK, camKc, pts3d, imMask)
%% Give point cloud colors

if(isa(pts3d, 'pointCloud'))
    pcloud = pts3d;
    pts3d = pts3d.Location;
else
    pcloud = pointCloud(pts3d);
end

h = size(imMask,1);
w = size(imMask,2);

% project to 2D image
pts3d = double(pcloud.Location);
pts3d = pts3d(pts3d(:,3) > 0,:);
% pts2d = cv.projectPoints(pts3d, [0,0,0], [0,0,0], param.camK, 'DistCoeffs', param.camKc);
pts2d = round(cv.projectPoints(pts3d, [0,0,0], [0,0,0], camK)) + 1; % make sure to +1 to convert OpenCV to matlab coord
inlierIdx = pts2d(:,1)<=w & pts2d(:,2)<=h & pts2d(:,1)>0 & pts2d(:,2)>0;
pts2d = pts2d(inlierIdx,:);
pts3d = pts3d(inlierIdx,:);

% create depth image
imD = zeros(h, w);
imD(sub2ind([h, w], pts2d(:,2), pts2d(:,1))) = pts3d(:,3);
imD = imD.*imMask;
end