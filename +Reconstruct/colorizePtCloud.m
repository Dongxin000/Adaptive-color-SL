function ptCloudOut = colorizePtCloud(camK, camKc, pts3d, imTexture, imMask)
%% Give point cloud colors
if(isa(pts3d, 'pointCloud'))
    pts3d = pts3d.Location;
end

[h, w, ~] = size(imTexture);
if(nargin < 5)
    imMask = ones(h, w);
end

pts2d = cv.projectPoints(pts3d, zeros(3,1), zeros(3,1), camK, 'DistCoeffs', camKc);
pts2dColor = pts2d;

% remove points that are out of camera's fov
inlierIdx = (pts2d(:,2) > 0) & (pts2d(:,1) > 0) & (pts2d(:,2) < h) & (pts2d(:,1) < w);
pts2dColor(~inlierIdx,:) = 1;
pts2d = pts2d(inlierIdx,:);
pts3d = pts3d(inlierIdx,:);

% only keep masked 2d, 3d points
pixVal = interp2(im2single(imMask), pts2d(:, 1), pts2d(:, 2));
pts2dInliers = pts2d(pixVal>0,:);
pts3dInliers = pts3d(pixVal>0,:);

% convert to single for color interpolation
imTexture = im2single(imTexture);

% assign color to pt3d
r = interp2(imTexture(:,:,1), pts2dInliers(:, 1), pts2dInliers(:, 2));
g = interp2(imTexture(:,:,2), pts2dInliers(:, 1), pts2dInliers(:, 2));
b = interp2(imTexture(:,:,3), pts2dInliers(:, 1), pts2dInliers(:, 2));

% create ptCloud with colors
ptCloudOut = pointCloud(pts3dInliers, 'Color', [r,g,b]);
end