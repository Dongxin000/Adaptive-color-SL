function imN = pointCloudToNormalMap(camK, camKc, pts3d, imMask)
%% Give point cloud colors

if(isa(pts3d, 'pointCloud'))
    pcloud = pts3d;
    pts3d = pts3d.Location;
else
    pcloud = pointCloud(pts3d);
end

h = size(imMask,1);
w = size(imMask,2);

n = pcnormals(pcloud);
pts2d = cv.projectPoints(pts3d, zeros(3,1), zeros(3,1), camK, 'DistCoeffs', camKc);

% remove points that are out of camera's fov
inlierIdx = (pts2d(:,2) > 0) & (pts2d(:,1) > 0) & (pts2d(:,2) < h) & (pts2d(:,1) < w);
pts2d = pts2d(inlierIdx,:);
% pts3d = pts3d(inlierIdx,:);
n = n(inlierIdx,:);

% grid points for normal map
[X, Y] = meshgrid(1:w, 1:h); % projector coordinates (dense)
nx = griddata(pts2d(:,1), pts2d(:,2), n(:,1), X, Y);
ny = griddata(pts2d(:,1), pts2d(:,2), n(:,2), X, Y);
nz = griddata(pts2d(:,1), pts2d(:,2), n(:,3), X, Y);

imN = cat(3,nx, ny, nz);
imN = imN * 0.5 + 0.5; % for vis
imN = imN.*imMask;

end