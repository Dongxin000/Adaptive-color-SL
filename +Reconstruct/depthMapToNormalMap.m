function imN = depthMapToNormalMap(camK, depthMap, imMask)
%% Compute normal from depth map, need to backproject depth to 3d then normal = cross(dzdx, dzdy)
h = size(imMask,1);
w = size(imMask,2);

[x, y] = meshgrid(1:w, 1:h);
pts3d = (inv(camK) * cv.convertPointsToHomogeneous([x(:), y(:)])' .* depthMap(:)')';
imPts3d = reshape(pts3d, h, w, 3);
[gx, gy, gz] = imgradientxyz(imPts3d, 'central');
gx = normalize(gx, 3, 'norm', 2);
gy = normalize(gy, 3, 'norm', 2);

% normal map
imN = cross(gx, gy);
imN(isnan(imN)) = 0;
imN(:,:, 1) = -imN(:,:, 1); % flip x
imN(:,:, 2) = -imN(:,:, 2); % flip y
imN = imN*0.5+0.5; % vis
imN = imN.*imMask;
end