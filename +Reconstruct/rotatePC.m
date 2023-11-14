function [ptCloudOut] = rotatePC(pc, ax, ay, az)
%% Rotate point cloud.

%%
angleToRad = pi/180;
ax = ax * angleToRad;
ay = ay * angleToRad;
az = az * angleToRad;

rx = [1      0          0; ...
      0     cos(ax)    -sin(ax); ...
     -0     sin(ax)    cos(ax) ];

ry = [cos(ay)       0   sin(ay); ...
        0           1    0; ...
      -sin(ay)      0   cos(ay) ];

rz = [cos(az)      0   sin(az); ...
        0          1    0; ...
     -sin(az)      0   cos(az) ];

rot = rx*ry*rz;
trans = [0     0    0];

tform = rigid3d(rot,trans);
ptCloudOut = pctransform(pc,tform);
end

