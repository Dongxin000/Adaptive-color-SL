function [newCornersMT] = correctMTCornersToMatlab(CornersMT, boardsize)
%% Change the order of storing corner points from OpenCV to the order used in MATLAB.

%%
T = boardsize(1)-1;

for i= 1:size(CornersMT,3)
    for j=1:boardsize(2)-1
        newCornersMT(1+T*(j-1):T*j, 1, i) = flip(CornersMT(1+T*(j-1):T*j, 1, i));
        newCornersMT(1+T*(j-1):T*j, 2, i) = flip(CornersMT(1+T*(j-1):T*j, 2, i));
    end
end
end

