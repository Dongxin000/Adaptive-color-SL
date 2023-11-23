function [camPts, prjPts, imMax, imMin] = decodeGrayPattern(imCam, prjW, prjH, thresh)

[camH, camW] = size(imCam, [1, 2]);

nbits = ceil(log2([prjW, prjH]));  % # of bits for vertical/horizontal patterns
offset = (2.^nbits - [prjW, prjH]) / 2; % offset the gray pattern to center symmetric

% find direct light mask using Nayar's TOG'06 method (also see Moreno & Taubin 3DV'12)
% we need at least two complementary images, but more images are better.
imCamGray = zeros(size(imCam, 1), size(imCam, 2), size(imCam, 4), 'double');

for i=1:size(imCam, 4)
    imCamGray(:,:,i) = rgb2gray(imCam(:,:,:,i));
%     imCamGray(:,:,i) = rgb2gray(rgb2lin(imCam(:,:,:,i))); % convert from srgb to linear before to gray
end

% use the 3rd and 2nd highest frequences (8 images) to estimate direct and global components as suggested by Moreno &
% Taubin 3DV'12
highFreqIdx = [2*nbits(1)-3:2*nbits(1), 2*nbits(1)+2*nbits(2)-3:2*nbits(1)+2*nbits(2)];

imMax = max(imCamGray(:,:, highFreqIdx), [], 3); % max image L+
imMin = min(imCamGray(:,:, highFreqIdx), [], 3); % min image L-

if(nargin < 4)
    t = 0.1; % min max contrast thresh (most significant for eliminating wrong decoded pixels)
    b = 0.7; % projector back light stength (for mask use a large b, for real direct/indirect separation, use a smaller b)
    m = 0.02;  % a threshold set by Xu and Aliaga GI'07 Table 2.
else
    t = thresh.t;
    b = thresh.b;
    m = thresh.m;
end


imD = (imMax-imMin) / (1-b); % direct light image
imG = 2*(imMin-b*imMax) / (1-b*b); % indirect (global) light image

imD(imD > 1) = 1;
imD(imG < 0) = imMax(imG < 0);
imG(imG < 0) = 0;

% fm({imD, imG});

% robust pixel classification (Xu and Aliaga GI'07 see and Moreno & Taubin 3DV'12)
imV = imCamGray(:,:,3:2:2*nbits(1)+1);
imVcmp = imCamGray(:,:,4:2:2*nbits(1)+2);

imH = imCamGray(:,:,2*nbits(1)+3:2:2*nbits(1) + 2*nbits(2)+1);
imHcmp = imCamGray(:,:,2*nbits(1)+4:2:2*nbits(1) + 2*nbits(2)+2);

grayPattern{1} = nan(size(imV));
grayPattern{2} = nan(size(imH));

grayPattern{1}((imV >= imG) & (imVcmp <= imD)) = 1;
grayPattern{1}((imV <= imD) & (imVcmp >= imG)) = 0;
grayPattern{1}((imD > imG) & (imV <= imVcmp)) = 0;
grayPattern{1}((imD > imG) & (imV > imVcmp)) = 1;

grayPattern{2}((imH >= imG) & (imHcmp <= imD)) = 1;
grayPattern{2}((imH <= imD) & (imHcmp >= imG)) = 0;
grayPattern{2}((imD > imG) & (imH <= imHcmp)) = 0;
grayPattern{2}((imD > imG) & (imH > imHcmp)) = 1;

% nanMask = any(isnan(grayPattern{1}), 3) | any(isnan(grayPattern{2}), 3) | any(abs(imV - imVcmp) < t, 3) | any(abs(imH - imHcmp) < t, 3);
nanMask = any(isnan(grayPattern{1}), 3) | any(isnan(grayPattern{2}), 3) | (abs(imMax - imMin) < t);


grayPattern{1}(repmat(nanMask, [1, 1, nbits(1)])) = 0;
grayPattern{2}(repmat(nanMask, [1, 1, nbits(2)])) = 0;

% grayPattern{1} = uint8(((imD > imG) & (imV > imVcmp)) | ((imV >= imG) & (imVcmp <= imD))); % vertical
% grayPattern{2} = uint8(((imD > imG) & (imH > imHcmp)) | ((imH >= imG) & (imHcmp <= imD))); % horizontal

% grayPattern{1} = uint8((imD > imG) & (imV > imVcmp)); % vertical
% grayPattern{2} = uint8((imD > imG) & (imH > imHcmp)); % horizontal


% grayPattern{1} = uint8(imV > imVcmp); % vertical
% grayPattern{2} = uint8(imH > imHcmp); % horizontal


% gray pattern to binary pattern
% https://www.matrixlab-examples.com/gray-code.html
% binPattern = grayPattern;
% 
% for n = 1:size(grayPattern, 2)
%     for i = 2:size(grayPattern{n}, 3)
%         binPattern{n}(:,:,i) = xor(binPattern{n}(:,:,i-1), grayPattern{n}(:,:,i));
%     end
% end

% gray pattern to decimal pattern
prjX = Reconstruct.gray2dec(grayPattern{1}) - offset(1);
prjY = Reconstruct.gray2dec(grayPattern{2}) - offset(2);

prjX(nanMask) = -1;
prjY(nanMask) = -1;

% binary to decimal, ascii code for '0' and '1' are 48 and 49
% camPts2d = cat(3, reshape(bin2dec(char(reshape(binPattern{1}, [], nbits(1)) + 48)), camH, camW)-offset(1), reshape(bin2dec(char(reshape(binPattern{2}, [], nbits(2)) + 48)), camH, camW)-offset(2));
% camPts2d(imD < m) = nan;

% x = reshape(bin2dec(char(reshape(binPattern{1}, [], nbits(1)) + 48)), camH, camW) - offset(1);
% y = reshape(bin2dec(char(reshape(binPattern{2}, [], nbits(2)) + 48)), camH, camW) - offset(2);

invalidIdx = (imD < m) | (prjX < 1) | (prjX > prjW) | (prjY < 1) | (prjY > prjH);
prjX(invalidIdx) = nan;
prjY(invalidIdx) = nan;
prjPts = [prjX(:), prjY(:)];

[camX, camY] = meshgrid(1:camW, 1:camH);
camPts = [camX(:), camY(:)];

camPts(invalidIdx(:), :) = [];
prjPts(invalidIdx(:), :) = [];

end

