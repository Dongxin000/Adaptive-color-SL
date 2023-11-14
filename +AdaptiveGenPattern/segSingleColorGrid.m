function [imOut] = segSingleColorGrid(imLight, imWhiteGrid,imBWGrid,imBWBoardMask)
%% Segment a structured light image with a single color.

%%
% get ROI
imWhiteGridROI = ImgProc.maskImage(imWhiteGrid, imBWBoardMask);
imLightROI = ImgProc.maskImage(imLight, imBWBoardMask);

% eliminate scene.
im1 = rgb2hsv(imWhiteGridROI);
im2 = rgb2hsv(imLightROI);
s = mean(im1(:,:,3)) / mean(im2(:,:,3));
s = min(s, 1);
imGrid = imWhiteGridROI-s*imLightROI; 

% mask color grid
mask = imBWBoardMask;
imGridMasked = ImgProc.maskImage(imGrid, mask);

%  segment image
[M, N, K] = size(imGridMasked);
I = reshape(imGridMasked, M*N,3);
idx = find(mask);
I = double(I(idx, 1:3));
[c, m] = ImgProc.covmatrix(I);

d = diag(c);
sd = (sqrt(d));

E = ImgProc.colorseg('euclidean',imGrid,ceil(max(sd)),m);
imOut = ImgProc.maskImage(~E, imBWGrid);

% %Otsu's method to segment image
% imGridGray = rgb2gray(imGrid);
% thresh = multithresh(imGridGray, 1);
% imLabels = imquantize(imGridGray,thresh);
% imLabels = imLabels-1;
% imOut = ImgProc.maskImage(imLabels, imBWGrid);

se1 = strel('line',2,0);
se2 = strel('line',2,90);
imOut = imclose(imOut, se1);
imOut = imclose(imOut, se2);

imBWCb = ~mask;
imBWCb = imdilate(imBWCb, ones(8, 8));
imOut(imBWCb(:)) = 0;
end

