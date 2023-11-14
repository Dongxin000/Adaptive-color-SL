function [imBWGrid] = testSegColorGrid(Scene, ColorGrid, imBWBoardMask, imBWCb)
%% Test the segmentation results using known prior information.

%%
imLight = Scene;
imColorGrid = ColorGrid;

%% Flood fill using the 1st checkerboard corner
% find color grid pixels
im1 = rgb2hsv(imColorGrid);
im2 = rgb2hsv(imLight);
s = mean(im1(:,:,3)) / mean(im2(:,:,3));
s = min(s, 1);
imGrid = imColorGrid-s*imLight;

%% Segment color grid
imColorGridMasked = ImgProc.maskImage(imGrid, imBWBoardMask);

% enhance color
imColorGridMasked = ImgProc.imadjust3(imColorGridMasked);

% to hsv
imHsv = rgb2hsv(imColorGridMasked);

% adaptive thresholding using Gaussian filter
sigma = 3;
imBWGrid = ImgProc.adaptiveThresh(imHsv(:,:,3), sigma);

% only keep the largest area
imBWGrid = bwareafilt(imBWGrid, 1);
% imBWGrid = bwmorph(imBWGrid,'clean', inf);

% close and open
se1 = strel('line',2,0);
se2 = strel('line',2,90);
imBWGrid = imclose(imBWGrid, se1);
imBWGrid = imclose(imBWGrid, se2);

% clean edges
imBWCb = imdilate(imBWCb, ones(8, 8));
imBWGrid(imBWCb(:)) = 0;
% 
% if(~isempty(camCorners))
%     if(verbose)
%         subplot(2,2,3);
%         imshow(imColorGridMasked);
%         title('Color grid masked');
%         drawnow
%         
%         subplot(2,2,4);
%         imshow(imBWGrid);
%         title('Color grid binary mask');
%         drawnow
%     end
% end

% imColorGridMasked = ImgProc.maskImage(imColorGridMasked, imBWGrid);
end

