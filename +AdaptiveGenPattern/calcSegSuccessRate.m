function [res] = calcSegSuccessRate(imBWGrid, imMask)
%% Calculate the segmentation success rate

%%
% get valid pixel idx
pixelIdx = find(imMask == 1);

% get valid pixel
vecPixels = imBWGrid(pixelIdx);

% calculate the number of pixels successfully segmented
correct = length(find(vecPixels == 1));

% debug display the segmentation success rate of every color 
% correct/length(vecPixels)

% The threshold for segmentation success rate is set to 0.8
if(correct/length(vecPixels) < 0.80)
    res = 0;
else
    res = 1;
end
end

