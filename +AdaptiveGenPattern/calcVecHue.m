function [vecHue] = calcVecHue(imMasked)
%% Calculate the vector of hue

imHsv = rgb2hsv(imMasked);
imHue = imHsv(:,:,1);
imValue = imHsv(:,:,3);

% get valid pixel idx
pixelIdx = find(imValue > 0);

% get valid hue
vecHue = imHue(pixelIdx);
end

