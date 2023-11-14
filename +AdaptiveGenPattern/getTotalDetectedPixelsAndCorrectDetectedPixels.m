function [totalDetectedPixels, correctDetectedPixels] = getTotalDetectedPixelsAndCorrectDetectedPixels(mask, imLabel, label)
%% Calculate the number of pixels detected and the number of pixels detected correctly.

%%
% get the pixel index of the valid area
idxPixel = find(mask == 255);
totalDetectedPixels = length(idxPixel);

% get the detected color label
vecLabels =imLabel(idxPixel);

% eliminate undetected pixels
% numOfZeros = sum(vecLabels(:) == 0);
numOfZeros = size(find(vecLabels(:) == 0), 1);
totalDetectedPixels = totalDetectedPixels - numOfZeros;

% get the number of correctly detected pixels
% correctDetectedPixels = sum(vecLabels(:) == label);
correctDetectedPixels = size(find(vecLabels(:) == label), 1);
end
