function [hueLabel] = getHueLabel(optimalHueInfo, numColor)
%% Calculate the color label for each hue bin using the Maximum A Posteriori (MAP) method.

%%
%unitization option
unitizationOption = 0;

posteriori = zeros([numColor,256]);

% iterate through each hue bin and confirm its label
for i = 1:numColor
    vecWeight = AdaptiveGenPattern.calVecWeight(optimalHueInfo(i).vecHue, unitizationOption);
    sizeOfLabel = size(optimalHueInfo(i).vecHue,1);
    posteriori(i, :) =  vecWeight.*sizeOfLabel;
end

[val, idxLabel] = max(posteriori);

% get hue's color label
hueLabel = [optimalHueInfo(idxLabel).hue];
hueLabel = [hueLabel; idxLabel];
end

