function [histHue] = calcHistHue(vecHue, unitizationOption)
%% Calculate the hue histgram for color

%% 
%get distribution
numBin = 256;

bins = linspace(0,1,numBin+1);
distribution = zeros([1,numBin]);

for i = 1:numBin
    if(i ~= numBin)
        distribution(i) = size(find(vecHue>=bins(i) & vecHue< bins(i+1)), 1);
    else
        distribution(i) = size(find(vecHue>=bins(i) & vecHue<= bins(i+1)), 1);
    end
end

% get histgram of hue
total = size(vecHue, 1);
histHue = distribution./total;

% unitization: convert to vector
if(unitizationOption)
    histHue = histHue/norm(histHue);
end
end

