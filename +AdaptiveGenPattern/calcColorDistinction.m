function [outLoss] = calcColorDistinction(histHue1, histHue2)
%%  Calculate the distinction of two colors 
% calculations based on the input hue distribution.

%% Vector dot product loss
% %unitization option
% unitization = 1;
% 
% %get weight
% weight1 = AdaptiveGenPattern.calVecWeight(vecHue1, unitization);
% weight2 = AdaptiveGenPattern.calVecWeight(vecHue2, unitization);

% calculate loss Vector dot product
outLoss = sum(histHue1.*histHue2);

%% Kullback–Leibler divergence loss
% %unitization option
% unitization = 0;
% 
% % get probability
% P1 = CompensateColor.calVecWeight(vecHue1, unitization);
% P2 = CompensateColor.calVecWeight(vecHue2, unitization);
% 
% % eliminate 0 
% P1 = CompensateColor.smoothPdf(P1');
% P2 = CompensateColor.smoothPdf(P2');
% 
% % calculate loss using Kullback–Leibler divergence
% % outLoss = -CompensateColor.calKLDivergence(P1', P2');
% outLoss = -CompensateColor.calJSDivergence(P1', P2');
end
