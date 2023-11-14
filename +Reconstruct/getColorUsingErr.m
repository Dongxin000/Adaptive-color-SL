function [errColor, errInlierIdx] = getColorUsingErr(currErr, maxErr, errMap, cMap)
%% Use the alignment error to define color for every point.

%%
% get inliers
errOutlierIdx = isnan(currErr);
errInlierIdx = ~errOutlierIdx;

% set over maxErr to maxErr
currErr(find(currErr>maxErr)) = maxErr;

% get color idx according to nearest neighbor
newErr = repmat(currErr, [1,256]);
newErrMap = repmat(errMap, [length(currErr),1]);
[~, idx] = min(abs(newErrMap - newErr), [], 2);

% convert double to uint8 
errColor = im2uint8(cMap(idx, :, :));
end

