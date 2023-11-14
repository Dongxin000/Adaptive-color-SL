function im = genWhiteLight(prjW, prjH,brightness)
%% Generate white color structured light pattern for calibration.

%%

if(nargin < 3)
    brightness = 1.0;
end

aspect = prjW / prjH;
imgW = 1920;
imgH = floor(imgW / aspect);

if(prjW > imgW)
    imgW = prjW;
end

if(prjH > imgH)
    imgH = prjH;
end

[horiList, vertList, horiPos, vertPos] = ImgProc.createDeBruijnSeq(imgW, imgH);

r = [1,1,1];
y = [1,1,1];
l = [1,1,1];
g =[1,1,1];
c = [1,1,1];
b =[1,1,1];
p =[1,1,1];
m =[1,1,1];

colorList = [r; y; l; g; c; b; p; m] * brightness;

% rgb image
imLarge = zeros(imgH, imgW, 3);

for j = 1:length(vertPos)
    curPos = vertPos(j);
    curPixel = reshape(colorList(vertList(j),:), [1, 1, 3]);
    curStripe = repmat(curPixel, [imgH, 3, 1]);
    imLarge(:,curPos-1:curPos+1,:) = curStripe;

end

for i = 1:length(horiPos)
    curPos = horiPos(i);
    curPixel = reshape(colorList(horiList(i),:), [1, 1, 3]);
    curStripe = repmat(curPixel, [3, imgW, 1]);
    imLarge(curPos-1:curPos+1,:,:) = curStripe;

end

%% Crop the pattern image according to projector resolution

im = imLarge(1:prjH, 1:prjW,:);
end

