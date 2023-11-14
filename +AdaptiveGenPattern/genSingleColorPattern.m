function im = genSingleColorPattern(prjW, prjH, colorFlag, brightness)
%% Generate single color structured light pattern.

%%
if(nargin < 4)
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

% only display stripes of a specific color
if(colorFlag == 'c1')
    r = [1, 1, 1];
    y = [1, 1, 1];
    l = [0, 0, 0];
    g = [0, 0, 0];
    c = [0, 0, 0];
    b = [0, 0, 0];
    p = [0, 0, 0];
    m = [0, 0, 0];
elseif(colorFlag == 'c2')
    r = [0, 0, 0];
    y = [0, 0, 0];
    l = [1, 1, 1];
    g = [1, 1, 1];
    c = [0, 0, 0];
    b = [0, 0, 0];
    p = [0, 0, 0];
    m = [0, 0, 0];
elseif(colorFlag == 'c3')
    r = [0, 0, 0];
    y = [0, 0, 0];
    l = [0, 0, 0];
    g = [0, 0, 0];
    c = [1, 1, 1];
    b = [1, 1, 1];
    p = [0, 0, 0];
    m = [0, 0, 0];
elseif(colorFlag == 'c4')
    r = [0, 0, 0];
    y = [0, 0, 0];
    l = [0, 0 ,0];
    g = [0, 0, 0];
    c = [0, 0, 0];
    b = [0, 0, 0];
    p = [1, 1, 1];
    m = [1, 1, 1];
end
    
colorList = [r; y; l; g; c; b; p; m] * brightness;

% rgb image
imLarge1 = zeros(imgH, imgW, 3);

for i = 1:length(horiPos)
    curPos = horiPos(i);
    curPixel = reshape(colorList(horiList(i),:), [1, 1, 3]);
    curStripe = repmat(curPixel, [3, imgW, 1]);
    imLarge1(curPos-1:curPos+1,:,:) = curStripe;

end

for j = 1:length(vertPos)
    curPos = vertPos(j);
    curPixel = reshape(colorList(vertList(j),:), [1, 1, 3]);
    curStripe = repmat(curPixel, [imgH, 3, 1]);
    imLarge1(:,curPos-1:curPos+1,:) = curStripe;
    
end

% rgb image
imLarge2 = zeros(imgH, imgW, 3);

for j = 1:length(vertPos)
    curPos = vertPos(j);
    curPixel = reshape(colorList(vertList(j),:), [1, 1, 3]);
    curStripe = repmat(curPixel, [imgH, 3, 1]);
    imLarge2(:,curPos-1:curPos+1,:) = curStripe;
    
end

for i = 1:length(horiPos)
    curPos = horiPos(i);
    curPixel = reshape(colorList(horiList(i),:), [1, 1, 3]);
    curStripe = repmat(curPixel, [3, imgW, 1]);
    imLarge2(curPos-1:curPos+1,:,:) = curStripe;

end

imLarge = imLarge1+imLarge2;

%% Crop the pattern image according to projector resolution
im = imLarge(1:prjH, 1:prjW,:);

end

