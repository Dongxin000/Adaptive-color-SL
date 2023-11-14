function [imOut] = segColorGridAndColorToLabel(imColorGridMasked, imBWGrid, hueLabel)
%% Segment the image and detect colors, converting them into labels.

%%
% mask color grid
imColorGridMasked = ImgProc.maskImage(imColorGridMasked, imBWGrid);

imHsv = rgb2hsv(imColorGridMasked);
imHue = imHsv(:,:,1);
imValue = imHsv(:,:,3);

tempImHue = imHue;
pixelIdx = find(imValue>0);

%% Calculate the color label for each hue bin using the Maximum A Posteriori (MAP) method.
numBin = 256;
bins = linspace(0,1,numBin+1);

for i = 1:size(pixelIdx, 1)
    for j=1:numBin
        if(j ~= numBin)
            if(imHue(pixelIdx(i)) >=  bins(j) && imHue(pixelIdx(i)) < bins(j+1))
                tempImHue(pixelIdx(i)) = hueLabel(1,j);
            end
        else
            if(imHue(pixelIdx(i)) >=  bins(j) && imHue(pixelIdx(i)) <= bins(j+1))
                tempImHue(pixelIdx(i)) = hueLabel(1,j);
            end
       end
    end
end

tempImHsv = tempImHue;

imValue(find(imValue == 0)) = 0;
imValue(find(imValue > 0)) = 1;

tempImHsv(:,:,2:3) = 1;
imOut = hsv2rgb(tempImHsv);

imOut = imOut.*imValue;
end

