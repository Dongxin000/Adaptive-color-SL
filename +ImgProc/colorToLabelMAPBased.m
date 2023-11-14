function [imAllLabel, imHoriLabel, imVertLabel] = colorToLabelMAPBased(imColorGrid, imNode, imHoriEdge, imVertEdge, hueLabel, verbose)
%% Extract color labels from adaptive color grid image using the Maximum A Posteriori (MAP) method.

%%
if(nargin < 6)
    verbose = false;
end

%% Create wide imNode, imHoriEdge and imVertEdge for color masking
imHoriRGB = maskColorGrid(imColorGrid, imNode, imHoriEdge);
imVertRGB = maskColorGrid(imColorGrid, imNode, imVertEdge);

%% Use kmeans for horizontal and vertical color detection
[imHoriLabel, imHoriRecv] = clusterColors(imHoriRGB, hueLabel, 1);
[imVertLabel, imVertRecv] = clusterColors(imVertRGB, hueLabel, 0);

if(verbose)
    figure('Name', 'colorToLabel', 'units','normalized','outerposition',[0 0  1  1]);
    subplot(2,1,1);
    imshowpair(imHoriRGB, imHoriRecv, 'Montage');
    title('[horizontal] Original color grid and detected labels in pseudocolor');
    drawnow
    
    subplot(2,1,2);
    imshowpair(imVertRGB, imVertRecv, 'Montage');
    title('[vertical] Original color grid and detected labels in pseudocolor');
    drawnow
end

%% combine horizontal and vertical labels
imAllLabel = imHoriLabel + imVertLabel;

end

%% Local functions 
function imMaskedRGB = maskColorGrid(imEnhance, imNode, imEdgeMask)

% dilated imNode
imNodeWide = imdilate(imNode, strel('disk',1));
%     figure;imshow(imNodeWide);
%     title('Dilated Nodes');

% subtract imNodeWide from imHoriWide to get rid of color interference at
% Node intersection
imEdgeMaskWide = imdilate(imEdgeMask, strel('disk',1));

% eliminate -1
imMask = imEdgeMaskWide - imNodeWide;
imMask(find(imMask == -1)) = 0;

imMaskedRGB = ImgProc.maskImage(imEnhance, imMask);
%     figure;
%     imshowpair(imEdgeMaskWide, imMaskedRGB, 'Montage');
%     title('Masked colors');

end

%% Local functions
function [imLabelOut, imRecoverRGB] = clusterColors(imRGB, hueLabel, isHorizontal)
% load the labels corresponding to each hue bin
if(isHorizontal)
    hueLabel = hueLabel(:,:,1);
    hueLabel(2,:) = hueLabel(2,:)*2-1;
else
    hueLabel = hueLabel(:,:,2);
    hueLabel(2,:) = hueLabel(2,:)*2;
end

% separate [hue, saturation, value] channels
imHSV = rgb2hsv(imRGB);
imHue = imHSV(:,:,1);
imSat = imHSV(:,:,2);
imVal = imHSV(:,:,3);
colorPixelIdx = find(imVal>0);

% initialization
tempImHue = imHue;
numBin = 256;
bins = linspace(0,1,numBin+1);

imLabelOut = zeros(size(tempImHue),'uint8');

% use the lookup table method to determine the label from the hue value
for i = 1:size(colorPixelIdx, 1)
    for j=1:numBin
        if(j ~= numBin)
            if(imHue(colorPixelIdx(i)) >=  bins(j) && imHue(colorPixelIdx(i)) < bins(j+1))
                tempImHue(colorPixelIdx(i)) = hueLabel(1,j);
                imLabelOut(colorPixelIdx(i)) = hueLabel(2,j);
            end
        else
            if(imHue(colorPixelIdx(i)) >=  bins(j) && imHue(colorPixelIdx(i)) <= bins(j+1))
                tempImHue(colorPixelIdx(i)) = hueLabel(1,j);
                imLabelOut(colorPixelIdx(i)) = hueLabel(2,j);
            end
       end
    end
end

tempImHsv = tempImHue;

imVal(find(imVal == 0)) = 0;
imVal(find(imVal > 0)) = 1;

tempImHsv(:,:,2:3) = 1;
imRecoverRGB = hsv2rgb(tempImHsv);
imRecoverRGB = imRecoverRGB.*imVal;
end
