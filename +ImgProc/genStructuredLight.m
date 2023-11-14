function im = genStructuredLight(prjW, prjH, adaptiveOption, currCompHsv, numColor, brightness)
%% Generate color-coded structured light pattern for calibration.
% prjW and prjH are the projector screen resolution's width and height in pixel.

%% License
% ACADEMIC OR NON-PROFIT ORGANIZATION NONCOMMERCIAL RESEARCH USE ONLY
% Copyright (c) 2018 Bingyao Huang
% All rights reserved.

% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met: 

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% If you publish results obtained using this software, please cite our paper.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

if(nargin < 6)
    brightness = 1.0;
end

numHoriColor = numColor(1);
numVertColor = numColor(2);

aspect = prjW / prjH;
imgW = 1920;
imgH = floor(imgW / aspect);

if(prjW > imgW)
    imgW = prjW;
end

if(prjH > imgH)
    imgH = prjH;
end

% adaptiveOption = 0;

if(adaptiveOption == 1)
   [horiList, vertList, horiPos, vertPos] = AdaptiveGenPattern.newCreateDeBruijnSeq(imgW, imgH, numHoriColor, numVertColor);
   % currCompHsv =  load('D:\git\ARSL3.0\src\MATLAB\AdaptivePatternAndHueLabel\currCompHsv.mat');
   % currCompHsv = currCompHsv.currCompHsv;
   colorList = hsv2rgb(currCompHsv)*brightness;
else
    [horiList, vertList, horiPos, vertPos] = ImgProc.createDeBruijnSeq(imgW, imgH);
    r = [1, 0, 0];
    y = [1, 0.75, 0];
    l = [0.5, 1, 0];
    g = [0, 1, 0.5];
    c = [0, 1, 1];
    b = [0, 0.25, 1];
    p = [0.75, 0, 1];
    m = [1, 0, 0.75];
    colorList = [r; y; l; g; c; b; p; m;] * brightness;
end
    
% rgb image
imLarge = zeros(imgH, imgW, 3);

% color of node pixels is horizontal stripe's or vertical stripe's color
isHori = 1;

if(adaptiveOption == 1)
    if(isHori == 1)
        % adpative pattern
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
    
    else
        
        for i = 1:length(horiPos)
            curPos = horiPos(i);
            curPixel = reshape(colorList(horiList(i),:), [1, 1, 3]);
            curStripe = repmat(curPixel, [3, imgW, 1]);
            imLarge(curPos-1:curPos+1,:,:) = curStripe;

        end

         for j = 1:length(vertPos)
            curPos = vertPos(j);
            curPixel = reshape(colorList(vertList(j),:), [1, 1, 3]);
            curStripe = repmat(curPixel, [imgH, 3, 1]);
            imLarge(:,curPos-1:curPos+1,:) = curStripe;

        end
        
    end
else
    % TASE20 pattern: the color node pixels is horizontal stripe's color
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
end

%% Crop the pattern image according to projector resolution
% im = imLarge(1:prjH, 1:prjW,:);

% dataRoot='C:\Users\hp\Desktop\imgset\imgset\Pro_Rad';
% Name(1).n = 'I.png';
% Name(2).n = 'Ib_in.png';
% Name(3).n = 'Ig_in.png';
% Name(4).n = 'Ir_in.png';
% Name(5).n = 'pattern.jpg';
% Name(6).n = 'patternS.png';
% Name(7).n = 'patternT.png';
% Name(8).n = 'patternU_new.png';
% 
% i=1;
% 
% dir = fullfile(dataRoot, Name(i).n);
% 
% im = imread(dir);
% 
% im = imread('C:\Users\hp\Desktop\800600\test3.jpg');
im = imLarge(1:prjH, 1:prjW,:);

% im(:,:,1) = [255];
% im(:,:,2) = [255];
% im(:,:,3) = [255];

end

