function imBW = segFloodFill(im, row, col)
%% Segment the image using flood fill, the seed locations is (row, col)

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

%%
imBW = zeros(size(im));
tol = 0.05;
thresh = 500;

% iterate if not enough flood fill was performed to avoid erasing the image
while(nnz(imBW) < thresh)
    % convert RGB to LAB
    imLab = rgb2lab(im);
    
    % normalize
    imLabNorm = sum((imLab - imLab(row,col,:)).^2, 3);
    imLabNorm = mat2gray(imLabNorm);
    
    tol = tol * 1.5; % tolerance
    imBW = grayconnected(imLabNorm, row, col, tol);
end
end
