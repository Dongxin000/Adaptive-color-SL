function [imColor1GridMasked,imColor2GridMasked,imColor3GridMasked,imColor4GridMasked] = getSingleColorGrid(imScene, imColorGrid, Mask1,Mask2,Mask3,Mask4 , boardMask)
%% Obtain the masked color grid with only one specific color.

%% 
% eliminate scene
im1 = rgb2hsv(imColorGrid);
im2 = rgb2hsv(imScene);
s = mean(im1(:,:,3)) / mean(im2(:,:,3));
s = min(s, 1);
imGrid = imColorGrid-s*imScene;

% get masked color grid
imColorGridMasked = ImgProc.maskImage(imGrid, boardMask);

% enhance color
% imColorGridMaskedEnhanced = ImgProc.imadjust3(imColorGridMasked);

% get masked color subgrid for different colors
imColor1GridMasked = ImgProc.maskImage(imColorGridMasked, Mask1);
imColor2GridMasked = ImgProc.maskImage(imColorGridMasked, Mask2);
imColor3GridMasked = ImgProc.maskImage(imColorGridMasked, Mask3);
imColor4GridMasked = ImgProc.maskImage(imColorGridMasked, Mask4);
end

