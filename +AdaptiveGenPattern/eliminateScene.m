function [s, imGrid] = eliminateScene(imColorGrid,imScene)
%% Eliminate the scene, retaining only the color grid

%%
im1 = rgb2hsv(imColorGrid);
im2 = rgb2hsv(imScene);

s = mean(im1(:,:,3)) / mean(im2(:,:,3));
s = min(s, 1);

% subtract the scene
imGrid = imColorGrid-s*imScene;
end