function [imMask1, imMask2, imMask3, imMask4, imBWBoardMask, imBWCb] = genMasks(imScene, imSingleColor)
%% Create masks for different colors to facilitate the extraction of stripes of various colors.

%%
% get single color image
imSingleColor1 = imSingleColor(1).image;
imSingleColor2 = imSingleColor(2).image;
imSingleColor3 = imSingleColor(3).image;
imSingleColor4 = imSingleColor(4).image;

% eliminate the scene to extract the grid
[s1, imGrid1] = AdaptiveGenPattern.eliminateScene(imSingleColor1, imScene);
[s2, imGrid2] = AdaptiveGenPattern.eliminateScene(imSingleColor2, imScene);
[s3, imGrid3] = AdaptiveGenPattern.eliminateScene(imSingleColor3, imScene);
[s4, imGrid4] = AdaptiveGenPattern.eliminateScene(imSingleColor4, imScene);

% combine to form complete grid pattern
imWhiteLight = im2uint8(imGrid1+imGrid2+imGrid3+imGrid4)+mean([s1,s2,s3,s4])*imScene;

[camCorners, boardSize, usedImIdx] = detectCheckerboardPoints(imScene, PartialDetections=false);
[imBWGrid, imBWBoardMask, imBWCb] = AdaptiveGenPattern.segWhiteGrid(imScene,imWhiteLight,camCorners,0);

% imWholeBWGrid = imBWGrid;

% Skeletonize
imSkele = ImgProc.bw2skele(imBWGrid);

% if (verbose)
%     figure;
%     imshowpair(imBWGrid, imSkele, 'Montage');
%     title('Grid mask and skeletonized grid mask');
% end

% Extract edge, node endpoint and # neighbors image
[imEdge, imNode, ~, ~] = extractStructs(imSkele);

imBWGrid = maskColorGrid(imBWGrid, imNode,imEdge);

% get single color grid mask
[imMask1] = AdaptiveGenPattern.segSingleColorGrid(imScene, imSingleColor1, imBWGrid, imBWBoardMask);
[imMask2] = AdaptiveGenPattern.segSingleColorGrid(imScene, imSingleColor2, imBWGrid, imBWBoardMask);
[imMask3] = AdaptiveGenPattern.segSingleColorGrid(imScene, imSingleColor3, imBWGrid, imBWBoardMask);
[imMask4] = AdaptiveGenPattern.segSingleColorGrid(imScene, imSingleColor4, imBWGrid, imBWBoardMask);

% remove intersections (nodes) pixles
[imMask1, imMask2, imMask3, imMask4] = AdaptiveGenPattern.removeNodePixels(imMask1, imMask2, imMask3, imMask4);

%% location functions
function [imEdges, imNodes, imEndPoints, imNeighbor] = extractStructs(imSkele)
imNeighbor = imfilter(double(imSkele), [1, 1, 1; 1, 0, 1; 1, 1, 1]);
imNeighbor = imNeighbor .* imSkele;

imEdges = imNeighbor == 2;
imEndPoints = imNeighbor == 1;
imNodes = logical(imSkele - imEdges - imEndPoints);
end

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

end

