function [accuracy] = calColorDetectionAccuracy(calibInfo, flag)
%% Calculate color detection accuracy

%%
% get sets
setNumImages = calibInfo.numSets;
sets = calibInfo.sets;

% initialization
numHoriPixelTotalAllPoses = 0;
numHoriPixelCorrectAllPoses = 0;
numVertPixelTotalAllPoses = 0;
numVertPixelCorrectAllPoses = 0;

dir = calibInfo.path;
idx = findstr(dir, '\');
maskDir = dir(1:idx(end)-1);

for i = 1:setNumImages
    % get set idx
    setIdx = sets(i);
    if(setIdx <10)
        setIdx = strcat('0',num2str(setIdx));
    else
        setIdx = num2str(setIdx);
    end
    
    % load masks for different colors
    color1Hori = imread(fullfile(maskDir, 'mask', ['imColor1HoriBWGrid', sprintf('%02d', str2num(setIdx)), '.png']));
    color1Vert = imread(fullfile(maskDir, 'mask', ['imColor1VertBWGrid', sprintf('%02d', str2num(setIdx)), '.png']));
    color2Hori = imread(fullfile(maskDir, 'mask', ['imColor2HoriBWGrid', sprintf('%02d', str2num(setIdx)), '.png']));
    color2Vert = imread(fullfile(maskDir, 'mask', ['imColor2VertBWGrid', sprintf('%02d', str2num(setIdx)), '.png']));
    color3Hori = imread(fullfile(maskDir, 'mask', ['imColor3HoriBWGrid', sprintf('%02d', str2num(setIdx)), '.png']));
    color3Vert = imread(fullfile(maskDir, 'mask', ['imColor3VertBWGrid', sprintf('%02d', str2num(setIdx)), '.png']));
    color4Hori = imread(fullfile(maskDir, 'mask', ['imColor4HoriBWGrid', sprintf('%02d', str2num(setIdx)), '.png']));
    color4Vert = imread(fullfile(maskDir, 'mask', ['imColor4VertBWGrid', sprintf('%02d', str2num(setIdx)), '.png']));
    
    % load color-detected images
    imHoriLabel = imread(strcat(dir,'\im',flag,'HoriLabel',setIdx,'.png'));
    imVertLabel = imread(strcat(dir,'\im',flag,'VertLabel',setIdx,'.png'));
    
    % calculate the total number of pixels and the number of correctly detected pixels
    [numHoriPixelTotal(1), numHoriPixelCorrect(1)] = AdaptiveGenPattern.getTotalDetectedPixelsAndCorrectDetectedPixels(color1Hori, imHoriLabel, 1);
    [numVertPixelTotal(1), numVertPixelCorrect(1)] = AdaptiveGenPattern.getTotalDetectedPixelsAndCorrectDetectedPixels(color1Vert, imVertLabel, 2);
    [numHoriPixelTotal(2), numHoriPixelCorrect(2)] = AdaptiveGenPattern.getTotalDetectedPixelsAndCorrectDetectedPixels(color2Hori, imHoriLabel, 3);
    [numVertPixelTotal(2), numVertPixelCorrect(2)] = AdaptiveGenPattern.getTotalDetectedPixelsAndCorrectDetectedPixels(color2Vert, imVertLabel, 4);
    [numHoriPixelTotal(3), numHoriPixelCorrect(3)] = AdaptiveGenPattern.getTotalDetectedPixelsAndCorrectDetectedPixels(color3Hori, imHoriLabel, 5);
    [numVertPixelTotal(3), numVertPixelCorrect(3)] = AdaptiveGenPattern.getTotalDetectedPixelsAndCorrectDetectedPixels(color3Vert, imVertLabel, 6);
    [numHoriPixelTotal(4), numHoriPixelCorrect(4)] = AdaptiveGenPattern.getTotalDetectedPixelsAndCorrectDetectedPixels(color4Hori, imHoriLabel, 7);
    [numVertPixelTotal(4), numVertPixelCorrect(4)] = AdaptiveGenPattern.getTotalDetectedPixelsAndCorrectDetectedPixels(color4Vert, imVertLabel, 8);
    
    % add the results of different poses together
    numHoriPixelTotalAllPoses = numHoriPixelTotalAllPoses + sum(numHoriPixelTotal);
    numHoriPixelCorrectAllPoses = numHoriPixelCorrectAllPoses + sum(numHoriPixelCorrect);
    numVertPixelTotalAllPoses = numVertPixelTotalAllPoses + sum(numVertPixelTotal);
    numVertPixelCorrectAllPoses = numVertPixelCorrectAllPoses + sum(numVertPixelCorrect);
end

% horimeanAcc = horiCorrect/horiTotal;
% vertmeanAcc = vertCorrect/vertTotal;

% get color detection accuracy for all poses
accuracy = (numHoriPixelCorrectAllPoses + numVertPixelCorrectAllPoses)/(numHoriPixelTotalAllPoses + numVertPixelTotalAllPoses);

% show result
disp([' ']);
disp([strcat(flag, ' color detection accuracy:', num2str(accuracy))]);
disp(['  ']);
end

