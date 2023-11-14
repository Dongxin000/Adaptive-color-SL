function imD = readAllImgs(imgPath, ext, idx)

if(nargin < 2)
    imgDs = imageDatastore(imgPath);
else
    imgDs = imageDatastore(imgPath, 'FileExtensions', ext);
end

numImgs = length(imgDs.Files);

if(nargin < 3)
    idx = 1:numImgs;
end

% read all images as cell (very fast)
imD = imgDs.readall;

% keep only the idx
imD = imD(idx);

% convert to 4D mat
imD = cat(4, imD{:});

% convert to single
imD = im2single(imD);
end

