function prjPatterns = createGrayPattern(w, h)

nbits = ceil(log2([w, h]));  % # of bits for vertical/horizontal patterns
offset = (2.^nbits - [w, h]) / 2; % offset the binary pattern to be symmetric

% coordinates to binary code
[c, r] = meshgrid(0:w-1, 0:h-1);
binPattern{1} = uint8(reshape(dec2bin(c + offset(1), nbits(1)), h, w, nbits(1))) - 48; % ascii code for '0' and '1' are 48 and 49
binPattern{2} = uint8(reshape(dec2bin(r + offset(2), nbits(2)), h, w, nbits(2))) - 48; % ascii code for '0' and '1' are 48 and 49

% binary pattern to gray pattern
% https://www.matrixlab-examples.com/gray-code.html
grayPattern = binPattern;

for n = 1:size(binPattern, 2)
    for i = 2:size(binPattern{n}, 3)
        grayPattern{n}(:,:,i) = xor(binPattern{n}(:,:,i-1), binPattern{n}(:,:,i));
    end
end

% allPatterns also containts complementary patterns and all 0/1 patterns
prjPatterns = zeros(h, w, 2*sum(nbits)+2, 'uint8');
prjPatterns(:,:,1) = ones(h, w);
% allPatterns(:,:,2) = zeros(h, w);

% vertical
for i = 1:size(grayPattern{1}, 3)
    prjPatterns(:,:,2*i+1) = grayPattern{1}(:,:,i); % gray patterns
    prjPatterns(:,:,2*i+2) = 1 - grayPattern{1}(:,:,i); % complementary patterns
end

% horizontal
for i = 1:size(grayPattern{2}, 3)
    prjPatterns(:,:,2*i+1+2*nbits(1)) = grayPattern{2}(:,:,i); % gray patterns
    prjPatterns(:,:,2*i+2+2*nbits(1)) = 1 - grayPattern{2}(:,:,i); % complementary patterns
end

prjPatterns = prjPatterns * 255;

% to RGB image
prjPatterns = permute(repmat(prjPatterns, [1, 1, 1, 3]), [1,2,4,3]);

% debug, write to file
% for i = 1:size(allPatterns, 3)
%     cv.imwrite(sprintf('data/img_%04d.png', i), allPatterns(:,:,i));
% end

end
