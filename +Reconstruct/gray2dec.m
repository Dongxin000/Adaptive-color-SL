function decPattern = gray2dec(grayPattern)

% code borrowed from http://mesh.brown.edu/byo3d/source.html

% grayMat is a 3D matrix, where n is the length of Gray code
[h, w, n] = size(grayPattern);

% gray pattern to binary pattern
% https://www.matrixlab-examples.com/gray-code.html
binPattern = grayPattern;

for i = 2:n
    binPattern(:,:,i) = xor(binPattern(:,:,i-1), grayPattern(:,:,i));
end

% Convert per-pixel binary code to decimal.
decPattern = zeros(h, w);
for i = n:-1:1
   decPattern = decPattern + 2^(n-i)*double(binPattern(:,:,i));
end
decPattern = decPattern + 1;