function dirList = listDir(inputDir, type)

if(strcmp(type, 'dir'))
    dirList = dir(inputDir);
    dirList = dirList(~ismember({dirList.name}, {'.', '..'}));
    dirList = dirList([dirList.isdir]);
else
    dirList = dir(fullfile(inputDir, type));
end
end