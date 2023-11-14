%% Remove dir if exists
function dir = rmdirs(dir)
if(exist(dir,'dir'))
    rmdir(dir, 's');
end
end