%% Make dir if does not exist
function dir = mkdirs(dir)
if(~exist(dir,'dir'))
    mkdir(dir);
end
end