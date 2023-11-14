function projectAndCapture(app, prjImgs, camImgSaveDir, idx)
arguments
    app
    prjImgs (:,:,3,:) {mustBeNumeric}
    camImgSaveDir char
    idx (1,2) {mustBeNumeric}=[1, size(prjImgs, 4)]
end

camName = app.cam.Name;

for i = idx(1):idx(2)
    if(~app.capturing)
        uialert(app.ProCamCalibUIFigure, 'Capture stopped by user.','Interruption');
        return;
    end
    
    imCamName =  fullfile(camImgSaveDir, sprintf('img_%04d.png', i));
    
    % project
    app.projectImage(prjImgs(:,:,:,i));
    pause(0.02);
    
    % clear buffer
    % pause(app.ExposureTimesSlider.Value);
    app.clearCamBuffer();
    
    im = app.cam.snapshot; % faster since MATLAB>2019b
    
    % for DeProCams Cannon6D with CamLink 4K. Comment this for other cases
    if(contains(camName, 'Cam Link 4K'))
        im = imresize(im(:, 240:1679,:), app.camImgSize); % compared with cv.resize, MATLAB imresize uses lowpass filter for anti-aliasing.
    elseif(contains(camName, 'EOS Webcam Utility'))
        % [bug] MATLAB 2021a shows upside down frames of EOS Webcam Utility
        im = flipud(im);
    end
    
    % Camera-captured scene image
    if(i==1)
        app.CamSceneImg.ImageSource = im;
    end
    
    % resize image to the given size, preview and save
    im = imresize(im, app.camImgSize);
    app.CamCapPrjImg.ImageSource = im;
    cv.imwrite(imCamName, im); % faster than MATLAB's imwrite
end
end