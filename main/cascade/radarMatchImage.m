function [imageTimestamp, imagePath] = radarMatchImage(radarTimestamp, image)
    
    idx_latter = 1;
    while(radarTimestamp > image(idx_latter).timestamp)
        idx_latter = idx_latter + 1;
    end
    idx_former = max(idx_latter - 1, 1);
    
    diff_former = radarTimestamp - image(idx_former).timestamp;
    diff_latter = image(idx_latter).timestamp - radarTimestamp;
    if (diff_former >= diff_latter)
        imageTimestamp = image(idx_latter).timestamp;
        imagePath = image(idx_latter).path;
    else
        imageTimestamp = image(idx_former).timestamp;
        imagePath = image(idx_former).path;
    end
end