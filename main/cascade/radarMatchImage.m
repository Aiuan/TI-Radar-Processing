% aifuyuan create 20210601
% match radar pointClouds with image
% input: 1.radar frame's timestamp
%           2.images struct
% output: 1.matched image's timestamp
%             2.matched image's path
function [imageTimestamp, imagePath] = radarMatchImage(radarTimestamp, images)
    
    if radarTimestamp<=images(1).timestamp
        % radarTimestamp___images(1).timestamp
        imageTimestamp = images(1).timestamp;
        imagePath = images(1).path;
    elseif radarTimestamp>=images(end).timestamp
        % images(end).timestamp___radarTimestamp
        imageTimestamp = images(end).timestamp;
        imagePath = images(end).path;
    else
        % idx_former___radarTimestamp___idx_latter
        idx_former = 1;
        idx_latter = 2;
        while(radarTimestamp > images(idx_latter).timestamp)
            idx_former = idx_former + 1;
            idx_latter = idx_latter + 1;
        end
        if (radarTimestamp - images(idx_former).timestamp) < (images(idx_latter).timestamp - radarTimestamp)
            % idx_former__radarTimestamp__________idx_latter
            imageTimestamp = images(idx_former).timestamp;
            imagePath = images(idx_former).path;
        else
            % idx_former__________radarTimestamp__idx_latter
            imageTimestamp = images(idx_latter).timestamp;
            imagePath = images(idx_latter).path;
        end
        
    end

    
end