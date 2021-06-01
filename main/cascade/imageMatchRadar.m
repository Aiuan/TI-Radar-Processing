% aifuyuan create 20210601
% match image with radar pointClouds
% input: 1.image's timestamp
%           2.radar_pointClouds struct
%           3.mode: accroding to timestamp or pc_timestamp
% output: 1.matched radar frame's timestamp
%             2.matched radar_pointCloud
function [radarTimestamp, radar_pointCloud] = imageMatchRadar(imageTimestamp, radar_pointClouds, mode)
    if strcmp(mode, 'timestamp')
        % =========================雷达采用内置时间戳=============================================
        if imageTimestamp<=radar_pointClouds(1).timestamp
            radarTimestamp = radar_pointClouds(1).timestamp;
            radar_pointCloud = radar_pointClouds(1).radar_pointCloud;
        elseif imageTimestamp>=radar_pointClouds(end).timestamp
            radarTimestamp = radar_pointClouds(end).timestamp;
            radar_pointCloud = radar_pointClouds(end).radar_pointCloud;
        else
            idx_former = 1;
            idx_latter = 2;
            while(imageTimestamp > radar_pointClouds(idx_latter).timestamp)
                idx_former = idx_former + 1;
                idx_latter = idx_latter + 1;
            end
            if (imageTimestamp - radar_pointClouds(idx_former).timestamp) < (radar_pointClouds(idx_latter).timestamp - imageTimestamp)
                radarTimestamp = radar_pointClouds(idx_former).timestamp;
                radar_pointCloud = radar_pointClouds(idx_former).radar_pointCloud;
            else
                radarTimestamp = radar_pointClouds(idx_latter).timestamp;
                radar_pointCloud = radar_pointClouds(idx_latter).radar_pointCloud;
            end
        end
        % ==================================================================================
    else
        % =========================雷达采用PC端时间戳============================================
        if imageTimestamp<=radar_pointClouds(1).pc_timestamp
            radarTimestamp = radar_pointClouds(1).pc_timestamp;
            radar_pointCloud = radar_pointClouds(1).radar_pointCloud;
        elseif imageTimestamp>=radar_pointClouds(end).pc_timestamp
            radarTimestamp = radar_pointClouds(end).pc_timestamp;
            radar_pointCloud = radar_pointClouds(end).radar_pointCloud;
        else
            idx_former = 1;
            idx_latter = 2;
            while(imageTimestamp > radar_pointClouds(idx_latter).pc_timestamp)
                idx_former = idx_former + 1;
                idx_latter = idx_latter + 1;
            end
            if (imageTimestamp - radar_pointClouds(idx_former).pc_timestamp) < (radar_pointClouds(idx_latter).pc_timestamp - imageTimestamp)
                radarTimestamp = radar_pointClouds(idx_former).pc_timestamp;
                radar_pointCloud = radar_pointClouds(idx_former).radar_pointCloud;
            else
                radarTimestamp = radar_pointClouds(idx_latter).pc_timestamp;
                radar_pointCloud = radar_pointClouds(idx_latter).radar_pointCloud;
            end
        end
        % ================================================================================== 
    end
end