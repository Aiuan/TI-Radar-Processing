% aifuyuan 20210601 create
% use to match radar processed results with pictures
clear all;
close all;
clc;

%% 参数设置
radarPath = './output/test.mat';
imagePath = 'K:\ourDataset\20210428\images\20210428mode3Group1';

% ========================设置想要观察的时段====================
% imageTimestamp_start = 1619619555539725;%0428mode3Group1环境地面杂波
% imageTimestamp_end = 1619619653492323;

imageTimestamp_start = 1619619573156350;%0428mode3Group1单车被检测成2部分
imageTimestamp_end = 1619619576340088;

% imageTimestamp_start = 1619619800523929;%0428mode3Group1车辆静止检测不到
% imageTimestamp_end = 1619619814356697;
% imageTimestamp_start = 1619619806196790;%0428mode3Group1车辆启动检测到
% imageTimestamp_end = 1619619814356697;

% imageTimestamp_start = 1620962777191675;%0514mode1Group1停车杂点少
% imageTimestamp_end = 1620962782662602;
% imageTimestamp_start = 1620962782662602;%0514mode1Group1启动杂点多
% imageTimestamp_end = 1620962782762864;
% ===========================================================

% 标定矩阵
%舟山标定矩阵
radar_camera_matchMatrix = [978.427720340050, 589.993462843703, 13.1229972166119, -0.840684195154766;...
                                                14.7696499738007, 388.888646655374, -959.947091338099, -137.871686789255;
                                                0.0239198127750308, 0.999635770515324, 0.0124967541002258, 0.0523119353667915];
% % 20210513-20210514玉泉标定矩阵
% radar_camera_matchMatrix = [296.017605935561, 835.703097555440, 33.2444346178279, -386.776196365743;...
%     -196.067005960755, 455.292940171630, -575.779148193414, 184.623265383036;...
%     -0.395085889573456, 0.917177372733613, 0.0518922615176608, 0.788409932235097];


% 按键下一帧
KEY_ON = 1;
% 连续帧
NUM_RESERVE_FRAME = 5;

%% 加载雷达数据
load(radarPath);
% xyz_all 点云坐标
% pc_timestamps PC端时间戳，UNIX16位时间戳
% timestamp 雷达时间戳
% starTime PC端起始时间戳，UNIX16位时间戳
% set_capture_frames 雷达预设记录帧数量
% cnt_frameGlocal 雷达实际记录帧数量
% cnt_processed 程序处理帧数量
radar_pointClouds = struct();
for i = 1:size(xyz_all,2)
    if isempty(xyz_all{i}) || isempty(radar_timestamps{i}) || isempty(pc_timestamps{i})
        continue;
    end
    
    radar_pointClouds(i).radar_pointCloud = xyz_all{i};
    radar_pointClouds(i).radar_timestamps = uint64(radar_timestamps{i});
    radar_pointClouds(i).pc_timestamp = pc_timestamps{i};
    
end

%% 加载图片数据
imageFolder = dir(imagePath);
images = struct();
for i = 3:size(imageFolder,1)
    idx = i-2;
    images(idx).path = [imageFolder(i).folder,filesep,imageFolder(i).name];
    % 16位UNIX时间戳
    images(idx).timestamp =  imageFolder(i).name(1:strfind(imageFolder(i).name,'.')-1);
    images(idx).timestamp = uint64(str2double(images(idx).timestamp));
end

% 矫正imageTimestamp_start和imageTimestamp_end
imageTimestamp_start = radarMatchImage(uint64(imageTimestamp_start), images) ;
imageTimestamp_end = radarMatchImage(uint64(imageTimestamp_end), images) ;

%% 匹配显示
fig = figure(1);
set(gcf,'units','normalized','outerposition',[0.1 0.1 0.8 0.8]);

for i = 1:size(radar_pointClouds, 2)
    
    disp('=====================================================');
    tic;
    
    radarTimestamp = radar_pointClouds(i).pc_timestamp;
    pointCloud = radar_pointClouds(i).radar_pointCloud;
    [imageTimestamp, imagePath] = radarMatchImage(radarTimestamp, images);
    if (imageTimestamp < imageTimestamp_start) || (imageTimestamp > imageTimestamp_end)
        %如果不在感兴趣段内，则跳过
        fprintf('跳过...雷达时间戳 %.6f\n', double(radarTimestamp)/1e6);
        continue;
    end
    
    
    fprintf('雷达时间戳 %.6f\n', double(radarTimestamp)/1e6);
    fprintf('相机时间戳 %.6f\n', double(imageTimestamp)/1e6);
    fprintf('相机滞后时间 %.3f ms\n', (double(imageTimestamp) - double(radarTimestamp))/1e3);
    
    subplot(3,2,[1,2,3,4]);
    imshow(imagePath);
    title(sprintf('图片时间戳: %10d.%06d\n',uint64(floor(double(imageTimestamp)/1000000)),mod(imageTimestamp,1000000)));
    hold on;
    %投影点云至图像
    remove_distance_min = 3;
    remove_distance_max = 75;
    pixel_coordinate = projection(pointCloud, radar_camera_matchMatrix, remove_distance_min, remove_distance_max);
    scatter(pixel_coordinate(1,:), pixel_coordinate(2,:), 70, pixel_coordinate(3,:), '.');
    hold off;
    
    subplot(3,2,5);
    scatter3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),10,(pointCloud(:,4)),'filled');
    c = colorbar;
    c.Label.String = 'velocity (m/s)';
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    axis('image');
    xlim([-50, 50]);
    ylim([0, 50]);
    view([0 90]);     
    title(sprintf('雷达时间戳: %10d.%06d\n',uint64(floor(double(radarTimestamp)/1000000)),mod(radarTimestamp,1000000)));
    
    subplot(3,2,6);
    reserve_frameIds = [i-NUM_RESERVE_FRAME+1:i];
    reserve_frameIds = reserve_frameIds(reserve_frameIds > 0);
    for reserve_frameId = reserve_frameIds
        xyz = radar_pointClouds(reserve_frameId).radar_pointCloud;
        scatter3(xyz(:,1),xyz(:,2),xyz(:,3),10,(xyz(:,4)),'filled');
        hold on;
    end
    hold off;
    c = colorbar;
    c.Label.String = 'velocity (m/s)';
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    axis('image');
    xlim([-50, 50]);
    ylim([0, 50]);
    view([0 90]);     
    title(sprintf('%d consecutive frames of 3D point cloud: ', NUM_RESERVE_FRAME));
    
    pause(0.1);%等待绘图
    time_use = toc;
    disp(['单帧耗时 ',num2str(time_use), 's']);
    
    
    %等待按键
    if KEY_ON == 0
        continue;
    end             
    key = waitforbuttonpress;
    while(key==0)
        key = waitforbuttonpress;
    end
    
    disp('=====================================================');
end





