clearvars
close all;
clc;

%% 参数设置

% 相机数据目录
imagePath = 'K:\ourDataset\20210428\images\20210428mode3Group1';

% ========================设置想要观察的时段====================
% imageTimestamp_start = 0;
% imageTimestamp_end = inf;

% imageTimestamp_start = 1619619555539725;%0428mode3Group1环境地面杂波
% imageTimestamp_end = 1619619653492323;

% imageTimestamp_start = 1619619573156350;%0428mode3Group1单车被检测成2部分
% imageTimestamp_end = 1619619576340088;

imageTimestamp_start = 1619619800523929;%0428mode3Group1车辆静止检测不到
imageTimestamp_end = 1619619814356697;
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


%运行帧数
numFrames_toRun = inf; 
%是否根据原始数据文件夹中的config.mmwave.json文件重新生成参数文件
PARAM_FILE_GEN_ON = 1;
% 等待按键
KEY_ON = 1;
% 残留帧
NUM_RESERVE_FRAME = 10;

%数据平台类型
dataPlatform = 'TDA2';
%% 加载相机数据
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

%% 

%input文件夹目录
input_path = strcat(pwd,'\input\');
%描述待处理数据文件目录
testList = strcat(input_path,'testList.txt');
%打开testList.txt文件
fidList = fopen(testList,'r');
%生成雷达参数文件的编号
testID = 1;

while ~feof(fidList)%如果test.List文件不为空
    %雷达adc数据 路径    
    dataFolder_test = fgetl(fidList);   
    %校准矩阵 路径
    dataFolder_calib = fgetl(fidList);    
    %雷达设置参数模板 路径
    module_param_file = fgetl(fidList);    
    %根据原始adc数据文件夹中的config.mmwave.json文件生成matlab格式雷达参数文件 路径
    pathGenParaFile = [input_path,'test',num2str(testID), '_param.m'];
        
    %重新生成雷达参数文件
    if PARAM_FILE_GEN_ON == 1     
        parameter_file_gen_json(dataFolder_test, dataFolder_calib, module_param_file, pathGenParaFile, dataPlatform);
    end
    
    %load calibration parameters
    % 加载校准文件
    load(dataFolder_calib)
    
    % simTopObj is used for top level parameter parsing and data loading and saving
    simTopObj           = simTopCascade('pfile', pathGenParaFile); %内含如何读取adc数据
    calibrationObj      = calibrationCascade('pfile', pathGenParaFile, 'calibrationfilePath', dataFolder_calib);
    rangeFFTObj         = rangeProcCascade('pfile', pathGenParaFile);%内含如何对数据进行rangeFFT处理
    DopplerFFTObj       = DopplerProcClutterRemove('pfile', pathGenParaFile);%内含如何对数据进行DopplerFFT处理
    detectionObj        = CFAR_CASO('pfile', pathGenParaFile);%内含通过angleFFT 和 CFAR算法的detection结果
    DOAObj              = DOACascade('pfile', pathGenParaFile);
    
    platform = simTopObj.platform;%使用平台类型
    numValidFrames = simTopObj.totNumFrames;%设定有效帧数
    
    % 处理帧计数器，1开始有效
    cnt_processed = 0;
    % 全局第几帧计数，1开始有效
    cnt_frameGlobal = 0;
    % 查询PC端采集开始时间戳
    startTimefile = dir(fullfile(dataFolder_test, '*.startTime.txt'));
    f_startTime = fopen(fullfile(startTimefile.folder, startTimefile.name), 'r');
    startTime = fscanf(f_startTime, '%f');
    fclose(f_startTime);    
    startTime = uint64(startTime*1000000);%16位UNIX时间
    % 获取采集帧间隔时间(单位s)
    frameInterval = getPara(pathGenParaFile, 'framePeriodicty');
    
    %聚合帧计数器
    cnt_reserve_frame = 0;
    
    figure(1);
    set(gcf,'units','normalized','outerposition',[0.1 0.2 0.8 0.8]);
%     set(gcf,'units','normalized','outerposition',[0 0 1 1]);
    
    %获得数据分片数，例：0000, 0001, 0002...   
    [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);    
    for i_file = 1:length(fileIdx_unique)
        
        % Get File Names for the Master, Slave1, Slave2, Slave3  
        % 得到adc数据文件、idx数据文件的文件名     
        [fileNameStruct]= getBinFileNames_withIdx(dataFolder_test, fileIdx_unique{i_file});   
       
        % pass the Data File to the calibration Object
        % 传递 数据文件名至calibrationObj的binfilePath属性
        calibrationObj.binfilePath = fileNameStruct;
        
        % 得有效帧数
        [numValidFrames, ~] = getValidNumFrames(fullfile(dataFolder_test, fileNameStruct.masterIdxFile));
        
        %intentionally skip the first frame due to TDA2
        for frameIdx = 1:1:min(numValidFrames,1+numFrames_toRun)
            
            %全局帧计数器+1
            cnt_frameGlobal = cnt_frameGlobal+1;
            disp('===========================================================');
            fprintf('正在访问第 %s 片中第 %d/%d 帧（全局的第 %d/%d 帧）\n',  fileIdx_unique{i_file}, frameIdx, numValidFrames,...
                cnt_frameGlobal, simTopObj.totNumFrames);
            
            %============================查询时间戳环节====================================
            % timestamp
            radar_timestamps{cnt_frameGlobal} = getTimestamp(fullfile(dataFolder_test, fileNameStruct.masterIdxFile), frameIdx);
            if cnt_frameGlobal ==1 || isempty(radar_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(radar_timestamps{cnt_frameGlobal} - radar_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('雷达端时间戳 %d，与上一帧间隔 %.3f ms\n', radar_timestamps{cnt_frameGlobal}, diff_timestamp);
            % pc_timestamp
            pc_timestamps{cnt_frameGlobal} = startTime + (cnt_frameGlobal-1)*frameInterval*1000000;%16位UNIX时间
            if cnt_frameGlobal ==1 || isempty(pc_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(pc_timestamps{cnt_frameGlobal} - pc_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('电脑端时间戳 %.6f，与上一帧间隔 %.3f ms\n', double(pc_timestamps{cnt_frameGlobal})/1e6, diff_timestamp);
            
            % 寻找对应图片帧时间戳
            radar_timestamp = pc_timestamps{cnt_frameGlobal};
            [image_timestamp, image_path] = radarMatchImage(radar_timestamp, images);
            camera_timestamps{cnt_frameGlobal} = image_timestamp;
            if cnt_frameGlobal ==1 || isempty(camera_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(camera_timestamps{cnt_frameGlobal} - camera_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('相机端时间戳 %.6f，与上一帧间隔 %.3f ms\n', double(camera_timestamps{cnt_frameGlobal})/1e6, diff_timestamp);
            
            if (image_timestamp < imageTimestamp_start) || (image_timestamp > imageTimestamp_end)
                % 若不在寻找的时间段内则跳过      
                fprintf('跳过第 %s 片中第 %d/%d 帧（全局的第 %d/%d 帧）\n',  fileIdx_unique{i_file}, frameIdx, numValidFrames,...
                cnt_frameGlobal, simTopObj.totNumFrames);
                disp('===========================================================');
                continue;                
            end
            %============================================================================            
            
            
            %============================读取环节==========================================
            
            tic;%开始计时，计算单帧读取耗费时间
            
            % read and calibrate raw ADC data            
            calibrationObj.frameIdx = frameIdx;
            % 维度（每个Chirp中的采样点数，loop的数目，RX数量，一个loop中chirps的数量（通常与TX数量相等））
            adcData = datapath(calibrationObj);
            % RX Channel re-ordering
            % 根据天线板的位置关系，重新排列天线编号 ：芯片4，芯片1，芯片3，芯片2
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:);
            
            time = toc;
            disp(['读取耗时 ',num2str(time), 's']);
            %============================================================================
            
            
            %============================信号处理环节==========================================
            tic;
            
            %perform 2D FFT
            rangeFFTOut = [];
            DopplerFFTOut = [];            
            for i_tx = 1: size(adcData,4)%对第i_tx个loop/发射天线进行处理
                % range FFT
                rangeFFTOut(:,:,:,i_tx)     = datapath(rangeFFTObj, adcData(:,:,:,i_tx));%进行rangeFFT处理，datapath位于module：rangeProc
                % Doppler FFT
                DopplerFFTOut(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut(:,:,:,i_tx));%进行DopplerFFT处理，datapath位于module：DopplerProc
            end
            
            % 转化为虚拟天线形式，维度（每个Chirp中的采样点数，每根天线发射的chirp数量，虚拟天线数量 ）
            DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));
            %对DopplerFFTOut在dim=3上求和取对数后得sig_integrate
            sig_integrate = 10*log10(sum((abs(DopplerFFTOut)).^2,3) + 1);
            
            
            % CFAR done along only TX and RX used in MIMO array
            % 进行detection（CFAR）处理，datapath位于module：detection
            detection_results = datapath(detectionObj, DopplerFFTOut);
            
            detect_all_points = [];%初始化检测到的点的信息矩阵
            for iobj = 1:length(detection_results)
                detect_all_points (iobj,1)=detection_results(iobj).rangeInd+1;%range index
                detect_all_points (iobj,2)=detection_results(iobj).dopplerInd_org+1;%doppler index
                detect_all_points (iobj,4)=detection_results(iobj).estSNR;%反射强度
            end
            
            
            % 绘制Doppler Map
            subplot(2,2,3);
            %意义：展示DopplerFFTOut的中间一列，速度为0静态目标的range图
            %横坐标range 纵坐标反射强度
            rangeList = (1: size(sig_integrate,1)) * detectionObj.rangeBinSize;
            powerList_v0 = sig_integrate(:, size(sig_integrate,2)/2+1);
            plot(rangeList, powerList_v0, 'g', 'LineWidth', 4);
            hold on;
            grid on;
            for ii=1:size(sig_integrate,2)%DopplerFFTOut的第ii列，速度不为0的列
                powerList_vii = sig_integrate(:,ii);
                plot(rangeList, powerList_vii);
                hold on;
                grid on;
                if ~isempty(detection_results)%如果通过CFAR算法检测到了目标
                    ind = find(detect_all_points(:,2)==ii);%找到当前Doppler检测速度对应的检测点
                    if (~isempty(ind))%如果有监测点
                        rangeInd = detect_all_points(ind,1);%取检测到的点的range
                        plot(rangeInd*detectionObj.rangeBinSize, sig_integrate(rangeInd,ii),'o','LineWidth',2,...
                            'MarkerEdgeColor','k',...
                            'MarkerFaceColor',[.49 1 .63],...
                            'MarkerSize',6);
                    end
                end
            end
            xlabel('Range(m)');
            ylabel('Receive Power (dB)')
            title(sprintf('雷达时间戳: %10d.%06d\n',uint64(floor(double(radar_timestamp)/1000000)),mod(radar_timestamp,1000000)));
            hold off;

                      
            angles_all_points = [];%初始化检测点方位角矩阵
            xyz = [];%初始化检测点空间坐标矩阵
            
            if ~isempty(detection_results)%如果检测到点
                % DOA, the results include detection results + angle estimation results.
                % access data with angleEst{frame}(objectIdx).fieldName
                angleEst = datapath(DOAObj, detection_results);%进行angle estimation
                
                if length(angleEst) > 0%如果检测到角度结果
                    for iobj = 1:length(angleEst)%对第iobj个检测结果
                        % angleEst.angles有4个值，取前两个：第一个为方位角azimuth，第二个为高程角elvation
                        angles_all_points (iobj,1)=angleEst(iobj).angles(1);%方位角azimuth
                        % 原先z值与现实坐标反了，原因是这里的elevation正负号出错了，因此于20210531修正
                        angles_all_points (iobj,2)=-angleEst(iobj).angles(2);%高程角elvation
                        angles_all_points (iobj,3)=angleEst(iobj).estSNR;%预测信噪比
                        angles_all_points (iobj,4)=angleEst(iobj).rangeInd;%取range的Index
                        angles_all_points (iobj,5)=angleEst(iobj).doppler_corr;%取doppler_corr
                        angles_all_points (iobj,6)=angleEst(iobj).range;%取range
                        
                        xyz(iobj,1) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,1))*cosd(angles_all_points (iobj,2));%x
                        xyz(iobj,2) = angles_all_points (iobj,6)*cosd(angles_all_points (iobj,1))*cosd(angles_all_points (iobj,2));%y
                        % switch upside and down, the elevation angle is flipped
                        % 原先这里z值与现实坐标反了，原因是elevation正负号出错了，已于20210531修正
                        xyz(iobj,3) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,2));%z
                        xyz(iobj,4) = angleEst(iobj).doppler_corr;%取doppler_corr
                        xyz(iobj,9) = angleEst(iobj).dopplerInd_org;%取dopplerInd_org
                        xyz(iobj,5) = angleEst(iobj).range;%取range
                        xyz(iobj,6) = angleEst(iobj).estSNR;%取估计信噪比
                        xyz(iobj,7) = angleEst(iobj).doppler_corr_overlap;%取doppler_corr_overlap
                        xyz(iobj,8) = angleEst(iobj).doppler_corr_FFT;%取doppler_corr_FFT
                    end
                    
                    moveID = find(abs(xyz(:,4))>=0);%检查abs(doppler_corr)大于等于0的目标
                    
                    % 绘制单帧点云图
                    subplot(2,2,4);
                    %定义坐标系为y是雷达板正前方
                    scatter3(xyz(moveID,1), xyz(moveID,2), xyz(moveID,3), 10, (xyz(moveID,4)),'filled');%x,y,z,doppler_corr
                    c = colorbar;
                    c.Label.String = 'velocity (m/s)';                        
                    grid on;
                    xlabel('X (m)');
                    ylabel('Y (m)');
                    zlabel('Z (m)');
                    axis('image');
                    maxRangeShow = detectionObj.rangeBinSize*rangeFFTObj.rangeFFTSize;
                    xlim([-maxRangeShow maxRangeShow]);
                    ylim([0 maxRangeShow]);
                    colormap('jet');
                    view([0 90]);                        
                    title(sprintf('雷达时间戳: %10d.%06d\n',uint64(floor(double(radar_timestamp)/1000000)),mod(radar_timestamp,1000000)));
                    
                    % 显示对应图片 + 投影点云至图像
                    remove_distance_min = 3;
                    remove_distance_max = 75;
                    pixel_coordinate = projection(xyz, radar_camera_matchMatrix, remove_distance_min, remove_distance_max);
                    subplot(2,2,1);
                    imshow(image_path);
                    title(sprintf('相机时间戳: %10d.%06d\n',uint64(floor(double(image_timestamp)/1000000)),mod(image_timestamp,1000000)));
                    hold on;
                    scatter(pixel_coordinate(1,:), pixel_coordinate(2,:), 70, pixel_coordinate(3,:), '.');
                    hold off;
                    
                    
                    %绘制多帧聚合点云图
                    if cnt_reserve_frame+1 <= NUM_RESERVE_FRAME
                        % 当残留帧数不超过设置时
                        cnt_reserve_frame = cnt_reserve_frame+1;
                    else
                        % 当残留帧数超过设置时
                        cnt_reserve_frame = 1;
                    end
                    constant_pointClouds{cnt_reserve_frame} = xyz;
                    subplot(2,2,2);
                    for reserve_frameId = 1:size(constant_pointClouds,2)
                        xyz = constant_pointClouds{reserve_frameId};
                        scatter3(xyz(:,1),xyz(:,2),-xyz(:,3),10,(xyz(:,4)),'filled');
                        hold on;
                    end
                    c = colorbar;
                    caxis([-detectionObj.dopplerFFTSize/2*detectionObj.velocityBinSize, (detectionObj.dopplerFFTSize/2-1)*detectionObj.velocityBinSize]);
                    c.Label.String = 'velocity (m/s)';                    
                    grid on;
                    xlabel('X (m)');
                    ylabel('Y (m)');
                    zlabel('Z (m)');
                    axis('image'); 
                    xlim([-maxRangeShow maxRangeShow]);
                    ylim([0 maxRangeShow])
                    view([0 90]);              
                    title(sprintf('%d 帧聚合点云图: ', NUM_RESERVE_FRAME));
                    hold off;
                    
                    pause(0.1); 
                end                
            end
                  
            time = toc;
            disp(['处理耗时',num2str(time), 's']);
            disp('===========================================================');
            cnt_processed = cnt_processed + 1; %处理数据计数器+1
            
            %等待按键
            if KEY_ON == 0
                continue;
            end             
            key = waitforbuttonpress;
            while(key==0)
                key = waitforbuttonpress;
            end
            
        end%逐帧处理循环结束        
    end%逐片数据处理循环结束
    %============================================================================
    
    testID = testID + 1;
end
fclose(fidList);