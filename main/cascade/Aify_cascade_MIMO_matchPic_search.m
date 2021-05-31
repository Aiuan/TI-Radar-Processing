clearvars
close all;
clc;

%% 加载图片文件，及时间戳
imageFolder = dir('K:\ourDataset\20210514\images\20210514mode1Group1');
images = struct();
for i = 3:size(imageFolder,1)
    idx = i-2;
    images(idx).path = [imageFolder(i).folder,filesep,imageFolder(i).name];
    % 16位UNIX时间戳
    images(idx).timestamp =  imageFolder(i).name(1:strfind(imageFolder(i).name,'.')-1);
    images(idx).timestamp = uint64(str2double(images(idx).timestamp));
end


%% 运行参数设置
% 设置想要检测的时段

% image_start_timestamp = 1619619555539725;%0428mode3Group1环境地面杂波
% image_end_timestamp = 1619619653492323;

% image_start_timestamp = 1619619573156350;%0428mode3Group1单车被检测成2部分
% image_end_timestamp = 1619619576340088;

% image_start_timestamp = 1619619800523929;%0428mode3Group1车辆静止检测不到
% image_end_timestamp = 1619619814356697;
% image_start_timestamp = 1619619806196790;%0428mode3Group1车辆启动检测到
% image_end_timestamp = 1619619814356697;

% image_start_timestamp = 1620962777191675;%0514mode1Group1停车杂点少
% image_end_timestamp = 1620962782662602;
image_start_timestamp = 1620962782662602;%0514mode1Group1启动杂点多
image_end_timestamp = 1620962782762864;


image_start_timestamp = radarMatchImage(uint64(image_start_timestamp), images) ;
image_end_timestamp = radarMatchImage(uint64(image_end_timestamp), images) ;

% 等待按键
KEY_ON = 1;
% 残留帧
NUM_RESERVE_FRAME = 10;

%是否根据原始数据文件夹中的config.mmwave.json文件重新生成参数文件
PARAM_FILE_GEN_ON = 1;
%数据平台类型
dataPlatform = 'TDA2';

% 路测10FPS
% 车载20FPS
radar_FPS = 20;




%% get the input path and testList
%程序目录
pro_path = 'D:\matlab_workspace\mmWaveProcessing\MatlabExamples\4chip_cascade_MIMO_example';
%input文件夹目录
input_path = strcat(pro_path,'\main\cascade\input\');
%描述待处理数据文件目录
testList = strcat(input_path,'testList.txt');
%打开testList.txt文件
fidList = fopen(testList,'r');
%作为生成参数文件的编号
testID = 1;

while ~feof(fidList)%如果test.List文件不为空
    
    % get each test vectors within the test list
    % test data file name
    dataFolder_test = fgetl(fidList);%原始数据文件夹路径   
    
    % 查询雷达采集开始时间戳，及FPS
    startTimefile = dir(fullfile(dataFolder_test, '*.startTime.txt'));
    f_startTime = fopen(fullfile(startTimefile.folder, startTimefile.name), 'r');
    radar_startTime = fscanf(f_startTime, '%f');
    fclose(f_startTime);    
    radar_startTime = uint64(radar_startTime*1000000); %16位UNIX时间戳 
    
   
    %calibration file name
    dataFolder_calib = fgetl(fidList);%校准文件路径
    
    %module_param_file defines parameters to init each signal processing
    %module
    module_param_file = fgetl(fidList);%根据原始数据文件夹中的config.mmwave.json文件重新生成参数文件的处理程序的路径
    
     %parameter file name for the test
    pathGenParaFile = [input_path,'test',num2str(testID), '_param.m'];%生成的参数文件的路径
    
    clear(pathGenParaFile);%清空原始存在的同名文件，似乎没用
    
    %generate parameter file for the test to run
    if PARAM_FILE_GEN_ON == 1%如果重新生成参数文件的开关打开     
        parameter_file_gen_json(dataFolder_test, dataFolder_calib, module_param_file, pathGenParaFile, dataPlatform);
    end
    
    %load calibration parameters
    load(dataFolder_calib)%加载校准文件
    
    % simTopObj is used for top level parameter parsing and data loading and saving
    simTopObj           = simTopCascade('pfile', pathGenParaFile); %内含如何读取adc数据
    calibrationObj      = calibrationCascade('pfile', pathGenParaFile, 'calibrationfilePath', dataFolder_calib);
    rangeFFTObj         = rangeProcCascade('pfile', pathGenParaFile);%内含如何对数据进行rangeFFT处理
    DopplerFFTObj       = DopplerProcClutterRemove('pfile', pathGenParaFile);%内含如何对数据进行DopplerFFT处理
    detectionObj        = CFAR_CASO('pfile', pathGenParaFile);%内含通过angleFFT 和 CFAR算法的detection结果
    DOAObj              = DOACascade('pfile', pathGenParaFile);
    
    % get system level variables
    platform            = simTopObj.platform;%使用平台类型
    numValidFrames      = simTopObj.totNumFrames;%设定有效帧数
    cnt = 0;%用于记录处理了几帧，并存储了结果的计数器
    frameCountGlobal = 0;%全局第几帧计数
    radar_timestamp = radar_startTime;%雷达起始时间戳
    cnt_reserve_frame = 0;%残留帧计数器
    
    % Get Unique File Idxs in the "dataFolder_test"   
    [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);%得数据编号，一般为0000
   
     figure(1);
%      set(gcf,'units','normalized','outerposition',[0.1 0.1 0.8 0.8]);
     set(gcf,'units','normalized','outerposition',[0 0 1 1]);
    
    
    for i_file = 1:length(fileIdx_unique)%不是间隔采集模式，一般为length(fileIdx_unique)=1
        
        % Get File Names for the Master, Slave1, Slave2, Slave3   
        [fileNameStruct]= getBinFileNames_withIdx(dataFolder_test, fileIdx_unique{i_file});%得到adc数据文件、idx数据文件的文件名        
       
        %pass the Data File to the calibration Object
        calibrationObj.binfilePath = fileNameStruct;%传递数据文件至calibrationObj的binfilePath属性
        
        detection_results = [];  %初始化检测结果
        
        % Get Valid Number of Frames
        % 确定有多少有效帧，有效帧数据大小，在master_0000_idx.bin文件中的第？个字节显示，有效帧为多少，有效帧数据大小
        [numValidFrames, dataFileSize] = getValidNumFrames(fullfile(dataFolder_test, fileNameStruct.masterIdxFile));
        %intentionally skip the first frame due to TDA2 
       
        %舍弃第一帧，从第二帧起始
        for frameIdx = 1:1:numValidFrames
            
            radar_timestamp = radar_startTime + frameCountGlobal * 1e6/radar_FPS;%计算当前雷达帧时间戳
            [image_timestamp, image_path] = radarMatchImage(radar_timestamp, images);%寻找对应图片帧时间戳
            if (image_timestamp < image_start_timestamp)  || (image_timestamp > image_end_timestamp) 
                disp('===========================================================');
                fprintf('跳过第 %s 片中第 %d 帧...（全局的第 %d/%d 帧）\n', fileIdx_unique{i_file},frameIdx,frameCountGlobal+1,simTopObj.totNumFrames);
                frameCountGlobal = frameCountGlobal+1;
                % 若不在寻找的时间段内则跳过                
                continue;
            end
            
            
            disp('===========================================================');
            fprintf('正在处理第 %s 片中第 %d 帧...（全局的第 %d/%d 帧，处理的第%d帧）\n', fileIdx_unique{i_file},frameIdx,frameCountGlobal+1,simTopObj.totNumFrames, cnt);
            
            
            fprintf('雷达帧时间戳 >>> %10d.%06d\n',uint64(floor(double(radar_timestamp)/1000000)),mod(radar_timestamp,1000000));
            
           
            fprintf('相机帧时间戳 >>> %10d.%06d\n',uint64(floor(double(image_timestamp)/1000000)),mod(image_timestamp,1000000));
            
            subplot(2,2,1);%显示对应图片
            imshow(image_path);
            title(sprintf('%10d.%06d\n',uint64(floor(double(image_timestamp)/1000000)),mod(image_timestamp,1000000)));
            
            tic;%开始计时，计算单帧处理耗费时间
            %read and calibrate raw ADC data            
            calibrationObj.frameIdx = frameIdx;%读取并校正，当前帧，原始adc数据
            frameCountGlobal = frameCountGlobal+1;%全局处理帧计数器+1
            %读取adc数据
            adcData = datapath(calibrationObj);%4-D complex double类型数据，维数对应于（每个Chirp中的采样点数，Loop的数目，天线编号，每个Loop中的chirp数量）
            time = toc;
            disp(['读取耗时',num2str(time)]);
            
            tic;
            % RX Channel re-ordering
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:); %根据RX的位置，重新排列【芯片4，芯片1，芯片3，芯片2】           
            
            %perform 2D FFT
            rangeFFTOut = [];
            DopplerFFTOut = [];
            
            for i_tx = 1: size(adcData,4)%对第i_tx个发射天线进行处理
                % range FFT
                rangeFFTOut(:,:,:,i_tx)     = datapath(rangeFFTObj, adcData(:,:,:,i_tx));%进行rangeFFT处理，datapath位于module：rangeProc
                
                % Doppler FFT
                DopplerFFTOut(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut(:,:,:,i_tx));%进行DopplerFFT处理，datapath位于module：DopplerProc
                
            end
            
            % CFAR done along only TX and RX used in MIMO array
            DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));
            
            %detection
            sig_integrate = 10*log10(sum((abs(DopplerFFTOut)).^2,3) + 1);%对DopplerFFTOut在dim=3上求和取对数后得sig_integrate

                        
            detection_results = datapath(detectionObj, DopplerFFTOut);%进行detection处理，datapath位于module：detection，
            
            detect_all_points = [];%初始化检测到的点的信息矩阵
            for iobj = 1:length(detection_results)
                detect_all_points (iobj,1)=detection_results(iobj).rangeInd+1;%取range信息
                detect_all_points (iobj,2)=detection_results(iobj).dopplerInd_org+1;%取Doppler信息
                detect_all_points (iobj,4)=detection_results(iobj).estSNR;%取反射强度信息
            end
            
            subplot(2,2,3);
            plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,size(sig_integrate,2)/2+1),'g','LineWidth',4);
            hold on; 
            grid on;
            for ii=1:size(sig_integrate,2)%DopplerFFTOut的第ii列   
                plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,ii));hold on; grid on
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
            title(sprintf('%10d.%06d\n',uint64(floor(double(radar_timestamp)/1000000)),mod(radar_timestamp,1000000)));
            hold off;
                                    
            angles_all_points = [];%初始化检测点方位角矩阵
            xyz = [];%初始化检测点空间坐标矩阵
            
            if ~isempty(detection_results)%如果检测到点
                % DOA, the results include detection results + angle estimation results.
                % access data with angleEst{frame}(objectIdx).fieldName
                angleEst = datapath(DOAObj, detection_results);%进行angle estimation
                
                if length(angleEst) > 0%如果检测到角度结果
                    for iobj = 1:length(angleEst)%对第iobj个检测结果
                        angles_all_points (iobj,1:2)=angleEst(iobj).angles(1:2);%angleEst.angles有4个值，取前两个：第一个为方位角，第二个为高程角
                        angles_all_points (iobj,3)=angleEst(iobj).estSNR;%预测信噪比
                        angles_all_points (iobj,4)=angleEst(iobj).rangeInd;%取range的Index
                        angles_all_points (iobj,5)=angleEst(iobj).doppler_corr;%取doppler_corr
                        angles_all_points (iobj,6)=angleEst(iobj).range;%取range
                        % switch left and right, the azimuth angle is flipped
                        xyz(iobj,1) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,1))*cosd(angles_all_points (iobj,2));%x
                        xyz(iobj,2) = angles_all_points (iobj,6)*cosd(angles_all_points (iobj,1))*cosd(angles_all_points (iobj,2));%y
                        % switch upside and down, the elevation angle is flipped
                        xyz(iobj,3) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,2));%z
                        xyz(iobj,4) = angleEst(iobj).doppler_corr;%取doppler_corr
                        xyz(iobj,9) = angleEst(iobj).dopplerInd_org;%取dopplerInd_org
                        xyz(iobj,5) = angleEst(iobj).range;%取range
                        xyz(iobj,6) = angleEst(iobj).estSNR;%取反射强度
                        xyz(iobj,7) = angleEst(iobj).doppler_corr_overlap;%取doppler_corr_overlap
                        xyz(iobj,8) = angleEst(iobj).doppler_corr_FFT;%取doppler_corr_FFT
                        
                    end
                    
                    %绘制当前帧点云图
                    subplot(2,2,4);                        
                    % 对反的z轴的值修正-xyz(moveID,3)
                    scatter3(xyz(:,1),xyz(:,2),-xyz(:,3),10,(xyz(:,4)),'filled');%x,y,z,doppler_corr
                    c = colorbar;
                    caxis([-detectionObj.dopplerFFTSize/2*detectionObj.velocityBinSize, (detectionObj.dopplerFFTSize/2-1)*detectionObj.velocityBinSize]);
                    c.Label.String = 'velocity (m/s)';
                    load('mycolormap.mat');
%                     colormap(jet);
%                     colormap(colormap_white);
                    colormap(colormap_black);  
                    grid on;
                    xlabel('X (m)');
                    ylabel('Y (m)');
                    zlabel('Z (m)');
                    axis('image');
                    maxRangeShow = detectionObj.rangeBinSize*rangeFFTObj.rangeFFTSize;
                    xlim([-maxRangeShow maxRangeShow]);
                    ylim([0 maxRangeShow])%定义坐标系为y是雷达板正前方
%                     view([28 28]);  
                    view([0 90]); 
                    title(' 3D point cloud');
                    
                    %投影点云至图像
                    pixel_coordinate = projection(xyz, 3,75);
                    subplot(2,2,1);
                    hold on;
                    scatter(pixel_coordinate(1,:),...
                        pixel_coordinate(2,:),...
                        70, pixel_coordinate(3,:), '.');
                    
                    
                    %绘制残留点云图 
                    if cnt_reserve_frame+1 <= NUM_RESERVE_FRAME
                        % 当残留帧数不超过设置时
                        cnt_reserve_frame = cnt_reserve_frame+1;
                    else
                        % 当残留帧数超过设置时
                        cnt_reserve_frame = 1;
                    end
                    pointCloud{cnt_reserve_frame} = xyz;                    
                    
                    subplot(2,2,2);
                    for reserve_frameId = 1:size(pointCloud,2)
                        xyz = pointCloud{reserve_frameId};
                        scatter3(xyz(:,1),xyz(:,2),-xyz(:,3),10,(xyz(:,4)),'filled');%x,y,z,doppler_corr
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
                    ylim([0 maxRangeShow])%定义坐标系为y是雷达板正前方
                    view([0 90]);              
                    title(sprintf('%d consecutive frames of 3D point cloud: ', NUM_RESERVE_FRAME));
                    hold off;
                    
                    pause(0.1); 
                                        
                end                
            end
                             
            cnt = cnt + 1; %处理数据和记录数据的计数器+1   
            time = toc;
            disp(['处理耗时',num2str(time)]);
            disp('===========================================================');
            
            
            %等待按键
            if KEY_ON == 0
                continue;
            end             
            key = waitforbuttonpress;
            while(key==0)
                key = waitforbuttonpress;
            end            
            disp('next frame.');
                        
            
        end%逐帧处理循环结束        
    end%逐段数据处理循环结束，一般只有“0000”一段
        
    testID = testID + 1;
    
end
