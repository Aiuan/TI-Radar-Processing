clearvars
close all;
clc;

%% 参数设置
%运行帧数
numFrames_toRun = inf; 
%是否根据原始数据文件夹中的config.mmwave.json文件重新生成参数文件
PARAM_FILE_GEN_ON = 1;
%保存
SAVEOUTPUT_ON = 1;
SAVE_NAME = 'test';

%数据平台类型
dataPlatform = 'TDA2';
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
    %     detectionObj        = CFAR_CASO('pfile', pathGenParaFile);%CFAR_CASO算法
    detectionObj        = CFAR_OS('pfile', pathGenParaFile);%CFAR_OS算法
    DOAObj              = DOACascade('pfile', pathGenParaFile);
    
    platform = simTopObj.platform;%使用平台类型
    numValidFrames = simTopObj.totNumFrames;%设定有效帧数
    
    % 雷达设定采集帧数
    set_capture_frames = simTopObj.totNumFrames;
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
    radar_fps = 1/frameInterval;
    
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
        
        for frameIdx = 1:1:min(numValidFrames,1+numFrames_toRun)
            
             %全局帧计数器+1
            cnt_frameGlobal = cnt_frameGlobal+1;
            
            disp('===========================================================');
            fprintf('正在访问第 %s 片中第 %d/%d 帧（全局的第 %d/%d 帧）\n',  fileIdx_unique{i_file}, frameIdx, numValidFrames,...
                cnt_frameGlobal, set_capture_frames);
            
            
            %============================读取环节==========================================
            tic;%开始计时，计算单帧读取耗费时间
            
            % read and calibrate raw ADC data            
            calibrationObj.frameIdx = frameIdx;
            % 维度（每个Chirp中的采样点数，loop的数目，RX数量，一个loop中chirps的数量（通常与TX数量相等））
            adcData = datapath(calibrationObj);
            % RX Channel re-ordering
            % 根据天线板的位置关系，重新排列天线编号 ：芯片4，芯片1，芯片3，芯片2
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:);
            
            % timestamp
            radar_timestamps{cnt_frameGlobal} = getTimestamp(fullfile(dataFolder_test, fileNameStruct.masterIdxFile), frameIdx);
            if cnt_frameGlobal ==1 || isempty(radar_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(radar_timestamps{cnt_frameGlobal} - radar_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('雷达端时间戳 %d，与上一帧间隔 %.3f ms\n', radar_timestamps{cnt_frameGlobal}, diff_timestamp);
            
            pc_timestamps{cnt_frameGlobal} = startTime + (cnt_frameGlobal-1)*frameInterval*1000000;%16位UNIX时间
            if cnt_frameGlobal ==1 || isempty(pc_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(pc_timestamps{cnt_frameGlobal} - pc_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('电脑端时间戳 %.6f，与上一帧间隔 %.3f ms\n', double(pc_timestamps{cnt_frameGlobal})/1e6, diff_timestamp);
            
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
            
            velocityList = (-size(DopplerFFTOut,2)/2: size(DopplerFFTOut,2)/2-1) * detectionObj.velocityBinSize;
            rangeList = (1: size(rangeFFTOut,1)) * detectionObj.rangeBinSize;    
            
            % 转化为虚拟天线形式，维度（每个Chirp中的采样点数，每根天线发射的chirp数量，虚拟天线数量 ）
            DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));
            
            % CFAR done along only TX and RX used in MIMO array
            % 进行detection（CFAR）处理，datapath位于module：detection
            detection_results = datapath(detectionObj, DopplerFFTOut);
            
            detect_all_points = [];%初始化检测到的点的信息矩阵
            for iobj = 1:length(detection_results)
                detect_all_points (iobj,1)=detection_results(iobj).rangeInd+1;%range index
                detect_all_points (iobj,2)=detection_results(iobj).dopplerInd_org+1;%doppler index
                detect_all_points (iobj,4)=detection_results(iobj).estSNR;%反射强度
            end
                     
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
                    xyz_all{cnt_frameGlobal}  = xyz;            
                end                
            end
                  
            time = toc;
            disp(['处理耗时',num2str(time), 's']);
            disp('===========================================================');
            cnt_processed = cnt_processed + 1; %处理数据计数器+1
            
            
        end%逐帧处理循环结束        
    end%逐片数据处理循环结束
    %============================================================================
    
    %============================保存数据环节==========================================
    if SAVEOUTPUT_ON == 1
        %保存的数据为               
        save(['.\output\', SAVE_NAME, '.mat'], 'radar_timestamps', 'pc_timestamps', 'xyz_all', 'startTime'...
            , 'cnt_processed', 'cnt_frameGlobal', 'set_capture_frames', 'radar_fps'...
            , 'rangeList', 'velocityList');
    end
    %============================================================================
    testID = testID + 1;
end
fclose(fidList);