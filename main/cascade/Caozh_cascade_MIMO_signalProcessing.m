clearvars
close all;
clc;

%处理结果是否需要画图展示
PLOT_ON = 1; % 1: turn plot on; 0: turn plot off
%处理结果展示是否采用对数坐标
LOG_ON = 1; % 1: log10 scale; 0: linear scale
%运行帧数
numFrames_toRun = 1; %number of frame to run, can be less than the frame saved in the raw data
%数据处理结果是否保存
SAVEOUTPUT_ON = 0;
%是否根据原始数据文件夹中的config.mmwave.json文件重新生成参数文件
PARAM_FILE_GEN_ON = 1;
%绘制热力图
DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP = 0 ; % Will make things slower
%数据平台类型
dataPlatform = 'TDA2';

%% get the input path and testList
%程序目录
pro_path = 'D:\matlab_workspace\毫米波雷达后处理程序\MatlabExamples\4chip_cascade_MIMO_example';
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
   
    %calibration file name
    dataFolder_calib = fgetl(fidList);%校准文件路径
    
    %module_param_file defines parameters to init each signal processing
    %module
    module_param_file = fgetl(fidList);%根据原始数据文件夹中的config.mmwave.json文件重新生成参数文件的处理程序的路径
    
     %parameter file name for the test
    pathGenParaFile = [input_path,'test',num2str(testID), '_param.m'];%生成的参数文件的路径
    %important to clear the same.m file, since Matlab does not clear cache
    %automatically
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
    cnt = 1;%用于记录处理了几帧，并存储了结果的计数器
    frameCountGlobal = 0;%全局处理到第几帧计数
    
    
    % Get Unique File Idxs in the "dataFolder_test"   
    [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);%得数据编号，一般为0000
   
     
    
    
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
        for frameIdx = 2:1:min(numValidFrames,1+numFrames_toRun)
            disp('===========================================================');
            fprintf('正在处理第 %s 片中第 %d 帧...（全局处理的第 %d/%d 帧）\n', fileIdx_unique{i_file},frameIdx,frameCountGlobal+1,simTopObj.totNumFrames);
            
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
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:); %根据chirp的发射顺序，重新排列天线编号           
            Azimuth_Heatmap_Exp(adcData);
            
          
        end%逐帧处理循环结束        
    end%逐段数据处理循环结束，一般只有“0000”一段
    
    
    testID = testID + 1;
    
end
