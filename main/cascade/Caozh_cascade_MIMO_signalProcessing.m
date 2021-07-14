clearvars
% close all;
clc;

%�������Ƿ���Ҫ��ͼչʾ
PLOT_ON = 1; % 1: turn plot on; 0: turn plot off
%������չʾ�Ƿ���ö�������
LOG_ON = 1; % 1: log10 scale; 0: linear scale
%����֡��
numFrames_toRun = 1; %number of frame to run, can be less than the frame saved in the raw data
%���ݴ������Ƿ񱣴�
SAVEOUTPUT_ON = 0;
%�Ƿ����ԭʼ�����ļ����е�config.mmwave.json�ļ��������ɲ����ļ�
PARAM_FILE_GEN_ON = 1;
%��������ͼ
DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP = 0 ; % Will make things slower
%����ƽ̨����
dataPlatform = 'TDA2';

%% get the input path and testList
%����Ŀ¼
pro_path = 'D:\matlab_workspace\mmWaveProcessing\MatlabExamples\4chip_cascade_MIMO_example';
%input�ļ���Ŀ¼
input_path = strcat(pro_path,'\main\cascade\input\');
%���������������ļ�Ŀ¼
testList = strcat(input_path,'testList_caozh.txt');
%��testList.txt�ļ�
fidList = fopen(testList,'r');
%��Ϊ���ɲ����ļ��ı��
testID = 1;

while ~feof(fidList)%���test.List�ļ���Ϊ��
    
    % get each test vectors within the test list
    % test data file name
    dataFolder_test = fgetl(fidList);%ԭʼ�����ļ���·��    
   
    %calibration file name
    dataFolder_calib = fgetl(fidList);%У׼�ļ�·��
    
    %module_param_file defines parameters to init each signal processing
    %module
    module_param_file = fgetl(fidList);%����ԭʼ�����ļ����е�config.mmwave.json�ļ��������ɲ����ļ��Ĵ�������·��
    
     %parameter file name for the test
    pathGenParaFile = [input_path,'test',num2str(testID), '_param.m'];%���ɵĲ����ļ���·��
    %important to clear the same.m file, since Matlab does not clear cache
    %automatically
    clear(pathGenParaFile);%���ԭʼ���ڵ�ͬ���ļ����ƺ�û��
    
    %generate parameter file for the test to run
    if PARAM_FILE_GEN_ON == 1%����������ɲ����ļ��Ŀ��ش�     
        parameter_file_gen_json(dataFolder_test, dataFolder_calib, module_param_file, pathGenParaFile, dataPlatform);
    end
    
    %load calibration parameters
    load(dataFolder_calib)%����У׼�ļ�
    
    % simTopObj is used for top level parameter parsing and data loading and saving
    simTopObj           = simTopCascade('pfile', pathGenParaFile); %�ں���ζ�ȡadc����
    calibrationObj      = calibrationCascade('pfile', pathGenParaFile, 'calibrationfilePath', dataFolder_calib);
    rangeFFTObj         = rangeProcCascade('pfile', pathGenParaFile);%�ں���ζ����ݽ���rangeFFT����
    DopplerFFTObj       = DopplerProcClutterRemove('pfile', pathGenParaFile);%�ں���ζ����ݽ���DopplerFFT����
    detectionObj        = CFAR_CASO('pfile', pathGenParaFile);%�ں�ͨ��angleFFT �� CFAR�㷨��detection���
    DOAObj              = DOACascade('pfile', pathGenParaFile);
    
    % get system level variables
    platform            = simTopObj.platform;%ʹ��ƽ̨����
    numValidFrames      = simTopObj.totNumFrames;%�趨��Ч֡��
    cnt = 1;%���ڼ�¼�����˼�֡�����洢�˽���ļ�����
    frameCountGlobal = 0;%ȫ�ִ����ڼ�֡����
    
    
    % Get Unique File Idxs in the "dataFolder_test"   
    [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);%�����ݱ�ţ�һ��Ϊ0000
   
     
    
    
    for i_file = 1:length(fileIdx_unique)%���Ǽ���ɼ�ģʽ��һ��Ϊlength(fileIdx_unique)=1
        
        % Get File Names for the Master, Slave1, Slave2, Slave3   
        [fileNameStruct]= getBinFileNames_withIdx(dataFolder_test, fileIdx_unique{i_file});%�õ�adc�����ļ���idx�����ļ����ļ���        
       
        %pass the Data File to the calibration Object
        calibrationObj.binfilePath = fileNameStruct;%���������ļ���calibrationObj��binfilePath����
        
        detection_results = [];  %��ʼ�������
        
        % Get Valid Number of Frames
        % ȷ���ж�����Ч֡����Ч֡���ݴ�С����master_0000_idx.bin�ļ��еĵڣ����ֽ���ʾ����Ч֡Ϊ���٣���Ч֡���ݴ�С
        [numValidFrames, dataFileSize] = getValidNumFrames(fullfile(dataFolder_test, fileNameStruct.masterIdxFile));
        %intentionally skip the first frame due to TDA2 
       
        %������һ֡���ӵڶ�֡��ʼ
        for frameIdx = 2:1:min(numValidFrames,1+numFrames_toRun)
            disp('===========================================================');
            fprintf('���ڴ���� %s Ƭ�е� %d ֡...��ȫ�ִ���ĵ� %d/%d ֡��\n', fileIdx_unique{i_file},frameIdx,frameCountGlobal+1,simTopObj.totNumFrames);
            
            tic;%��ʼ��ʱ�����㵥֡����ķ�ʱ��
            %read and calibrate raw ADC data            
            calibrationObj.frameIdx = frameIdx;%��ȡ��У������ǰ֡��ԭʼadc����
            frameCountGlobal = frameCountGlobal+1;%ȫ�ִ���֡������+1
            %��ȡadc����
            adcData = datapath(calibrationObj);%4-D complex double�������ݣ�ά����Ӧ�ڣ�ÿ��Chirp�еĲ���������Loop����Ŀ�����߱�ţ�ÿ��Loop�е�chirp������
            time = toc;
            disp(['��ȡ��ʱ',num2str(time)]);
            
            tic;
            % RX Channel re-ordering
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:); %����chirp�ķ���˳�������������߱��           
            Azimuth_Heatmap_Exp(adcData);
            
          
        end%��֡����ѭ������        
    end%������ݴ���ѭ��������һ��ֻ�С�0000��һ��
    
    
    testID = testID + 1;
    
end
