clearvars
close all;
clc;

%% ��������
%����֡��
numFrames_toRun = inf; 
%�Ƿ����ԭʼ�����ļ����е�config.mmwave.json�ļ��������ɲ����ļ�
PARAM_FILE_GEN_ON = 1;
%����
SAVEOUTPUT_ON = 1;
SAVE_NAME = 'test';

%����ƽ̨����
dataPlatform = 'TDA2';
%% 

%input�ļ���Ŀ¼
input_path = strcat(pwd,'\input\');
%���������������ļ�Ŀ¼
testList = strcat(input_path,'testList.txt');
%��testList.txt�ļ�
fidList = fopen(testList,'r');
%�����״�����ļ��ı��
testID = 1;

while ~feof(fidList)%���test.List�ļ���Ϊ��
    %�״�adc���� ·��    
    dataFolder_test = fgetl(fidList);   
    %У׼���� ·��
    dataFolder_calib = fgetl(fidList);    
    %�״����ò���ģ�� ·��
    module_param_file = fgetl(fidList);    
    %����ԭʼadc�����ļ����е�config.mmwave.json�ļ�����matlab��ʽ�״�����ļ� ·��
    pathGenParaFile = [input_path,'test',num2str(testID), '_param.m'];
        
    %���������״�����ļ�
    if PARAM_FILE_GEN_ON == 1     
        parameter_file_gen_json(dataFolder_test, dataFolder_calib, module_param_file, pathGenParaFile, dataPlatform);
    end
    
    %load calibration parameters
    % ����У׼�ļ�
    load(dataFolder_calib)
    
    % simTopObj is used for top level parameter parsing and data loading and saving
    simTopObj           = simTopCascade('pfile', pathGenParaFile); %�ں���ζ�ȡadc����
    calibrationObj      = calibrationCascade('pfile', pathGenParaFile, 'calibrationfilePath', dataFolder_calib);
    rangeFFTObj         = rangeProcCascade('pfile', pathGenParaFile);%�ں���ζ����ݽ���rangeFFT����
    DopplerFFTObj       = DopplerProcClutterRemove('pfile', pathGenParaFile);%�ں���ζ����ݽ���DopplerFFT����
    %     detectionObj        = CFAR_CASO('pfile', pathGenParaFile);%CFAR_CASO�㷨
    detectionObj        = CFAR_OS('pfile', pathGenParaFile);%CFAR_OS�㷨
    DOAObj              = DOACascade('pfile', pathGenParaFile);
    
    platform = simTopObj.platform;%ʹ��ƽ̨����
    numValidFrames = simTopObj.totNumFrames;%�趨��Ч֡��
    
    % �״��趨�ɼ�֡��
    set_capture_frames = simTopObj.totNumFrames;
    % ����֡��������1��ʼ��Ч
    cnt_processed = 0;
    % ȫ�ֵڼ�֡������1��ʼ��Ч
    cnt_frameGlobal = 0;
    % ��ѯPC�˲ɼ���ʼʱ���
    startTimefile = dir(fullfile(dataFolder_test, '*.startTime.txt'));
    f_startTime = fopen(fullfile(startTimefile.folder, startTimefile.name), 'r');
    startTime = fscanf(f_startTime, '%f');
    fclose(f_startTime);    
    startTime = uint64(startTime*1000000);%16λUNIXʱ��
    % ��ȡ�ɼ�֡���ʱ��(��λs)
    frameInterval = getPara(pathGenParaFile, 'framePeriodicty');
    radar_fps = 1/frameInterval;
    
    %������ݷ�Ƭ��������0000, 0001, 0002...   
    [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);    
    for i_file = 1:length(fileIdx_unique)
        
        % Get File Names for the Master, Slave1, Slave2, Slave3  
        % �õ�adc�����ļ���idx�����ļ����ļ���     
        [fileNameStruct]= getBinFileNames_withIdx(dataFolder_test, fileIdx_unique{i_file});   
       
        % pass the Data File to the calibration Object
        % ���� �����ļ�����calibrationObj��binfilePath����
        calibrationObj.binfilePath = fileNameStruct;
        
        % ����Ч֡��
        [numValidFrames, ~] = getValidNumFrames(fullfile(dataFolder_test, fileNameStruct.masterIdxFile));
        
        for frameIdx = 1:1:min(numValidFrames,1+numFrames_toRun)
            
             %ȫ��֡������+1
            cnt_frameGlobal = cnt_frameGlobal+1;
            
            disp('===========================================================');
            fprintf('���ڷ��ʵ� %s Ƭ�е� %d/%d ֡��ȫ�ֵĵ� %d/%d ֡��\n',  fileIdx_unique{i_file}, frameIdx, numValidFrames,...
                cnt_frameGlobal, set_capture_frames);
            
            
            %============================��ȡ����==========================================
            tic;%��ʼ��ʱ�����㵥֡��ȡ�ķ�ʱ��
            
            % read and calibrate raw ADC data            
            calibrationObj.frameIdx = frameIdx;
            % ά�ȣ�ÿ��Chirp�еĲ���������loop����Ŀ��RX������һ��loop��chirps��������ͨ����TX������ȣ���
            adcData = datapath(calibrationObj);
            % RX Channel re-ordering
            % �������߰��λ�ù�ϵ�������������߱�� ��оƬ4��оƬ1��оƬ3��оƬ2
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:);
            
            % timestamp
            radar_timestamps{cnt_frameGlobal} = getTimestamp(fullfile(dataFolder_test, fileNameStruct.masterIdxFile), frameIdx);
            if cnt_frameGlobal ==1 || isempty(radar_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(radar_timestamps{cnt_frameGlobal} - radar_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('�״��ʱ��� %d������һ֡��� %.3f ms\n', radar_timestamps{cnt_frameGlobal}, diff_timestamp);
            
            pc_timestamps{cnt_frameGlobal} = startTime + (cnt_frameGlobal-1)*frameInterval*1000000;%16λUNIXʱ��
            if cnt_frameGlobal ==1 || isempty(pc_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(pc_timestamps{cnt_frameGlobal} - pc_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('���Զ�ʱ��� %.6f������һ֡��� %.3f ms\n', double(pc_timestamps{cnt_frameGlobal})/1e6, diff_timestamp);
            
            time = toc;
            disp(['��ȡ��ʱ ',num2str(time), 's']);
            %============================================================================
            
            
            %============================�źŴ�����==========================================
            tic;
            %perform 2D FFT
            rangeFFTOut = [];
            DopplerFFTOut = [];            
            for i_tx = 1: size(adcData,4)%�Ե�i_tx��loop/�������߽��д���
                % range FFT
                rangeFFTOut(:,:,:,i_tx)     = datapath(rangeFFTObj, adcData(:,:,:,i_tx));%����rangeFFT����datapathλ��module��rangeProc
                % Doppler FFT
                DopplerFFTOut(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut(:,:,:,i_tx));%����DopplerFFT����datapathλ��module��DopplerProc
            end
            
            velocityList = (-size(DopplerFFTOut,2)/2: size(DopplerFFTOut,2)/2-1) * detectionObj.velocityBinSize;
            rangeList = (1: size(rangeFFTOut,1)) * detectionObj.rangeBinSize;    
            
            % ת��Ϊ����������ʽ��ά�ȣ�ÿ��Chirp�еĲ���������ÿ�����߷����chirp������������������ ��
            DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));
            
            % CFAR done along only TX and RX used in MIMO array
            % ����detection��CFAR������datapathλ��module��detection
            detection_results = datapath(detectionObj, DopplerFFTOut);
            
            detect_all_points = [];%��ʼ����⵽�ĵ����Ϣ����
            for iobj = 1:length(detection_results)
                detect_all_points (iobj,1)=detection_results(iobj).rangeInd+1;%range index
                detect_all_points (iobj,2)=detection_results(iobj).dopplerInd_org+1;%doppler index
                detect_all_points (iobj,4)=detection_results(iobj).estSNR;%����ǿ��
            end
                     
            angles_all_points = [];%��ʼ�����㷽λ�Ǿ���
            xyz = [];%��ʼ������ռ��������
            
            if ~isempty(detection_results)%�����⵽��
                % DOA, the results include detection results + angle estimation results.
                % access data with angleEst{frame}(objectIdx).fieldName
                angleEst = datapath(DOAObj, detection_results);%����angle estimation
                
                if length(angleEst) > 0%�����⵽�ǶȽ��
                    for iobj = 1:length(angleEst)%�Ե�iobj�������
                        % angleEst.angles��4��ֵ��ȡǰ��������һ��Ϊ��λ��azimuth���ڶ���Ϊ�߳̽�elvation
                        angles_all_points (iobj,1)=angleEst(iobj).angles(1);%��λ��azimuth
                        % ԭ��zֵ����ʵ���귴�ˣ�ԭ���������elevation�����ų����ˣ������20210531����
                        angles_all_points (iobj,2)=-angleEst(iobj).angles(2);%�߳̽�elvation
                        angles_all_points (iobj,3)=angleEst(iobj).estSNR;%Ԥ�������
                        angles_all_points (iobj,4)=angleEst(iobj).rangeInd;%ȡrange��Index
                        angles_all_points (iobj,5)=angleEst(iobj).doppler_corr;%ȡdoppler_corr
                        angles_all_points (iobj,6)=angleEst(iobj).range;%ȡrange
                        
                        xyz(iobj,1) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,1))*cosd(angles_all_points (iobj,2));%x
                        xyz(iobj,2) = angles_all_points (iobj,6)*cosd(angles_all_points (iobj,1))*cosd(angles_all_points (iobj,2));%y
                        % switch upside and down, the elevation angle is flipped
                        % ԭ������zֵ����ʵ���귴�ˣ�ԭ����elevation�����ų����ˣ�����20210531����
                        xyz(iobj,3) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,2));%z
                        xyz(iobj,4) = angleEst(iobj).doppler_corr;%ȡdoppler_corr
                        xyz(iobj,9) = angleEst(iobj).dopplerInd_org;%ȡdopplerInd_org
                        xyz(iobj,5) = angleEst(iobj).range;%ȡrange
                        xyz(iobj,6) = angleEst(iobj).estSNR;%ȡ���������
                        xyz(iobj,7) = angleEst(iobj).doppler_corr_overlap;%ȡdoppler_corr_overlap
                        xyz(iobj,8) = angleEst(iobj).doppler_corr_FFT;%ȡdoppler_corr_FFT
                    end
                    xyz_all{cnt_frameGlobal}  = xyz;            
                end                
            end
                  
            time = toc;
            disp(['�����ʱ',num2str(time), 's']);
            disp('===========================================================');
            cnt_processed = cnt_processed + 1; %�������ݼ�����+1
            
            
        end%��֡����ѭ������        
    end%��Ƭ���ݴ���ѭ������
    %============================================================================
    
    %============================�������ݻ���==========================================
    if SAVEOUTPUT_ON == 1
        %���������Ϊ               
        save(['.\output\', SAVE_NAME, '.mat'], 'radar_timestamps', 'pc_timestamps', 'xyz_all', 'startTime'...
            , 'cnt_processed', 'cnt_frameGlobal', 'set_capture_frames', 'radar_fps'...
            , 'rangeList', 'velocityList');
    end
    %============================================================================
    testID = testID + 1;
end
fclose(fidList);