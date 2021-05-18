function [RFmap, VFmap, VFmap_removeStatic, VFmap_relative, frame_list, velocity_bin] = microDopplerExtract(numFrames_toRun, rangeMin, rangeMax)
    
    %�Ƿ����ԭʼ�����ļ����е�config.mmwave.json�ļ��������ɲ����ļ�
    PARAM_FILE_GEN_ON = 1;
    %����ƽ̨����
    dataPlatform = 'TDA2';
    
    %����Ŀ¼
    pro_path = 'E:\matlab_workspace\202011���ײ��״����ʵ��\MatlabExamples\4chip_cascade_MIMO_example';
    %input�ļ���Ŀ¼
    input_path = strcat(pro_path,'\main\cascade\input\');
    %���������������ļ�Ŀ¼
    testList = strcat(input_path,'testList.txt');
    %��testList.txt�ļ�
    fidList = fopen(testList,'r');
    %��Ϊ���ɲ����ļ��ı��
    testID = 1;
    
    % processing
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
        %����У׼�ļ�
        load(dataFolder_calib)

        % simTopObj is used for top level parameter parsing and data loading and saving
        simTopObj           = simTopCascade('pfile', pathGenParaFile); %�ں���ζ�ȡadc����
        calibrationObj      = calibrationCascade('pfile', pathGenParaFile, 'calibrationfilePath', dataFolder_calib);
        rangeFFTObj         = rangeProcCascade('pfile', pathGenParaFile);%�ں���ζ����ݽ���rangeFFT����
        DopplerFFTObj       = DopplerProcClutterRemove('pfile', pathGenParaFile);%�ں���ζ����ݽ���DopplerFFT����
        detectionObj        = CFAR_CASO('pfile', pathGenParaFile);%�ں�ͨ��angleFFT �� CFAR�㷨��detection���
        DOAObj              = DOACascade('pfile', pathGenParaFile);

        % get system level variables
        platform            = simTopObj.platform;%ʹ��ƽ̨����
        numValidFrames      = simTopObj.totNumFrames;%�趨��Ч֡����ʵ���ϻ����1֡
        cnt = 1;%���ڼ�¼�����˼�֡�����洢�˽���ļ�����
        frameCountGlobal = 0;%ȫ�ִ����ڼ�֡����


        % Get Unique File Idxs in the "dataFolder_test"   
        [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);%�����ݱ�ţ�һ��Ϊ0000


        for i_file = 1:length(fileIdx_unique)%���Ǽ���ɼ�ģʽ��һ��Ϊlength(fileIdx_unique)=1
            
            % Get File Names for the Master, Slave1, Slave2, Slave3   
            % �õ�Master, Slave1, Slave2, Slave3 data���ļ�����Idx���ļ���
            [fileNameStruct]= getBinFileNames_withIdx(dataFolder_test, fileIdx_unique{i_file});%�õ�adc�����ļ���idx�����ļ����ļ���        

            %pass the Data File to the calibration Object
            calibrationObj.binfilePath = fileNameStruct;%���������ļ���calibrationObj��binfilePath����

            % Get Valid Number of Frames
            % ȷ���ж�����Ч֡����Ч֡���ݴ�С����master_0000_idx.bin�ļ��еĵڣ����ֽ���ʾ����Ч֡Ϊ���٣���Ч֡���ݴ�С
            [numValidFrames, ~] = getValidNumFrames(fullfile(dataFolder_test, fileNameStruct.masterIdxFile));
            %intentionally skip the first frame due to TDA2 


            % ---------------------------------��֡����----------------------------------------------
            
            %����֡����
            frame_list = 2:1:min(numValidFrames,1+numFrames_toRun);
            %����-֡������
            RFmap = zeros(rangeFFTObj.rangeFFTSize, size(frame_list,2));
            %�ٶȡ�֡������
            VFmap = zeros(DopplerFFTObj.dopplerFFTSize, size(frame_list,2));
            %��̬�Ƴ����ٶȡ�֡������
            VFmap_removeStatic = zeros(DopplerFFTObj.dopplerFFTSize, size(frame_list,2));
            %��ǰ֡��ȥǰһ֡���ٶȡ�֡������
            VFmap_relative = zeros(DopplerFFTObj.dopplerFFTSize, size(frame_list,2));
            %�ٶȸ�դ
            velocity_bin = (-DopplerFFTObj.dopplerFFTSize/2:DopplerFFTObj.dopplerFFTSize/2-1)*detectionObj.velocityBinSize;
            %��һ֡DopplerFFT�Ľ��
            old_sig_integrate = 0;
            
            % ������һ֡���ӵڶ�֡��ʼ
            for frameIdx = 2:1:min(numValidFrames,1+numFrames_toRun)
                tic%��ʼ��ʱ�����㵥֡����ķ�ʱ��

                %read and calibrate raw ADC data            
                calibrationObj.frameIdx = frameIdx;%��ȡ��ǰ֡У׼����
                frameCountGlobal = frameCountGlobal+1;%ȫ�ִ���֡������+1
                disp(frameCountGlobal);

                % ��ȡadc����
                % 4-D complex double��������
                % ά����Ӧ�ڣ�ÿ��Chirp�еĲ���������chirps����Ŀ���������߱�ţ��������߱�ţ�
                adcData = datapath(calibrationObj);

                % RX Channel re-ordering
                % ����chirp�ķ���˳�������������߱��
                adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:);            

                %���õ���оƬ������
                adcData_single = adcData(:,:,1:4,10:12);

                %rangeFFT DopplerFFT����
                rangeFFTOut = [];
                DopplerFFTOut = [];
                for i_tx = 1: size(adcData_single,4)%�Ե�i_tx���������߽��д���
                    % range FFT
                    rangeFFTOut(:,:,:,i_tx)     = datapath(rangeFFTObj, adcData_single(:,:,:,i_tx));%����rangeFFT����datapathλ��module��rangeProc
                    % Doppler FFT
                    DopplerFFTOut(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut(:,:,:,i_tx));%����DopplerFFT����datapathλ��module��DopplerProc
                end


                % ��3��4�գ�DopplerFFT����ϳ�Ϊ12�������ߵ�DopplerFFT���
                DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));

                %��12�������ߵ�DopplerFFT�����ȡ����ֵ->ƽ��->���
                sig_integrate = sum((abs(DopplerFFTOut)).^2,3);


                %------------------------------������ʱƵͼ��ȡ--------------------------------------
                rangeMin_idx = floor(rangeMin/detectionObj.rangeBinSize);
                rangeMax_idx = ceil(rangeMax/detectionObj.rangeBinSize);
                
                %֡��-����ͼ
                RFmap(:,frameIdx-1) = sum(sig_integrate,2);

                %֡��-�ٶ�ͼ
                value = sum(sig_integrate(rangeMin_idx:rangeMax_idx,:));
                VFmap(:,frameIdx-1) = value';
                
                %��̬�Ƴ�
                avg = mean(rangeFFTOut,2);
                rangeFFTOut_removeStatic = rangeFFTOut - avg;
                DopplerFFTOut_removeStatic = [];
                for i_tx = 1: size(adcData_single,4)%�Ե�i_tx���������߽��д���
                    % Doppler FFT
                    DopplerFFTOut_removeStatic(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut_removeStatic(:,:,:,i_tx));%����DopplerFFT����datapathλ��module��DopplerProc
                end
                % ��3��4�գ�DopplerFFT����ϳ�Ϊ12�������ߵ�DopplerFFT���
                DopplerFFTOut_removeStatic = reshape(DopplerFFTOut_removeStatic,size(DopplerFFTOut_removeStatic,1), size(DopplerFFTOut_removeStatic,2), size(DopplerFFTOut_removeStatic,3)*size(DopplerFFTOut_removeStatic,4));
                %��12�������ߵ�DopplerFFT�����ȡ����ֵ->ƽ��->���
                sig_integrate_removeStatic = sum((abs(DopplerFFTOut_removeStatic)).^2,3);
                value = sum(sig_integrate_removeStatic(rangeMin_idx:rangeMax_idx,:));
                VFmap_removeStatic(:,frameIdx-1) = value';
                
                %��һ֡�����Ƴ�
                sig_integrate_relative = sig_integrate - old_sig_integrate;
                old_sig_integrate = sig_integrate;
                value = sum(sig_integrate_relative(rangeMin_idx:rangeMax_idx,:));
                VFmap_relative(:,frameIdx-1) = value';
                
                
                
                cnt = cnt + 1; %�������ݺͼ�¼���ݵļ�����+1   
                toc%��ʱ����
                
            end%ѭ��������֡
            
            
            
            
            
        end%������ݴ���ѭ��������һ��ֻ�С�0000��һ��
        
        testID = testID + 1;%���ݶκ�+1
    end
end