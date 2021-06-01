function [RFmap, VFmap, VFmap_removeStatic, VFmap_relative, frame_list, velocity_bin] = microDopplerExtract(numFrames_toRun, rangeMin, rangeMax)
    
    %是否根据原始数据文件夹中的config.mmwave.json文件重新生成参数文件
    PARAM_FILE_GEN_ON = 1;
    %数据平台类型
    dataPlatform = 'TDA2';
    
    %程序目录
    pro_path = 'E:\matlab_workspace\202011毫米波雷达测试实验\MatlabExamples\4chip_cascade_MIMO_example';
    %input文件夹目录
    input_path = strcat(pro_path,'\main\cascade\input\');
    %描述待处理数据文件目录
    testList = strcat(input_path,'testList.txt');
    %打开testList.txt文件
    fidList = fopen(testList,'r');
    %作为生成参数文件的编号
    testID = 1;
    
    % processing
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
        %加载校准文件
        load(dataFolder_calib)

        % simTopObj is used for top level parameter parsing and data loading and saving
        simTopObj           = simTopCascade('pfile', pathGenParaFile); %内含如何读取adc数据
        calibrationObj      = calibrationCascade('pfile', pathGenParaFile, 'calibrationfilePath', dataFolder_calib);
        rangeFFTObj         = rangeProcCascade('pfile', pathGenParaFile);%内含如何对数据进行rangeFFT处理
        DopplerFFTObj       = DopplerProcClutterRemove('pfile', pathGenParaFile);%内含如何对数据进行DopplerFFT处理
        detectionObj        = CFAR_CASO('pfile', pathGenParaFile);%内含通过angleFFT 和 CFAR算法的detection结果
        DOAObj              = DOACascade('pfile', pathGenParaFile);

        % get system level variables
        platform            = simTopObj.platform;%使用平台类型
        numValidFrames      = simTopObj.totNumFrames;%设定有效帧数，实际上会减少1帧
        cnt = 1;%用于记录处理了几帧，并存储了结果的计数器
        frameCountGlobal = 0;%全局处理到第几帧计数


        % Get Unique File Idxs in the "dataFolder_test"   
        [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);%得数据编号，一般为0000


        for i_file = 1:length(fileIdx_unique)%不是间隔采集模式，一般为length(fileIdx_unique)=1
            
            % Get File Names for the Master, Slave1, Slave2, Slave3   
            % 得到Master, Slave1, Slave2, Slave3 data的文件名及Idx的文件名
            [fileNameStruct]= getBinFileNames_withIdx(dataFolder_test, fileIdx_unique{i_file});%得到adc数据文件、idx数据文件的文件名        

            %pass the Data File to the calibration Object
            calibrationObj.binfilePath = fileNameStruct;%传递数据文件至calibrationObj的binfilePath属性

            % Get Valid Number of Frames
            % 确定有多少有效帧，有效帧数据大小，在master_0000_idx.bin文件中的第？个字节显示，有效帧为多少，有效帧数据大小
            [numValidFrames, ~] = getValidNumFrames(fullfile(dataFolder_test, fileNameStruct.masterIdxFile));
            %intentionally skip the first frame due to TDA2 


            % ---------------------------------逐帧处理----------------------------------------------
            
            %处理帧序列
            frame_list = 2:1:min(numValidFrames,1+numFrames_toRun);
            %距离-帧数矩阵
            RFmap = zeros(rangeFFTObj.rangeFFTSize, size(frame_list,2));
            %速度—帧数矩阵
            VFmap = zeros(DopplerFFTObj.dopplerFFTSize, size(frame_list,2));
            %静态移除，速度—帧数矩阵
            VFmap_removeStatic = zeros(DopplerFFTObj.dopplerFFTSize, size(frame_list,2));
            %当前帧减去前一帧，速度—帧数矩阵
            VFmap_relative = zeros(DopplerFFTObj.dopplerFFTSize, size(frame_list,2));
            %速度格栅
            velocity_bin = (-DopplerFFTObj.dopplerFFTSize/2:DopplerFFTObj.dopplerFFTSize/2-1)*detectionObj.velocityBinSize;
            %上一帧DopplerFFT的结果
            old_sig_integrate = 0;
            
            % 舍弃第一帧，从第二帧起始
            for frameIdx = 2:1:min(numValidFrames,1+numFrames_toRun)
                tic%开始计时，计算单帧处理耗费时间

                %read and calibrate raw ADC data            
                calibrationObj.frameIdx = frameIdx;%读取当前帧校准矩阵
                frameCountGlobal = frameCountGlobal+1;%全局处理帧计数器+1
                disp(frameCountGlobal);

                % 读取adc数据
                % 4-D complex double类型数据
                % 维数对应于（每个Chirp中的采样点数，chirps的数目，接收天线编号，发射天线编号）
                adcData = datapath(calibrationObj);

                % RX Channel re-ordering
                % 根据chirp的发射顺序，重新排列天线编号
                adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:);            

                %仅用单个芯片的数据
                adcData_single = adcData(:,:,1:4,10:12);

                %rangeFFT DopplerFFT环节
                rangeFFTOut = [];
                DopplerFFTOut = [];
                for i_tx = 1: size(adcData_single,4)%对第i_tx个发射天线进行处理
                    % range FFT
                    rangeFFTOut(:,:,:,i_tx)     = datapath(rangeFFTObj, adcData_single(:,:,:,i_tx));%进行rangeFFT处理，datapath位于module：rangeProc
                    % Doppler FFT
                    DopplerFFTOut(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut(:,:,:,i_tx));%进行DopplerFFT处理，datapath位于module：DopplerProc
                end


                % 将3发4收，DopplerFFT结果合成为12虚拟天线的DopplerFFT结果
                DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));

                %对12虚拟天线的DopplerFFT结果，取绝对值->平方->求和
                sig_integrate = sum((abs(DopplerFFTOut)).^2,3);


                %------------------------------多普勒时频图提取--------------------------------------
                rangeMin_idx = floor(rangeMin/detectionObj.rangeBinSize);
                rangeMax_idx = ceil(rangeMax/detectionObj.rangeBinSize);
                
                %帧数-距离图
                RFmap(:,frameIdx-1) = sum(sig_integrate,2);

                %帧数-速度图
                value = sum(sig_integrate(rangeMin_idx:rangeMax_idx,:));
                VFmap(:,frameIdx-1) = value';
                
                %静态移除
                avg = mean(rangeFFTOut,2);
                rangeFFTOut_removeStatic = rangeFFTOut - avg;
                DopplerFFTOut_removeStatic = [];
                for i_tx = 1: size(adcData_single,4)%对第i_tx个发射天线进行处理
                    % Doppler FFT
                    DopplerFFTOut_removeStatic(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut_removeStatic(:,:,:,i_tx));%进行DopplerFFT处理，datapath位于module：DopplerProc
                end
                % 将3发4收，DopplerFFT结果合成为12虚拟天线的DopplerFFT结果
                DopplerFFTOut_removeStatic = reshape(DopplerFFTOut_removeStatic,size(DopplerFFTOut_removeStatic,1), size(DopplerFFTOut_removeStatic,2), size(DopplerFFTOut_removeStatic,3)*size(DopplerFFTOut_removeStatic,4));
                %对12虚拟天线的DopplerFFT结果，取绝对值->平方->求和
                sig_integrate_removeStatic = sum((abs(DopplerFFTOut_removeStatic)).^2,3);
                value = sum(sig_integrate_removeStatic(rangeMin_idx:rangeMax_idx,:));
                VFmap_removeStatic(:,frameIdx-1) = value';
                
                %上一帧内容移除
                sig_integrate_relative = sig_integrate - old_sig_integrate;
                old_sig_integrate = sig_integrate;
                value = sum(sig_integrate_relative(rangeMin_idx:rangeMax_idx,:));
                VFmap_relative(:,frameIdx-1) = value';
                
                
                
                cnt = cnt + 1; %处理数据和记录数据的计数器+1   
                toc%计时结束
                
            end%循环完所有帧
            
            
            
            
            
        end%逐段数据处理循环结束，一般只有“0000”一段
        
        testID = testID + 1;%数据段号+1
    end
end