%  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
%
%
%   Redistribution and use in source and binary forms, with or without
%   modification, are permitted provided that the following conditions
%   are met:
%
%     Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%
%     Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the
%     documentation and/or other materials provided with the
%     distribution.
%
%     Neither the name of Texas Instruments Incorporated nor the names of
%     its contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.
%
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
%   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
%   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
%   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
%   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
%   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
%   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
%

% cascade_MIMO_signalProcessing.m
%
% Top level main test chain to process the raw ADC data. The processing
% chain including adc data calibration module, range FFT module, DopplerFFT
% module, CFAR module, DOA module. Each module is first initialized before
% actually used in the chain.

clearvars
close all

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
%是否存储中间过程图片
SAVE_FIG_ON = 0;

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
    
    %% get each test vectors within the test list
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
            tic%开始计时，计算单帧处理耗费时间
            %read and calibrate raw ADC data            
            calibrationObj.frameIdx = frameIdx;%读取并校正，当前帧，原始adc数据
            frameCountGlobal = frameCountGlobal+1;%全局处理帧计数器+1
            %读取adc数据
            adcData = datapath(calibrationObj);%4-D complex double类型数据
            
            % RX Channel re-ordering
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:); %按照RX的位置，重新排列天线编号 
            
            % adcData
            % 维度：   每个Chirp中的采样点数（采样先后顺序）
            %             Loop的数目（发射先后顺序）
            %             RX数量（RX空间排列位置顺序）***
            %             TX数量（TX发射先后顺序）
            
            
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
            
            if PLOT_ON
                figure(1);
                set(gcf,'units','normalized','outerposition',[0 0 1 1])                
                subplot(2,2,1)%绘图 横坐标range 纵坐标反射强度 意义：展示DopplerFFTOut的中间一列，速度为0静态目标的range图             
                plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,size(sig_integrate,2)/2+1),'g','LineWidth',4);hold on; grid on
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
                
                %title(['FrameID: ' num2str(cnt)]);
                xlabel('Range(m)');
                ylabel('Receive Power (dB)')
                title(['Range Profile(zero Doppler - thick green line): frameID ' num2str(frameIdx)]);
                hold off;
                subplot(2,2,2);
                %subplot_tight(2,2,2,0.1)
                %imagesc((sig_integrate))
                imagesc((-size(sig_integrate,2)/2:size(sig_integrate,2)/2-1)*detectionObj.velocityBinSize,(1:size(sig_integrate,1))*detectionObj.rangeBinSize,(sig_integrate));
                c = colorbar;
                c.Label.String = 'Relative Power(dB)';
                title(' Range/Velocity Plot');
                pause(0.01)
            end
            
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
                                        
                    maxRangeShow = detectionObj.rangeBinSize*rangeFFTObj.rangeFFTSize;%由采样率限制的最大检测距离计算
                    if PLOT_ON
                        moveID = find(abs(xyz(:,4))>=0);%检查abs(doppler_corr)大于等于0的目标，猜测doppler_corr为速度
                        subplot(2,2,4);%绘制生成点云图                        

                        scatter3(xyz(moveID,1),xyz(moveID,2),xyz(moveID,3),10,(xyz(moveID,4)),'filled');%x,y,z,doppler_corr
                        
                        c = colorbar;
                        c.Label.String = 'velocity (m/s)';                        
                        grid on;
                        
                        xlabel('X (m)');
                        ylabel('Y (m)');
                        zlabel('Z (m)');
                        axis('image');
                        colormap('jet');
                        
                        view([-9 15])                        
                        title(' 3D point cloud');
                        
                        %plot range and azimuth heatmap
                        subplot(2,2,3)%绘制静态目标，range和azimuth热力图
                        STATIC_ONLY = 1;%只显示静态目标
                        minRangeBinKeep =  5;
                        rightRangeBinDiscard =  20;
                        [mag_data_static(:,:,frameCountGlobal) mag_data_dynamic(:,:,frameCountGlobal) y_axis x_axis]= plot_range_azimuth_2D(detectionObj.rangeBinSize, DopplerFFTOut,...
                            length(calibrationObj.IdTxForMIMOProcess),length(calibrationObj.RxForMIMOProcess), ...
                            detectionObj.antenna_azimuthonly, LOG_ON, STATIC_ONLY, PLOT_ON, minRangeBinKeep, rightRangeBinDiscard);
                        title('range/azimuth heat map static objects')
                        
                        
                        if (DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP)                   
                            figure(2)
                            subplot(121);%绘制静态目标，range和azimuth热力图
                            surf(y_axis, x_axis, (mag_data_static(:,:,frameCountGlobal)).^0.1,'EdgeColor','none');
                            view(2);
                            xlabel('meters');    ylabel('meters');
                            axis('equal');
                            title({'Static Range-Azimuth Heatmap',strcat('Current Frame Number = ', num2str(frameCountGlobal))})

                            subplot(122);%绘制动态目标，range和azimuth热力图
                            surf(y_axis, x_axis, (mag_data_dynamic(:,:,frameCountGlobal)).^0.4,'EdgeColor','none');
                            view(2);    
                            xlabel('meters');    ylabel('meters');
                            axis('equal');
                            title('Dynamic HeatMap')
                        end
    
                        pause(0.1) 
                    end                    
                end                
            end
                             
            cnt = cnt + 1; %处理数据和记录数据的计数器+1   
            toc%计时结束
            
        end%逐帧处理循环结束        
    end%逐段数据处理循环结束，一般只有“0000”一段
    
    %保存处理数据阶段
    if SAVEOUTPUT_ON == 1
        disp('save start...');
    end        
    testID = testID + 1;
    
end
