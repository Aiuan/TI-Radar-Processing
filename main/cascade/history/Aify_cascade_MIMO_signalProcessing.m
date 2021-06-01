clear all;
close all;
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
testList = strcat(input_path,'testList.txt');
%��testList.txt�ļ�
fidList = fopen(testList,'r');
%��Ϊ���ɲ����ļ��ı��
testID = 1;

while ~feof(fidList)%���test.List�ļ���Ϊ��
    
    %% get each test vectors within the test list
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
            tic%��ʼ��ʱ�����㵥֡����ķ�ʱ��
            %read and calibrate raw ADC data            
            calibrationObj.frameIdx = frameIdx;%��ȡ��У������ǰ֡��ԭʼadc����
            frameCountGlobal = frameCountGlobal+1;%ȫ�ִ���֡������+1
            %��ȡadc����
            adcData = datapath(calibrationObj);%4-D complex double��������
            
            % RX Channel re-ordering
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:); %����RX��λ�ã������������߱�� 
            
            % adcData
            % ά�ȣ�   ÿ��Chirp�еĲ��������������Ⱥ�˳��
            %             Loop����Ŀ�������Ⱥ�˳��
            %             RX������RX�ռ�����λ��˳��***
            %             TX������TX�����Ⱥ�˳��
            
            
            %perform 2D FFT
            rangeFFTOut = [];
            DopplerFFTOut = [];
            
            for i_tx = 1: size(adcData,4)%�Ե�i_tx���������߽��д���
                % range FFT
                rangeFFTOut(:,:,:,i_tx)     = datapath(rangeFFTObj, adcData(:,:,:,i_tx));%����rangeFFT����datapathλ��module��rangeProc
                
                % Doppler FFT
                DopplerFFTOut(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut(:,:,:,i_tx));%����DopplerFFT����datapathλ��module��DopplerProc
                
            end
            
            
            % CFAR done along only TX and RX used in MIMO array
            DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));
            
            %detection
            sig_integrate = 10*log10(sum((abs(DopplerFFTOut)).^2,3) + 1);%��DopplerFFTOut��dim=3�����ȡ�������sig_integrate
                                    
            detection_results = datapath(detectionObj, DopplerFFTOut);%����detection����datapathλ��module��detection��
              
            detect_all_points = [];%��ʼ����⵽�ĵ����Ϣ����
            for iobj = 1:length(detection_results)
                detect_all_points (iobj,1)=detection_results(iobj).rangeInd+1;%ȡrange��Ϣ
                detect_all_points (iobj,2)=detection_results(iobj).dopplerInd_org+1;%ȡDoppler��Ϣ
                detect_all_points (iobj,4)=detection_results(iobj).estSNR;%ȡ����ǿ����Ϣ
            end
            
            if PLOT_ON
                figure(1);
                set(gcf,'units','normalized','outerposition',[0 0 1 1])                
                subplot(2,2,1)%��ͼ ������range �����귴��ǿ�� ���壺չʾDopplerFFTOut���м�һ�У��ٶ�Ϊ0��̬Ŀ���rangeͼ             
                plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,size(sig_integrate,2)/2+1),'g','LineWidth',4);hold on; grid on
                for ii=1:size(sig_integrate,2)%DopplerFFTOut�ĵ�ii��   
                    plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,ii));hold on; grid on
                    if ~isempty(detection_results)%���ͨ��CFAR�㷨��⵽��Ŀ��
                        ind = find(detect_all_points(:,2)==ii);%�ҵ���ǰDoppler����ٶȶ�Ӧ�ļ���
                        if (~isempty(ind))%����м���
                            rangeInd = detect_all_points(ind,1);%ȡ��⵽�ĵ��range
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
            
            angles_all_points = [];%��ʼ�����㷽λ�Ǿ���
            xyz = [];%��ʼ������ռ��������
            
            if ~isempty(detection_results)%�����⵽��
                % DOA, the results include detection results + angle estimation results.
                % access data with angleEst{frame}(objectIdx).fieldName
                angleEst = datapath(DOAObj, detection_results);%����angle estimation
                
                if length(angleEst) > 0%�����⵽�ǶȽ��
                    for iobj = 1:length(angleEst)%�Ե�iobj�������
                        angles_all_points (iobj,1:2)=angleEst(iobj).angles(1:2);%angleEst.angles��4��ֵ��ȡǰ��������һ��Ϊ��λ�ǣ��ڶ���Ϊ�߳̽�
                        angles_all_points (iobj,3)=angleEst(iobj).estSNR;%Ԥ�������
                        angles_all_points (iobj,4)=angleEst(iobj).rangeInd;%ȡrange��Index
                        angles_all_points (iobj,5)=angleEst(iobj).doppler_corr;%ȡdoppler_corr
                        angles_all_points (iobj,6)=angleEst(iobj).range;%ȡrange
                        % switch left and right, the azimuth angle is flipped
                        xyz(iobj,1) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,1))*cosd(angles_all_points (iobj,2));%x
                        xyz(iobj,2) = angles_all_points (iobj,6)*cosd(angles_all_points (iobj,1))*cosd(angles_all_points (iobj,2));%y
                        % switch upside and down, the elevation angle is flipped
                        xyz(iobj,3) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,2));%z
                        xyz(iobj,4) = angleEst(iobj).doppler_corr;%ȡdoppler_corr
                        xyz(iobj,9) = angleEst(iobj).dopplerInd_org;%ȡdopplerInd_org
                        xyz(iobj,5) = angleEst(iobj).range;%ȡrange
                        xyz(iobj,6) = angleEst(iobj).estSNR;%ȡ����ǿ��
                        xyz(iobj,7) = angleEst(iobj).doppler_corr_overlap;%ȡdoppler_corr_overlap
                        xyz(iobj,8) = angleEst(iobj).doppler_corr_FFT;%ȡdoppler_corr_FFT
                        
                    end
                                        
                    maxRangeShow = detectionObj.rangeBinSize*rangeFFTObj.rangeFFTSize;%�ɲ��������Ƶ������������
                    if PLOT_ON
                        moveID = find(abs(xyz(:,4))>=0);%���abs(doppler_corr)���ڵ���0��Ŀ�꣬�²�doppler_corrΪ�ٶ�
                        subplot(2,2,4);%�������ɵ���ͼ                        

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
                        subplot(2,2,3)%���ƾ�̬Ŀ�꣬range��azimuth����ͼ
                        STATIC_ONLY = 1;%ֻ��ʾ��̬Ŀ��
                        minRangeBinKeep =  5;
                        rightRangeBinDiscard =  20;
                        [mag_data_static(:,:,frameCountGlobal) mag_data_dynamic(:,:,frameCountGlobal) y_axis x_axis]= plot_range_azimuth_2D(detectionObj.rangeBinSize, DopplerFFTOut,...
                            length(calibrationObj.IdTxForMIMOProcess),length(calibrationObj.RxForMIMOProcess), ...
                            detectionObj.antenna_azimuthonly, LOG_ON, STATIC_ONLY, PLOT_ON, minRangeBinKeep, rightRangeBinDiscard);
                        title('range/azimuth heat map static objects')
                        
                        
                        if (DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP)                   
                            figure(2)
                            subplot(121);%���ƾ�̬Ŀ�꣬range��azimuth����ͼ
                            surf(y_axis, x_axis, (mag_data_static(:,:,frameCountGlobal)).^0.1,'EdgeColor','none');
                            view(2);
                            xlabel('meters');    ylabel('meters');
                            axis('equal');
                            title({'Static Range-Azimuth Heatmap',strcat('Current Frame Number = ', num2str(frameCountGlobal))})

                            subplot(122);%���ƶ�̬Ŀ�꣬range��azimuth����ͼ
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
                             
            cnt = cnt + 1; %�������ݺͼ�¼���ݵļ�����+1   
            toc%��ʱ����
            
        end%��֡����ѭ������        
    end%������ݴ���ѭ��������һ��ֻ�С�0000��һ��
    
    %���洦�����ݽ׶�
    if SAVEOUTPUT_ON == 1
        disp('save start...');
    end        
    testID = testID + 1;
    
end
