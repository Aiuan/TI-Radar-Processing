clearvars
close all;
clc;

%% ����ͼƬ�ļ�����ʱ���
imageFolder = dir('K:\ourDataset\20210514\images\20210514mode1Group1');
images = struct();
for i = 3:size(imageFolder,1)
    idx = i-2;
    images(idx).path = [imageFolder(i).folder,filesep,imageFolder(i).name];
    % 16λUNIXʱ���
    images(idx).timestamp =  imageFolder(i).name(1:strfind(imageFolder(i).name,'.')-1);
    images(idx).timestamp = uint64(str2double(images(idx).timestamp));
end


%% ���в�������
% ������Ҫ����ʱ��

% image_start_timestamp = 1619619555539725;%0428mode3Group1���������Ӳ�
% image_end_timestamp = 1619619653492323;

% image_start_timestamp = 1619619573156350;%0428mode3Group1����������2����
% image_end_timestamp = 1619619576340088;

% image_start_timestamp = 1619619800523929;%0428mode3Group1������ֹ��ⲻ��
% image_end_timestamp = 1619619814356697;
% image_start_timestamp = 1619619806196790;%0428mode3Group1����������⵽
% image_end_timestamp = 1619619814356697;

% image_start_timestamp = 1620962777191675;%0514mode1Group1ͣ���ӵ���
% image_end_timestamp = 1620962782662602;
image_start_timestamp = 1620962782662602;%0514mode1Group1�����ӵ��
image_end_timestamp = 1620962782762864;


image_start_timestamp = radarMatchImage(uint64(image_start_timestamp), images) ;
image_end_timestamp = radarMatchImage(uint64(image_end_timestamp), images) ;

% �ȴ�����
KEY_ON = 1;
% ����֡
NUM_RESERVE_FRAME = 10;

%�Ƿ����ԭʼ�����ļ����е�config.mmwave.json�ļ��������ɲ����ļ�
PARAM_FILE_GEN_ON = 1;
%����ƽ̨����
dataPlatform = 'TDA2';

% ·��10FPS
% ����20FPS
radar_FPS = 20;




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
    
    % get each test vectors within the test list
    % test data file name
    dataFolder_test = fgetl(fidList);%ԭʼ�����ļ���·��   
    
    % ��ѯ�״�ɼ���ʼʱ�������FPS
    startTimefile = dir(fullfile(dataFolder_test, '*.startTime.txt'));
    f_startTime = fopen(fullfile(startTimefile.folder, startTimefile.name), 'r');
    radar_startTime = fscanf(f_startTime, '%f');
    fclose(f_startTime);    
    radar_startTime = uint64(radar_startTime*1000000); %16λUNIXʱ��� 
    
   
    %calibration file name
    dataFolder_calib = fgetl(fidList);%У׼�ļ�·��
    
    %module_param_file defines parameters to init each signal processing
    %module
    module_param_file = fgetl(fidList);%����ԭʼ�����ļ����е�config.mmwave.json�ļ��������ɲ����ļ��Ĵ�������·��
    
     %parameter file name for the test
    pathGenParaFile = [input_path,'test',num2str(testID), '_param.m'];%���ɵĲ����ļ���·��
    
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
    cnt = 0;%���ڼ�¼�����˼�֡�����洢�˽���ļ�����
    frameCountGlobal = 0;%ȫ�ֵڼ�֡����
    radar_timestamp = radar_startTime;%�״���ʼʱ���
    cnt_reserve_frame = 0;%����֡������
    
    % Get Unique File Idxs in the "dataFolder_test"   
    [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);%�����ݱ�ţ�һ��Ϊ0000
   
     figure(1);
%      set(gcf,'units','normalized','outerposition',[0.1 0.1 0.8 0.8]);
     set(gcf,'units','normalized','outerposition',[0 0 1 1]);
    
    
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
        for frameIdx = 1:1:numValidFrames
            
            radar_timestamp = radar_startTime + frameCountGlobal * 1e6/radar_FPS;%���㵱ǰ�״�֡ʱ���
            [image_timestamp, image_path] = radarMatchImage(radar_timestamp, images);%Ѱ�Ҷ�ӦͼƬ֡ʱ���
            if (image_timestamp < image_start_timestamp)  || (image_timestamp > image_end_timestamp) 
                disp('===========================================================');
                fprintf('������ %s Ƭ�е� %d ֡...��ȫ�ֵĵ� %d/%d ֡��\n', fileIdx_unique{i_file},frameIdx,frameCountGlobal+1,simTopObj.totNumFrames);
                frameCountGlobal = frameCountGlobal+1;
                % ������Ѱ�ҵ�ʱ�����������                
                continue;
            end
            
            
            disp('===========================================================');
            fprintf('���ڴ���� %s Ƭ�е� %d ֡...��ȫ�ֵĵ� %d/%d ֡������ĵ�%d֡��\n', fileIdx_unique{i_file},frameIdx,frameCountGlobal+1,simTopObj.totNumFrames, cnt);
            
            
            fprintf('�״�֡ʱ��� >>> %10d.%06d\n',uint64(floor(double(radar_timestamp)/1000000)),mod(radar_timestamp,1000000));
            
           
            fprintf('���֡ʱ��� >>> %10d.%06d\n',uint64(floor(double(image_timestamp)/1000000)),mod(image_timestamp,1000000));
            
            subplot(2,2,1);%��ʾ��ӦͼƬ
            imshow(image_path);
            title(sprintf('%10d.%06d\n',uint64(floor(double(image_timestamp)/1000000)),mod(image_timestamp,1000000)));
            
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
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:); %����RX��λ�ã��������С�оƬ4��оƬ1��оƬ3��оƬ2��           
            
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
            
            subplot(2,2,3);
            plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,size(sig_integrate,2)/2+1),'g','LineWidth',4);
            hold on; 
            grid on;
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
            xlabel('Range(m)');
            ylabel('Receive Power (dB)')
            title(sprintf('%10d.%06d\n',uint64(floor(double(radar_timestamp)/1000000)),mod(radar_timestamp,1000000)));
            hold off;
                                    
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
                    
                    %���Ƶ�ǰ֡����ͼ
                    subplot(2,2,4);                        
                    % �Է���z���ֵ����-xyz(moveID,3)
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
                    ylim([0 maxRangeShow])%��������ϵΪy���״����ǰ��
%                     view([28 28]);  
                    view([0 90]); 
                    title(' 3D point cloud');
                    
                    %ͶӰ������ͼ��
                    pixel_coordinate = projection(xyz, 3,75);
                    subplot(2,2,1);
                    hold on;
                    scatter(pixel_coordinate(1,:),...
                        pixel_coordinate(2,:),...
                        70, pixel_coordinate(3,:), '.');
                    
                    
                    %���Ʋ�������ͼ 
                    if cnt_reserve_frame+1 <= NUM_RESERVE_FRAME
                        % ������֡������������ʱ
                        cnt_reserve_frame = cnt_reserve_frame+1;
                    else
                        % ������֡����������ʱ
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
                    ylim([0 maxRangeShow])%��������ϵΪy���״����ǰ��
                    view([0 90]);              
                    title(sprintf('%d consecutive frames of 3D point cloud: ', NUM_RESERVE_FRAME));
                    hold off;
                    
                    pause(0.1); 
                                        
                end                
            end
                             
            cnt = cnt + 1; %�������ݺͼ�¼���ݵļ�����+1   
            time = toc;
            disp(['�����ʱ',num2str(time)]);
            disp('===========================================================');
            
            
            %�ȴ�����
            if KEY_ON == 0
                continue;
            end             
            key = waitforbuttonpress;
            while(key==0)
                key = waitforbuttonpress;
            end            
            disp('next frame.');
                        
            
        end%��֡����ѭ������        
    end%������ݴ���ѭ��������һ��ֻ�С�0000��һ��
        
    testID = testID + 1;
    
end
