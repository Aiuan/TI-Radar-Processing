clearvars
close all;
clc;

%% ��������

% �������Ŀ¼
imagePath = 'K:\ourDataset\20210428\images\20210428mode3Group1';

% ========================������Ҫ�۲��ʱ��====================
% imageTimestamp_start = 0;
% imageTimestamp_end = inf;

% imageTimestamp_start = 1619619555539725;%0428mode3Group1���������Ӳ�
% imageTimestamp_end = 1619619653492323;

% imageTimestamp_start = 1619619573156350;%0428mode3Group1����������2����
% imageTimestamp_end = 1619619576340088;

% imageTimestamp_start = 1619619797524763;%0428mode3Group1�����ٶ�ģ��
imageTimestamp_start = 1619619800917161;%0428mode3Group1�����ٶ�ģ��������ģ��
% imageTimestamp_start = 1619619800523929;%0428mode3Group1������ֹ��ⲻ���������ٶ�ģ������ȫģ��
imageTimestamp_end = 1619619814356697;
% imageTimestamp_start = 1619619806196790;%0428mode3Group1����������⵽
% imageTimestamp_end = 1619619814356697;

% imageTimestamp_start = 1620962777191675;%0514mode1Group1ͣ���ӵ���
% imageTimestamp_end = 1620962782662602;
% imageTimestamp_start = 1620962782662602;%0514mode1Group1�����ӵ��
% imageTimestamp_end = 1620962782762864;
% ===========================================================

% �궨����
%��ɽ�궨����
radar_camera_matchMatrix = [978.427720340050, 589.993462843703, 13.1229972166119, -0.840684195154766;...
                                                14.7696499738007, 388.888646655374, -959.947091338099, -137.871686789255;
                                                0.0239198127750308, 0.999635770515324, 0.0124967541002258, 0.0523119353667915];
% % 20210513-20210514��Ȫ�궨����
% radar_camera_matchMatrix = [296.017605935561, 835.703097555440, 33.2444346178279, -386.776196365743;...
%     -196.067005960755, 455.292940171630, -575.779148193414, 184.623265383036;...
%     -0.395085889573456, 0.917177372733613, 0.0518922615176608, 0.788409932235097];


%����֡��
numFrames_toRun = inf; 
%�Ƿ����ԭʼ�����ļ����е�config.mmwave.json�ļ��������ɲ����ļ�
PARAM_FILE_GEN_ON = 1;
% �ȴ�����
KEY_ON = 1;
% ����֡
NUM_RESERVE_FRAME = 10;

%����ƽ̨����
dataPlatform = 'TDA2';
%% �����������
imageFolder = dir(imagePath);
images = struct();
for i = 3:size(imageFolder,1)
    idx = i-2;
    images(idx).path = [imageFolder(i).folder,filesep,imageFolder(i).name];
    % 16λUNIXʱ���
    images(idx).timestamp =  imageFolder(i).name(1:strfind(imageFolder(i).name,'.')-1);
    images(idx).timestamp = uint64(str2double(images(idx).timestamp));
end

% ����imageTimestamp_start��imageTimestamp_end
imageTimestamp_start = radarMatchImage(uint64(imageTimestamp_start), images) ;
imageTimestamp_end = radarMatchImage(uint64(imageTimestamp_end), images) ;

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
    detectionObj        = CFAR_CASO('pfile', pathGenParaFile);%�ں�ͨ��angleFFT �� CFAR�㷨��detection���
    DOAObj              = DOACascade('pfile', pathGenParaFile);
    
    platform = simTopObj.platform;%ʹ��ƽ̨����
    numValidFrames = simTopObj.totNumFrames;%�趨��Ч֡��
    
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
    
    %�ۺ�֡������
    cnt_reserve_frame = 0;
    
    figure(1);
    set(gcf,'units','normalized','outerposition',[0.1 0.2 0.8 0.8]);
%     set(gcf,'units','normalized','outerposition',[0 0 1 1]);
    
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
        
        %intentionally skip the first frame due to TDA2
        for frameIdx = 1:1:min(numValidFrames,1+numFrames_toRun)
            
            %ȫ��֡������+1
            cnt_frameGlobal = cnt_frameGlobal+1;
            disp('===========================================================');
            fprintf('���ڷ��ʵ� %s Ƭ�е� %d/%d ֡��ȫ�ֵĵ� %d/%d ֡��\n',  fileIdx_unique{i_file}, frameIdx, numValidFrames,...
                cnt_frameGlobal, simTopObj.totNumFrames);
            
            %============================��ѯʱ�������====================================
            % timestamp
            radar_timestamps{cnt_frameGlobal} = getTimestamp(fullfile(dataFolder_test, fileNameStruct.masterIdxFile), frameIdx);
            if cnt_frameGlobal ==1 || isempty(radar_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(radar_timestamps{cnt_frameGlobal} - radar_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('�״��ʱ��� %d������һ֡��� %.3f ms\n', radar_timestamps{cnt_frameGlobal}, diff_timestamp);
            % pc_timestamp
            pc_timestamps{cnt_frameGlobal} = startTime + (cnt_frameGlobal-1)*frameInterval*1000000;%16λUNIXʱ��
            if cnt_frameGlobal ==1 || isempty(pc_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(pc_timestamps{cnt_frameGlobal} - pc_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('���Զ�ʱ��� %.6f������һ֡��� %.3f ms\n', double(pc_timestamps{cnt_frameGlobal})/1e6, diff_timestamp);
            
            % Ѱ�Ҷ�ӦͼƬ֡ʱ���
            radar_timestamp = pc_timestamps{cnt_frameGlobal};
            [image_timestamp, image_path] = radarMatchImage(radar_timestamp, images);
            camera_timestamps{cnt_frameGlobal} = image_timestamp;
            if cnt_frameGlobal ==1 || isempty(camera_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(camera_timestamps{cnt_frameGlobal} - camera_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('�����ʱ��� %.6f������һ֡��� %.3f ms\n', double(camera_timestamps{cnt_frameGlobal})/1e6, diff_timestamp);
            
            if (image_timestamp < imageTimestamp_start) || (image_timestamp > imageTimestamp_end)
                % ������Ѱ�ҵ�ʱ�����������      
                fprintf('������ %s Ƭ�е� %d/%d ֡��ȫ�ֵĵ� %d/%d ֡��\n',  fileIdx_unique{i_file}, frameIdx, numValidFrames,...
                cnt_frameGlobal, simTopObj.totNumFrames);
                disp('===========================================================');
                continue;                
            end
            %============================================================================            
            
            
            %============================��ȡ����==========================================
            
            tic;%��ʼ��ʱ�����㵥֡��ȡ�ķ�ʱ��
            
            % read and calibrate raw ADC data            
            calibrationObj.frameIdx = frameIdx;
            % ά�ȣ�ÿ��Chirp�еĲ���������loop����Ŀ��RX������һ��loop��chirps��������ͨ����TX������ȣ���
            adcData = datapath(calibrationObj);
            % RX Channel re-ordering
            % �������߰��λ�ù�ϵ�������������߱�� ��оƬ4��оƬ1��оƬ3��оƬ2
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:);
            
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
            
            % ת��Ϊ����������ʽ��ά�ȣ�ÿ��Chirp�еĲ���������ÿ�����߷����chirp������������������ ��
            DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));
            %��DopplerFFTOut��dim=3�����ȡ�������sig_integrate
            sig_integrate = 10*log10(sum((abs(DopplerFFTOut)).^2,3) + 1);
            
            
            % CFAR done along only TX and RX used in MIMO array
            % ����detection��CFAR������datapathλ��module��detection
            detection_results = datapath(detectionObj, DopplerFFTOut);
            
            detect_all_points = [];%��ʼ����⵽�ĵ����Ϣ����
            for iobj = 1:length(detection_results)
                detect_all_points (iobj,1)=detection_results(iobj).rangeInd+1;%range index
                detect_all_points (iobj,2)=detection_results(iobj).dopplerInd_org+1;%doppler index
                detect_all_points (iobj,4)=detection_results(iobj).estSNR;%����ǿ��
            end
            
            
            % ����Doppler Map
            subplot(2,3,4);
            %���壺չʾDopplerFFTOut���м�һ�У��ٶ�Ϊ0��̬Ŀ���rangeͼ
            %������range �����귴��ǿ��
            rangeList = (1: size(sig_integrate,1)) * detectionObj.rangeBinSize;
            powerList_v0 = sig_integrate(:, size(sig_integrate,2)/2+1);
            plot(rangeList, powerList_v0, 'g', 'LineWidth', 4);
            hold on;
            grid on;
            for ii=1:size(sig_integrate,2)%DopplerFFTOut�ĵ�ii�У��ٶȲ�Ϊ0����
                powerList_vii = sig_integrate(:,ii);
                plot(rangeList, powerList_vii);
                hold on;
                grid on;
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
            title(sprintf('�״�ʱ���: %10d.%06d\n',uint64(floor(double(radar_timestamp)/1000000)),mod(radar_timestamp,1000000)));
            hold off;
            subplot(2,3,3);
            % ���壺Doppler Map
            % �����ꡪ���ٶȣ������ꡪ������
            velocityList = (-size(sig_integrate,2)/2: size(sig_integrate,2)/2-1) * detectionObj.velocityBinSize;
            rangeList = (1: size(sig_integrate,1)) * detectionObj.rangeBinSize;                
            imagesc(velocityList, rangeList, sig_integrate);
            c = colorbar;
            c.Label.String = 'Relative Power(dB)';
            title(' Range/Velocity Plot');
            

                      
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
                    
                    moveID = find(abs(xyz(:,4))>=0);%���abs(doppler_corr)���ڵ���0��Ŀ��
                    
                    
                    % ���Ƶ�֡����ͼ
                    subplot(2,3,5);
                    %��������ϵΪy���״����ǰ��
                    scatter3(xyz(moveID,1), xyz(moveID,2), xyz(moveID,3), 10, (xyz(moveID,4)),'filled');%x,y,z,doppler_corr
                    c = colorbar;
                    c.Label.String = 'velocity (m/s)'; 
                    set(gca, 'CLim', [velocityList(1), velocityList(end)]);
                    grid on;
                    xlabel('X (m)');
                    ylabel('Y (m)');
                    zlabel('Z (m)');
                    axis('image');
                    maxRangeShow = detectionObj.rangeBinSize*rangeFFTObj.rangeFFTSize;
                    xlim([-maxRangeShow maxRangeShow]);
                    ylim([0 maxRangeShow]);
                    colormap('jet');
                    view([0 90]);                        
                    title(sprintf('�״�ʱ���: %10d.%06d\n',uint64(floor(double(radar_timestamp)/1000000)),mod(radar_timestamp,1000000)));
                    subplot(2,3,5,'color', [0.8,0.8,0.8]);
                    hold on;
                    xyz_zero = xyz(xyz(moveID,4)==0, :);
                    scatter3(xyz_zero(:,1), xyz_zero(:,2), xyz_zero(:,3), 10, (xyz_zero(:,4)),'w', 'filled');
                    hold off;
                    
                    % ��ʾ��ӦͼƬ + ͶӰ������ͼ��
                    remove_distance_min = 3;
                    remove_distance_max = 75;
                    pixel_coordinate = projection(xyz, radar_camera_matchMatrix, remove_distance_min, remove_distance_max);
                    subplot(2,3,1);
                    imshow(image_path);
                    title(sprintf('���ʱ���: %10d.%06d\n',uint64(floor(double(image_timestamp)/1000000)),mod(image_timestamp,1000000)));
                    hold on;
                    scatter(pixel_coordinate(1,:), pixel_coordinate(2,:), 10, pixel_coordinate(3,:), 'filled');
                    hold off;
                    set(gca, 'CLim', [velocityList(1), velocityList(end)]);
                    hold on;
                    pixel_coordinate_zero = pixel_coordinate(:,pixel_coordinate(3,:)==0);
                    scatter(pixel_coordinate_zero(1,:), pixel_coordinate_zero(2,:), 10, pixel_coordinate_zero(3,:), 'w', 'filled');
                    hold off;
                    
                    
                    %���ƶ�֡�ۺϵ���ͼ
                    if cnt_reserve_frame+1 <= NUM_RESERVE_FRAME
                        % ������֡������������ʱ
                        cnt_reserve_frame = cnt_reserve_frame+1;
                    else
                        % ������֡����������ʱ
                        cnt_reserve_frame = 1;
                    end
                    constant_pointClouds{cnt_reserve_frame} = xyz;
                    subplot(2,3,2);
                    for reserve_frameId = 1:size(constant_pointClouds,2)
                        xyz = constant_pointClouds{reserve_frameId};
                        scatter3(xyz(:,1),xyz(:,2),-xyz(:,3),10,(xyz(:,4)),'filled');
                        hold on;
                    end
                    c = colorbar;
                    caxis([-detectionObj.dopplerFFTSize/2*detectionObj.velocityBinSize, (detectionObj.dopplerFFTSize/2-1)*detectionObj.velocityBinSize]);
                    c.Label.String = 'velocity (m/s)';
                    % �趨�ٶȷ�Χ
                    set(gca, 'CLim', [velocityList(1), velocityList(end)]);
                    grid on;
                    xlabel('X (m)');
                    ylabel('Y (m)');
                    zlabel('Z (m)');
                    axis('image'); 
                    xlim([-maxRangeShow maxRangeShow]);
                    ylim([0 maxRangeShow])
                    view([0 90]);              
                    title(sprintf('%d ֡�ۺϵ���ͼ: ', NUM_RESERVE_FRAME));
                    hold off;
                    subplot(2,3,2,'color', [0.8,0.8,0.8]);
                    hold on;
                    xyz_zero = xyz(xyz(:,4)==0, :);
                    scatter3(xyz_zero(:,1), xyz_zero(:,2), xyz_zero(:,3), 10, (xyz_zero(:,4)),'w', 'filled');
                    hold off;
                    
                    
                    %plot range and azimuth heatmap
                    subplot(2,3,6)%���ƾ�̬Ŀ�꣬range��azimuth����ͼ
                    mode = 'static';%mode: 'static'/'dynamic'/'static+dynamic'
                    minRangeBinKeep =  5;
                    rightRangeBinDiscard =  20;
                    LOG_ON = 1;
                    PLOT_ON =1;
                    [~, ~, ~, ~]= plot_range_azimuth_2D(detectionObj.rangeBinSize, DopplerFFTOut,...
                        length(calibrationObj.IdTxForMIMOProcess),length(calibrationObj.RxForMIMOProcess), ...
                        detectionObj.antenna_azimuthonly, LOG_ON, mode, PLOT_ON, minRangeBinKeep, rightRangeBinDiscard);
                    title('range/azimuth heat map static objects');
                    axis('image');
                    
                    pause(0.1); 
                end                
            end
                  
            time = toc;
            disp(['�����ʱ',num2str(time), 's']);
            disp('===========================================================');
            cnt_processed = cnt_processed + 1; %�������ݼ�����+1
            
            %�ȴ�����
            if KEY_ON == 0
                continue;
            end             
            key = waitforbuttonpress;
            while(key==0)
                key = waitforbuttonpress;
            end
            
        end%��֡����ѭ������        
    end%��Ƭ���ݴ���ѭ������
    %============================================================================
    
    testID = testID + 1;
end
fclose(fidList);