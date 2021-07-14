clearvars
% close all;
clc;

%% ��������

%����֡��
numFrames_toRun = inf; %number of frame to run, can be less than the frame saved in the raw data
%�Ƿ�������һ֡�����ڵ�һ֡�������ѣ�TI�ٷ�����������һ֡
abandonFirstFrame_ON = 0;
%������һ֡����
KEY_ON = 1;% 1: on; 0:off
%��ӡ������Ϣ
PRINT_INFO_ON = 0;

%�Ƿ����ԭʼ�����ļ����е�config.mmwave.json�ļ��������ɲ����ļ�
PARAM_FILE_GEN_ON = 1;

%�������Ƿ���Ҫ��ͼչʾ
PLOT_ON = 1; % 1: turn plot on; 0: turn plot off
%������չʾ�Ƿ���ö�������
LOG_ON = 1; % 1: log10 scale; 0: linear scale
%��������ͼ
DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP = 0 ; % Will make things slower

%����ƽ̨����
dataPlatform = 'TDA2';
%% 

%input�ļ���Ŀ¼
input_path = strcat(pwd,'\input\');
%���������������ļ�Ŀ¼
testList = strcat(input_path,'testList_view.txt');
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
        for frameIdx = min(2, abandonFirstFrame_ON+1):1:min(numValidFrames,1+numFrames_toRun)
            
            if abandonFirstFrame_ON==1 && frameIdx==2
                cnt_frameGlobal = cnt_frameGlobal+1;
            end
             %ȫ��֡������+1
            cnt_frameGlobal = cnt_frameGlobal+1;
            
            disp('===========================================================');
            fprintf('���ڷ��ʵ� %s Ƭ�е� %d/%d ֡��ȫ�ֵĵ� %d/%d ֡��\n',  fileIdx_unique{i_file}, frameIdx, numValidFrames,...
                cnt_frameGlobal, simTopObj.totNumFrames);
            
            
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
            timestamps{cnt_frameGlobal} = getTimestamp(fullfile(dataFolder_test, fileNameStruct.masterIdxFile), frameIdx);
            if cnt_frameGlobal ==1 || isempty(timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(timestamps{cnt_frameGlobal} - timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('���ײ��״�ʱ��� %d������һ֡��� %.3f ms\n', timestamps{cnt_frameGlobal}, diff_timestamp);
            %pc_timestamp
            pc_timestamps{cnt_frameGlobal} = startTime + (cnt_frameGlobal-1)*frameInterval*1000000;%16λUNIXʱ��
            if cnt_frameGlobal ==1 || isempty(pc_timestamps{cnt_frameGlobal-1})
                diff_timestamp = NaN;
            else
                diff_timestamp = double(pc_timestamps{cnt_frameGlobal} - pc_timestamps{cnt_frameGlobal-1})/1000;
            end
            fprintf('PC��ʱ��� %d������һ֡��� %.3f ms\n', pc_timestamps{cnt_frameGlobal}, diff_timestamp);
            
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
            
%             %�궨ʱ��ͨ������ȥ���������ҽǷ�����
%             base = load('./temp');
%             sig_integrate = sig_integrate - base.sig_integrate;
            
            % CFAR done along only TX and RX used in MIMO array
            % ����detection��CFAR������datapathλ��module��detection
            detection_results = datapath(detectionObj, DopplerFFTOut);
            
            detect_all_points = [];%��ʼ����⵽�ĵ����Ϣ����
            for iobj = 1:length(detection_results)
                detect_all_points (iobj,1)=detection_results(iobj).rangeInd+1;%range index
                detect_all_points (iobj,2)=detection_results(iobj).dopplerInd_org+1;%doppler index
                detect_all_points (iobj,4)=detection_results(iobj).estSNR;%����ǿ��
            end
            
            if PLOT_ON==1
                if KEY_ON==1
                    figure(1);
                else
                    figure();
                end
                set(gcf,'units','normalized','outerposition',[0.1 0.2 0.8 0.8]);
                
                subplot(2,2,1);
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
                title(['Range Profile(zero Doppler - thick green line): frameID ' num2str(cnt_frameGlobal)]);
                hold off;
                
                subplot(2,2,2);
                % ���壺Doppler Map
                % �����ꡪ���ٶȣ������ꡪ������
                velocityList = (-size(sig_integrate,2)/2: size(sig_integrate,2)/2-1) * detectionObj.velocityBinSize;
                rangeList = (1: size(sig_integrate,1)) * detectionObj.rangeBinSize;                
                imagesc(velocityList, rangeList, sig_integrate);
                c = colorbar;
                c.Label.String = 'Relative Power(dB)';
                title(' Range/Velocity Plot');
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
                    
                    % ��ӡ������Ϣ
                    if PRINT_INFO_ON==1
                        fprintf('%14s%20s%20s%22s%14s%16s\n', 'X', 'Y', 'Z', 'Velocity', 'Range', 'estSNR');
                        for iprint = 1:size(xyz,1)
                            fprintf('%15f%15f%15f%15f%15f%15f\n', xyz(iprint,1), xyz(iprint,2), xyz(iprint,3), xyz(iprint,4), xyz(iprint,5), xyz(iprint,6));
                        end
                    end
                    
                    if PLOT_ON==1
                        moveID = find(abs(xyz(:,4))>=0);%���abs(doppler_corr)���ڵ���0��Ŀ��
                        subplot(2,2,4);%���Ƶ���ͼ
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
                        xlim([-rangeList(end) rangeList(end)]);
                        ylim([0 rangeList(end)]);
                        zlim([-4 4]);
                        colormap('jet');
                        view([0 90]);                        
                        title('3D point cloud');
                        subplot(2,2,4,'color', [0.8,0.8,0.8]);
                        hold on;
                        xyz_zero = xyz(xyz(moveID,4)==0, :);
                        scatter3(xyz_zero(:,1), xyz_zero(:,2), xyz_zero(:,3), 10, (xyz_zero(:,4)),'w', 'filled');
                        hold off;
                        
                        %plot range and azimuth heatmap
                        subplot(2,2,3)%���ƾ�̬Ŀ�꣬range��azimuth����ͼ
                        mode = 'static';%mode: 'static'/'dynamic'/'static+dynamic'
                        minRangeBinKeep =  5;
                        rightRangeBinDiscard =  20;
                        [mag_data_static(:,:,cnt_frameGlobal) mag_data_dynamic(:,:,cnt_frameGlobal) y_axis x_axis]= plot_range_azimuth_2D(detectionObj.rangeBinSize, DopplerFFTOut,...
                            length(calibrationObj.IdTxForMIMOProcess),length(calibrationObj.RxForMIMOProcess), ...
                            detectionObj.antenna_azimuthonly, LOG_ON, mode, PLOT_ON, minRangeBinKeep, rightRangeBinDiscard);
                        title('range/azimuth heat map static objects');
                       
                        
                        % ���ƶ�̬��̬����ͼ
                        if DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP==1
                            if KEY_ON==1
                                figure(2);
                            else
                                figure();
                            end
                            subplot(121);%���ƾ�̬Ŀ�꣬range��azimuth����ͼ
                            surf(y_axis, x_axis, (mag_data_static(:,:,cnt_frameGlobal)).^0.1,'EdgeColor','none');
                            view(2);
                            xlabel('meters');    ylabel('meters');
                            axis('equal');
                            title({'Static Range-Azimuth Heatmap',strcat('Current Frame Number = ', num2str(cnt_frameGlobal))})

                            subplot(122);%���ƶ�̬Ŀ�꣬range��azimuth����ͼ
                            surf(y_axis, x_axis, (mag_data_dynamic(:,:,cnt_frameGlobal)).^0.4,'EdgeColor','none');
                            view(2);    
                            xlabel('meters');    ylabel('meters');
                            axis('equal');
                            title('Dynamic HeatMap');
                        end
    
                    end                    
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