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

PLOT_ON = 1; % 1: turn plot on; 0: turn plot off
LOG_ON = 1; % 1: log10 scale; 0: linear scale
% numFrames_toRun = 10; %number of frame to run, can be less than the frame saved in the raw data
SAVEOUTPUT_ON = 0;
PARAM_FILE_GEN_ON = 1;
DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP = 0 ; % Will make things slower
dataPlatform = 'TDA2'

%% get the input path and testList
pro_path = 'E:\matlab_workspace\202011毫米波雷达测试实验\MatlabExamples\4chip_cascade_MIMO_example';
input_path = strcat(pro_path,'\main\cascade\input\');
testList = strcat(input_path,'testList.txt');
%path for input folder
fidList = fopen(testList,'r');
testID = 1;

while ~feof(fidList)
    
    %% get each test vectors within the test list
    % test data file name
    dataFolder_test = fgetl(fidList);    
   
    %calibration file name
    dataFolder_calib = fgetl(fidList);
    
    %module_param_file defines parameters to init each signal processing
    %module
    module_param_file = fgetl(fidList);
    
     %parameter file name for the test
    pathGenParaFile = [input_path,'test',num2str(testID), '_param.m'];
    %important to clear the same.m file, since Matlab does not clear cache
    %automatically
    clear(pathGenParaFile);
    
    %generate parameter file for the test to run
    if PARAM_FILE_GEN_ON == 1     
        parameter_file_gen_json(dataFolder_test, dataFolder_calib, module_param_file, pathGenParaFile, dataPlatform);
    end
    
    %load calibration parameters
    load(dataFolder_calib)
    
    % simTopObj is used for top level parameter parsing and data loading and saving
    simTopObj           = simTopCascade('pfile', pathGenParaFile);
    calibrationObj      = calibrationCascade('pfile', pathGenParaFile, 'calibrationfilePath', dataFolder_calib);
    rangeFFTObj         = rangeProcCascade('pfile', pathGenParaFile);
    DopplerFFTObj       = DopplerProcClutterRemove('pfile', pathGenParaFile);
    detectionObj        = CFAR_CASO('pfile', pathGenParaFile);
    DOAObj              = DOACascade('pfile', pathGenParaFile);
    
    % get system level variables
    platform            = simTopObj.platform;
    numValidFrames      = simTopObj.totNumFrames;
    cnt = 1;
    frameCountGlobal = 0;
    
    
   % Get Unique File Idxs in the "dataFolder_test"   
   [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);
    
    for i_file = 1:2%(length(fileIdx_unique))
        
       % Get File Names for the Master, Slave1, Slave2, Slave3   
       [fileNameStruct]= getBinFileNames_withIdx(dataFolder_test, fileIdx_unique{i_file});        
       
      %pass the Data File to the calibration Object
      calibrationObj.binfilePath = fileNameStruct;
        
      detection_results = [];  
        
       % Get Valid Number of Frames
       % 确定有多少有效帧，有效帧中数据大小
       [numValidFrames dataFileSize] = getValidNumFrames(fullfile(dataFolder_test, fileNameStruct.masterIdxFile));
        %intentionally skip the first frame due to TDA2 
       
        %舍弃第一帧，从第二帧起始
        for frameIdx = 2:1:numValidFrames;%numFrames_toRun
            tic
            %read and calibrate raw ADC data            
            calibrationObj.frameIdx = frameIdx;
            frameCountGlobal = frameCountGlobal+1
            %读取adc数据
            adcData = datapath(calibrationObj);
            
            % RX Channel re-ordering
            adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:);            
            
            %only take TX and RXs required for MIMO data analysis
            % adcData = adcData
            
            if mod(frameIdx, 10)==1
                fprintf('Processing %3d frame...\n', frameIdx);
            end
            
            
            %perform 2D FFT
            rangeFFTOut = [];
            DopplerFFTOut = [];
            
            for i_tx = 1: size(adcData,4)
                % range FFT
                rangeFFTOut(:,:,:,i_tx)     = datapath(rangeFFTObj, adcData(:,:,:,i_tx));
                
                % Doppler FFT
                DopplerFFTOut(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut(:,:,:,i_tx));
                
            end
            
  
            % CFAR done along only TX and RX used in MIMO array
            DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));
            
            %detection
            sig_integrate = 10*log10(sum((abs(DopplerFFTOut)).^2,3) + 1);
                        
            detection_results = datapath(detectionObj, DopplerFFTOut);
            detection_results_all{cnt} =  detection_results;
            
            detect_all_points = [];
            for iobj = 1:length(detection_results)
                detect_all_points (iobj,1)=detection_results(iobj).rangeInd+1;
                detect_all_points (iobj,2)=detection_results(iobj).dopplerInd_org+1;
                detect_all_points (iobj,4)=detection_results(iobj).estSNR;
            end
            
            if PLOT_ON
                figure(1);
                set(gcf,'units','normalized','outerposition',[0 0 1 1])                
                subplot(2,2,1)               
                plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,size(sig_integrate,2)/2+1),'g','LineWidth',4);hold on; grid on
                for ii=1:size(sig_integrate,2)
                    plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,ii));hold on; grid on
                    if ~isempty(detection_results)
                        ind = find(detect_all_points(:,2)==ii);
                        if (~isempty(ind))
                            rangeInd = detect_all_points(ind,1);
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
                imagesc((sig_integrate))
                c = colorbar;
                c.Label.String = 'Relative Power(dB)';
                title(' Range/Velocity Plot');
                pause(0.01)
            end
            
            angles_all_points = [];
            xyz = [];
            %if 0
            if ~isempty(detection_results)
                % DOA, the results include detection results + angle estimation results.
                % access data with angleEst{frame}(objectIdx).fieldName
                angleEst = datapath(DOAObj, detection_results);
                
                if length(angleEst) > 0
                    for iobj = 1:length(angleEst)
                        angles_all_points (iobj,1:2)=angleEst(iobj).angles(1:2);
                        angles_all_points (iobj,3)=angleEst(iobj).estSNR;
                        angles_all_points (iobj,4)=angleEst(iobj).rangeInd;
                        angles_all_points (iobj,5)=angleEst(iobj).doppler_corr;
                        angles_all_points (iobj,6)=angleEst(iobj).range;
                        %switch left and right, the azimuth angle is flipped
                        xyz(iobj,1) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,1)*-1)*cosd(angles_all_points (iobj,2));
                        xyz(iobj,2) = angles_all_points (iobj,6)*cosd(angles_all_points (iobj,1)*-1)*cosd(angles_all_points (iobj,2));
                        %switch upside and down, the elevation angle is flipped
                        xyz(iobj,3) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,2)*-1);
                        xyz(iobj,4) = angleEst(iobj).doppler_corr;
                        xyz(iobj,9) = angleEst(iobj).dopplerInd_org;
                        xyz(iobj,5) = angleEst(iobj).range;
                        xyz(iobj,6) = angleEst(iobj).estSNR;
                        xyz(iobj,7) = angleEst(iobj).doppler_corr_overlap;
                        xyz(iobj,8) = angleEst(iobj).doppler_corr_FFT;
                        
                    end
                    angles_all_all{cnt} = angles_all_points;
                    xyz_all{cnt}  = xyz;
                    maxRangeShow = detectionObj.rangeBinSize*rangeFFTObj.rangeFFTSize;
                    %tic
                    if PLOT_ON
                        moveID = find(abs(xyz(:,4))>=0);
                        subplot(2,2,4);                        
                        
                        if cnt==1
                            scatter3(xyz(moveID,1),xyz(moveID,2),xyz(moveID,3),45,(xyz(moveID,4)),'filled');
                        else
                            yz = [xyz_all{cnt}; xyz_all{cnt-1}];
                            scatter3(xyz(moveID,1),xyz(moveID,2),xyz(moveID,3),45,(xyz(moveID,4)),'filled');
                        end
                        
                        c = colorbar;
                        c.Label.String = 'velocity (m/s)';                        
                        grid on;
                        
                        xlim([-20 20])
                        ylim([1 maxRangeShow])
                        %zlim([-4 4])
                        zlim([-5 5])
                        xlabel('X (m)')
                        ylabel('y (m)')
                        zlabel('Z (m)')                        
                        
                        view([-9 15])                        
                        title(' 3D point cloud');
                        
                        %plot range and azimuth heatmap
                        subplot(2,2,3)
                        STATIC_ONLY = 1;
                        minRangeBinKeep =  5;
                        rightRangeBinDiscard =  20;
                        [mag_data_static(:,:,frameCountGlobal) mag_data_dynamic(:,:,frameCountGlobal) y_axis x_axis]= plot_range_azimuth_2D(detectionObj.rangeBinSize, DopplerFFTOut,...
                            length(calibrationObj.IdTxForMIMOProcess),length(calibrationObj.RxForMIMOProcess), ...
                            detectionObj.antenna_azimuthonly, LOG_ON, STATIC_ONLY, PLOT_ON, minRangeBinKeep, rightRangeBinDiscard);
                        title('range/azimuth heat map static objects')
                       
                        
    if (DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP)                   
    figure(2)
    subplot(121);
    surf(y_axis, x_axis, (mag_data_static(:,:,frameCountGlobal)).^0.1,'EdgeColor','none');
    view(2);
    xlabel('meters');    ylabel('meters')
    title({'Static Range-Azimuth Heatmap',strcat('Current Frame Number = ', num2str(frameCountGlobal))})
    
    subplot(122);
    surf(y_axis, x_axis, (mag_data_dynamic(:,:,frameCountGlobal)).^0.4,'EdgeColor','none');
    view(2);    
    xlabel('meters');    ylabel('meters')
    title('Dynamic HeatMap')
    end
    pause(0.1) 

     
                    end
                    
                end
                
            end
                             
            cnt = cnt + 1;    
       toc    
        end
        
        
    end
    
    ind = strfind(dataFolder_test, '\');
    
    date = dataFolder_test(ind(end-2)+1:(ind(end-1)-1));%日期
    group = dataFolder_test(ind(end)+1:end);%组号
    if SAVEOUTPUT_ON == 1
        save(['.\output\', date,'_',group,'.mat'],'angles_all_all', 'detection_results_all','xyz_all');
    end
    testID = testID + 1;
    
end
