%  Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

% cascade_TX_Phase_Calibration.m
%  
% Top level main test chain to perform TX phase shifter calibration. 
% Output is saved to calibrateTXPhaseResults.mat
% 
% Usage: modify the dataFolder_calib_data_path 

tic
clearvars
close all

pro_path = getenv('CASCADE_SIGNAL_PROCESSING_CHAIN_MIMO');
input_path = strcat(pro_path,'\main\cascade\input\');
dataPlatform = 'TDA2'

DEBUG_PLOTS = 0;                % optionally display debug plots while calibration data is being processed 

numDevices = 4;                 % number of AWRx devices being processed, set to 4, for the full MMWCAS-RF-EVM device array
numTX = 3;                      % number of AWRx TX channels being processed, set to 3, for the full AWRx device channels
numRX = 16;                     % number of MMWCAS-RF-EVM RX channels being processed, set to 16, for the full MMWCAS-RF-EVM RX channels
numPhaseShifterOffsets = 64;    % number of phase-shifter offset increments being processed, set to 64 for full phase-shifter range (number of datasets)
numChirpsLoopsPerFrame = 12;    % number of chirp-loops per frame. Only a single TX active per chirp-loop. 
numChirpsPerLoop = 64;          % number of chirps per TX active phase. 
numSamplesPerChirp = 256;       % number of samples per chirp
searchBinsSkip = 50;            % number of bins to skip when looking for peak from corner reflector 

% select reference TX/Device channel for computing offsets - determined by 
% TX antenna array geometry and phase-shifter offset utilization (all RX
% channels are processed)
refTX = 1;
refRX = 1;
refPhaseOffset = 1;

targetRange = 5.0;  % estimated corner-reflector target range (in meters) 
dataFolder_calib_data_path = 'C:\Radar_Data\20200721_phase_shifter_cal_testing\20200722_testdata_SN5733600018_outdoor_cal2\'; % folder holding all of the calibration datasets

fig1 = figure(1); % FFT magnitude (dB)
axes1 = axes;

fig2 = figure(2); % FFT phase (deg)
axes2 = axes;

fig3 = figure(3); % accumulated target bin values
axes3 = axes;

fig4 = figure(4); % accumulated target distance values
axes4 = axes;

fig5 = figure(5); % accumulated target phase angle values
axes5 = axes;

fig6 = figure(6); % plot phase offset values
axes6 = axes;

fig7 = figure(7); % plot phase offset error values
axes7 = axes;

%parameter file name for the test
pathGenParaFile = [input_path,'generateClibrationMatrix_param.m'];

%important to clear the same.m file, since Matlab does not clear cache
%automatically
clear(pathGenParaFile);



% loop through each folder in the calibration directory create 
% matrix with dimensions [devices, number of TX, number of phase shift offsets] 
% with the phase values for the detected range-bin peak
dataFolder_calib_data_info = dir(dataFolder_calib_data_path);

iterationMax = numPhaseShifterOffsets;
iteration = 0;

% find the first folder of cal data
for folderIdx = 1:length(dataFolder_calib_data_info)
    if(dataFolder_calib_data_info(folderIdx).isdir && dataFolder_calib_data_info(folderIdx).name ~= "." && dataFolder_calib_data_info(folderIdx).name ~= "..") 
        folderIdxStart = folderIdx;
        break;
    end

end

folderIdx = folderIdxStart; % start at first index that is a folder

for idxPS = 1:numPhaseShifterOffsets % loop through phase shifter offset/transmitter/device

    if(dataFolder_calib_data_info(folderIdx).isdir) 
        disp(['dataFolder_calib_data_info(folderIdx).name = ', dataFolder_calib_data_info(folderIdx).name]);

        % create folder structure for this iteration        
        fileNameCascade.dataFolderName = strcat([dataFolder_calib_data_info(folderIdx).folder, '\', dataFolder_calib_data_info(folderIdx).name, '\']);
        [fileIdx_unique] = getUniqueFileIdx(fileNameCascade.dataFolderName);
        [fileNameStruct] = getBinFileNames_withIdx(fileNameCascade.dataFolderName, fileIdx_unique{1});                  

        %generate parameter file for the dataset
        parameter_file_gen_antennaCalib_json(fileNameCascade.dataFolderName, pathGenParaFile, dataPlatform);

        %generate calibration matrix object for the dataset
        genCalibrationMatrixObj = genCalibrationMatrixCascade('pfile', pathGenParaFile,...
        'calibrateFileName',fileNameCascade.dataFolderName, 'targetRange', targetRange);
        genCalibrationMatrixObj.binDataFile = fileNameStruct;% dataFolder_calib_data;%[dataFolder_calib_data listing.name];
    end
            
    % use second frame for calibration 
    genCalibrationMatrixObj.frameIdx = 2;             

    % read in .bin files
    %calData(deviceIdx, TXIdx, PSIdx, :, :, :) = cascade_Read_TX_Cal_Data(genCalibrationMatrixObj);
    calData = cascade_Read_TX_Cal_Data(genCalibrationMatrixObj);
          

    for idxTX = 1:numChirpsLoopsPerFrame % loop through each TX phase   
        for idxRX = 1:numRX % loop through each RX
        

            calData_1DFFT(:, :, idxTX, idxRX) = fftshift(fft(calData(:, :, idxRX, idxTX), genCalibrationMatrixObj.numSamplePerChirp, 1));
            calData_2DFFT(:, :, idxTX, idxRX) = 1/(numChirpsPerLoop) * fftshift(fft(calData_1DFFT(:, :, idxTX, idxRX), numChirpsPerLoop, 2));
            
          
            % find target peak bin in 2D-FFT 0-velocity bin (skip close
            % bins to avoid DC leakage or bumper reflections
            [TargetBinValue, TargetBinIdx] = max(abs(squeeze(calData_2DFFT(searchBinsSkip:numSamplesPerChirp, numChirpsPerLoop/2 + 1, idxTX, idxRX))));
            phaseValuesBin(idxTX, idxRX, idxPS) = TargetBinIdx + searchBinsSkip - 1;
            phaseValuesTargetDistance(idxTX, idxRX, idxPS) = TargetBinIdx * genCalibrationMatrixObj.rangeResolution;

            % record phase at target peak bin
            phaseValues(idxTX, idxRX, idxPS) = angle(squeeze(calData_2DFFT(phaseValuesBin(idxTX, idxRX, idxPS), numChirpsPerLoop/2 + 1, idxTX, idxRX))) * 180 / pi; 

            % debug plots 
            if(DEBUG_PLOTS)


                plot(axes1, 10*log(abs(squeeze(calData_2DFFT(:, numChirpsPerLoop/2 + 1, idxTX, idxRX)))));
                hold(axes1, 'on');
                plot(axes1, phaseValuesBin(idxTX, idxRX, idxPS), 10*log(abs(TargetBinValue)), '-o', 'color', 'red');
                hold(axes1, 'off');
                %titleString = sprintf("idxTX = %d, idxRX = %d, ")
                title(axes1, 'Calibration Target Power vs. IF bins');
                xlabel(axes1, '1D-FFT Spectrum (bins)'); 
                ylabel(axes1, '1D-FFT Magnitude (dB)');

                plot(axes2, angle(squeeze(calData_2DFFT(:, numChirpsPerLoop/2 + 1, idxTX, idxRX))) * 180 / pi );
                hold(axes2, 'on');
                plot(axes2, phaseValuesBin(idxTX, idxRX, idxPS), phaseValues(idxTX, idxRX, idxPS) , '-o', 'color', 'red');
                hold(axes2, 'off');                
                title(axes2, 'Calibration Target Phase vs. IF bins');
                xlabel(axes2, '1D-FFT Spectrum (bins)');
                ylabel(axes2, '1D-FFT Phase (degrees)');

                plot(axes3, squeeze(phaseValuesBin(idxTX, idxRX, :)));                
                title(axes3, 'Calibration Target Detected Index');
                xlabel(axes3, 'Phase Shifter Offset (5.625 degrees/LSB)');
                ylabel(axes3, 'Calibration Target Sampled IF Index');


                plot(axes4, squeeze(phaseValuesTargetDistance(idxTX, idxRX, :)));
                title(axes4, 'Calibration Target Distance');
                xlabel(axes4, 'Phase Shifter Offset (5.625 degrees/LSB)');
                ylabel(axes4, 'Target Distance (meters)');


                plot(axes5, squeeze(phaseValues(idxTX, idxRX, :)));
                title(axes5, 'Calibration Target Phase vs. IF bins');
                xlabel(axes5, 'Phase Shifter Offset (5.625 degrees/LSB)');
                ylabel(axes5, 'Target 1D-FFT Phase (degrees)');            
                pause(0.01);


            end
            
            
        end   
        
        
        
    end

    folderIdx = folderIdx + 1;

end





% compute phase offsets and phase errors
for idxTX = 1:numChirpsLoopsPerFrame % loop through transmitter/device
    for idxRX = 1:numRX % loop through transmitter/device
        for idxPS = 1:numPhaseShifterOffsets % loop through phase shifter offset/transmitter/device

            % reference the phase-shifter 0 setting, from channel refDevice, refTX 
            % as reference and compute offset phase values for the other settings
            phaseOffsetValues(idxTX, idxRX, idxPS) = phaseValues(idxTX, idxRX, idxPS) - phaseValues(refTX, idxRX, refPhaseOffset); 
            %phaseOffsetValues(idxTX, idxRX, idxPS) = phaseValues(idxTX, idxRX, idxPS) - phaseValues(refTX, idxRX, idxPS); 

            % find error between actual phase offset and expected (ideal) phase offset
            % this error will be a combination of static phase error and phase-shifter induced error
            phaseOffsetError(idxTX, idxRX, idxPS) = phaseValues(idxTX, idxRX, idxPS) - idxPS * 5.625;

        end
    end
end




 
    phase_shift_values = 0:5.625:5.625*64-1;
   
    for TXIdx = 1:numChirpsLoopsPerFrame % loop through transmitter/device
        for RXIdx = 2:2 % loop through receivers   

            plot(axes6, squeeze(phaseValues(TXIdx, RXIdx, :)), 'color',[TXIdx/numChirpsLoopsPerFrame, TXIdx/numChirpsLoopsPerFrame,  RXIdx * 0.0625]);
            hold(axes6, 'on');

        end
    end
   

    
%% plot results
title(axes6, 'TX Channel Target Phase Value (deg) vs. TX Phase-Shifter Offset Value (5.625 deg increment)');
xlabel(axes6, 'TX Phase-Shifter Offset Value (5.625 deg increment)'); 
ylabel(axes6, 'TX Channel Absolute Phase (deg)'); 
legend(axes6, "TX1", "TX2", "TX3", "TX4", "TX5", "TX6", "TX7", "TX8", "TX9", "TX10", "TX11", "TX12");
grid(axes6, 'on');



for TXIdx = 1:numChirpsLoopsPerFrame % loop through transmitter/device
    for RXIdx = 1:1 % loop through receivers    

        plot(axes7, squeeze(phaseOffsetValues(TXIdx, RXIdx, :)), 'color',[TXIdx/numChirpsLoopsPerFrame, TXIdx/numChirpsLoopsPerFrame,  RXIdx * 0.0625]);
        hold(axes7, 'on');

    end
end


title(axes7, 'TX Channel Phase Offset (deg) vs. TX Phase-Shifter Offset Value (5.625 deg increment)');
xlabel(axes7, 'TX Phase-Shifter Offset Value (5.625 deg increment)'); 
ylabel(axes7, 'TX Channel Phase Offset (deg)'); 
legend(axes7, "TX1", "TX2", "TX3", "TX4", "TX5", "TX6", "TX7", "TX8", "TX9", "TX10", "TX11", "TX12");
grid(axes7, 'on');
    

%% save results
% save calibration data to .mat file
calibrateTXPhaseResultsFile = [dataFolder_calib_data_path '\calibrateTXPhaseResults.mat'];
save(calibrateTXPhaseResultsFile, 'phaseOffsetValues', 'phaseValues', 'phaseOffsetError');

% save full debug data to .mat file 
calibrateTXPhaseResultsFileFull = [dataFolder_calib_data_path 'calibrateTXPhaseResultsFileFull.mat'];
save(calibrateTXPhaseResultsFileFull);

toc
timeProcessing = toc;


