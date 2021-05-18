%% TXBF_Create_PSCal_Advanced_Frame_Config.m
% script to convert the TX phase shifter calibration matrix data
% to a form that can be used by the existing TXBF Advanced Frame
% Configuration example setup. 
%
% phaseOffsetValues = [numDevices, numTXPerDevice, numRXPerStystem, numPSOffsets]
% Ph = [numPSOffsets - 1, numRXPerSystem, bum]
%
clear all;
close all;

%%load TX phase shifter calibration file
phaseShiftCalFile = 'R:\Radar_Data\20200721_phase_shifter_cal_testing\20200723_testdata_SN5733600017_outdoor_cal1\calibrateTXPhaseResults.mat'; 
load(phaseShiftCalFile);

numTX = 12;
numRX = 16;
numPSOffsets = 64;


%% map phaseOffsetValues to Ph matrix format

Ph = zeros(numPSOffsets - 1, numRX, numTX);

for idxTX = 1:numTX
    for idxRX = 1:numRX
       for idxPSOffsets = 1:numPSOffsets - 1

           Ph(idxPSOffsets, idxRX, idxTX) = phaseOffsetValues(idxTX, idxRX, idxPSOffsets + 1);

       end 
    end
end   


%% graph both cal matrices
fig1 = figure(1);
axes1 = axes;
fig2 = figure(2);
axes2 = axes;


for idxTX = 1:numTX
    for idxRX = 1%:numRX * numDevices                
        plot(axes1, squeeze(phaseOffsetValues(idxTX, idxRX, :)));
        hold(axes1, 'on');
    end
end   


for idxTX = 1:numTX 
    for idxRX = 1%:numRX * numDevices
            plot(axes2, squeeze(Ph(:, idxRX, idxTX)));
            hold(axes2, 'on');
    end
end   

title(axes1, 'phaseOffsetValues(idxDevices, idxTX, idxRX, idxPhaseOffsets)');
title(axes2, 'Ph(idxPhaseOffsets, idxRX, idxTX)');

% save calibration data to .mat file
calibrateTXPhaseResultsFile = [pwd, '\phaseShifterCalibration.mat'];
save(calibrateTXPhaseResultsFile, 'Ph');