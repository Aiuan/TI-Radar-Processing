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

% TXBF_BFPS_LUT_Generate.m
% This script generates an beam-steering angle to phase-shifter offset
% lookup table that can be used with the AWRx Cascade EVM kit
%%

clear all;
close all hidden;

%% setup linear array variables
lambda = 3.900424 * 10 ^ -3;    % lambda in meters
d = 2 * lambda;                 % array separation in lambda
numAnts = 9;                    % number of antenna elements in the azimuth array
deltaPhi = 360/64;              % ideal AWRx phase shifter discrete value in degrees

deltaTheta = 1.0;               % beam-steering angle offset
%theta = 80:deltaTheta:100;     % beam-steering angle array -- defined
theta = -15:deltaTheta:15;      % beam-steering angle array
numPsOffsets = 64;

psOffsetCal = zeros(numAnts, numPsOffsets); % phase offset cal LUT 

%% other options
LOADCALDATA = 1;
LOADTESTCALDATA = 0;
DEBUG_PLOTS = 1;

%% load and process phase shifter offset cal data

%phase Shifter calibration file
phaseShiftCalFile = 'C:\Radar_Data\20200721_phase_shifter_cal_testing\20200723_testdata_SN5733600017_outdoor_cal1\calibrateTXPhaseResults.mat'; 
load(phaseShiftCalFile);

% Read in AWRx #2, #3, and #4 data - this is the azimuth linear array
% Only looking at RX1 dataset
% unwrap data so it is zero referenced to 360 degrees
if(LOADCALDATA) % load real data
    
    SelectedRX = 1;
    for txIdx = 1:12   
        for psIdx = 1:63     

            tempOffset = -phaseOffsetValues(txIdx, SelectedRX, psIdx); % import the phaseOffsetValues matrix RX1 channel from 
                                                                      % phaseShiftCalFile, and flip sign

            if(tempOffset < 0)  %map raw phase phase offsets to 0 to 360 deg range
                tempOffset = tempOffset + 360;
            end

            phaseOffsetValuesUnwrap(txIdx, SelectedRX, psIdx) = tempOffset;  


        end
    end
   
    
end

if(LOADTESTCALDATA) % generate some test data
    
    for txIdx = 1:12  
        for psIdx = 1:63     
            phaseOffsetValuesUnwrap(txIdx, SelectedRX, psIdx) = psIdx * deltaPhi;
        end
    end
    
end

if(DEBUG_PLOTS) % debug plog of phase shifter cal LUT dataset
    fig1 = figure(1);
    axes1 = axes;
    
    for txIdx = 1:12      

        plot(axes1, squeeze(phaseOffsetValuesUnwrap(txIdx, SelectedRX, :)));
        hold(axes1, 'on');

    end

    
    title(axes1, 'Phase Offset Values (360 Unwrap) (degrees) vs. Phase Shifter Programmed Value');
    xlabel(axes1, 'Phase Shifter Programmed Value');
    ylabel(axes1, 'Phase Offset Values (degrees)');
    legend(axes1, 'dev1, tx1', 'dev1, tx2', 'dev1, tx3', 'dev2, tx1', 'dev2, tx2', 'dev2, tx3', 'dev3, tx1', 'dev3, tx2', 'dev3, tx3', 'dev4, tx1', 'dev4, tx2', 'dev4, tx3');

end

% Map cal LUT data (phaseOffsetValuesUnwrap) to azimuth linear array elements. 
% MMWCAS-RF-EVM uses AWRx #2, #3 and #4 elements. The AWRx #1 TX are used
% to form the MSA elevation array. 
%
% Array TX1 = AWR2 TX1
% Array TX2 = AWR2 TX2
% Array TX3 = AWR2 TX3
% Array TX4 = AWR3 TX1
% Array TX5 = AWR3 TX2
% Array TX6 = AWR3 TX3
% Array TX7 = AWR4 TX1
% Array TX8 = AWR4 TX2
% Array TX9 = AWR4 TX1
antIdx = 1;
for devIdx = 1:4 
    for txIdx = 1:3   
        
        psOffsetCal(antIdx, 1) = 0;
        
        for psIdx = 2:64     
             psOffsetCal(antIdx, psIdx) = phaseOffsetValuesUnwrap(txIdx, SelectedRX, psIdx-1);
        end
        
        antIdx = antIdx + 1;
    
    end
end



%% initialize output arrays
psSettingIdeal = zeros(length(theta), numAnts);
psSettingProgram = zeros(length(theta), numAnts);
psSettingProgramVal = zeros(length(theta), numAnts);
psSettingProgramCal = zeros(1, numAnts);
psSettingProgramCalVal = zeros(1, numAnts);
psError = zeros(length(theta), numAnts);


%% calculated TX phase calibration data
for thetaIdx = 1:length(theta)
   
   [psSettingIdeal(thetaIdx, :), psSettingProgram(thetaIdx, :), ... 
    psSettingProgramVal(thetaIdx, :), psSettingProgramCal(thetaIdx, :), ... 
    psSettingProgramCalVal(thetaIdx, :), psError(thetaIdx, :)] = ... 
    TXBF_Calc_Phase_Settings(theta(thetaIdx), lambda, d, numAnts, psOffsetCal);
    
end


%% plot results
[X,Y] = meshgrid(1:9, theta(:));


fig5 = figure('position', [50 50 50 + 600 50 + 800]);
axes5 = subplot(3, 1, 1);
colormap 'cool';
surf(axes5, X, Y, psSettingIdeal);
view([-90 90]);
title(axes5, 'Discrete Phase-Shifter Value (Calculated, ideal value. No calibration.)')
xlabel(axes5, 'TX Antenna Number');
ylabel(axes5, 'Beam-Steering Angle (degrees)');
axes5colorbar = colorbar;
ylabel(axes5colorbar, 'degrees');
grid off;

%fig6 = figure(6);
%axes6 = axes;
%colormap 'cool';
%surf(axes6, X, Y, psSettingProgram);
%view(axes6, [-90 90])
%colorbar
%grid off;


%fig7 = figure(7);
axes7 = subplot(3, 1, 2);
colormap 'cool';
surf(axes7, X, Y, psSettingProgramCalVal);
view([-90 90]);
title(axes7, 'Discrete Phase-Shifter Value (Calibrated)')
xlabel(axes7, 'TX Antenna Number');
ylabel(axes7, 'Beam-Steering Angle (degrees)');
axes7colorbar = colorbar;
ylabel(axes7colorbar, 'degrees');
grid off;


%fig8 = figure(8);
axes8 = subplot(3, 1, 3);
colormap 'cool';
surf(axes8, X, Y, psError);
view([-90 90]);
title(axes8, 'Phase-Shifter Settings (Calibration Error)')
xlabel(axes8, 'TX Antenna Number');
ylabel(axes8, 'Beam-Steering Angle (degrees)');
axes8colorbar = colorbar;
ylabel(axes8colorbar, 'degrees');
grid off;

%% write calibrated phase shifter programming settings to file
csvwrite('psSettingProgramCal.csv', psSettingProgramCal)
csvwrite('psSettingProgram.csv', psSettingProgram)

fprintf("Phase shifter programming (calibrated) data written to: \n%s\\psSettingProgramCal.csv\n", pwd);
fprintf("Phase shifter programming (uncalibrated) data written to: \n%s\\psSettingProgram.csv\n", pwd);
