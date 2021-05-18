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


% moudle_param_antennaCalib.m
%  
% Contains a list of inital parameters for genCalibrationMatrixObj modules
% used for antenna calibration. It is important to know that each parameter needs to be defined as
% moudleName_parameterName, the parameterName is defined in the
% corresponding module. Users need to know what parameters each module have
% before change this file.


platform = 'TI_4Chip_CASCADE';

%% fixed antenna ID and postion values for TI 4-chip cascade board. Should not be changed if user is based on TI board


% Format TDA
TI_Cascade_TX_position_azi = [11 10 9 32 28 24 20 16 12 8 4 0 ];%12 TX antenna azimuth position on TI 4-chip cascade EVM
TI_Cascade_TX_position_ele = [6  4  1  0  0  0  0  0  0 0 0 0];%12 TX antenna elevation position on TI 4-chip cascade EVM

TI_Cascade_RX_position_ele = zeros(1,16);%16 RX antenna elevation position on TI 4-chip cascade EVM

TI_Cascade_RX_position_azi = [ 11:14 50:53 46:49 0:3  ];
TI_Cascade_RX_ID = [13 14 15 16 1 2 3 4 9 10 11 12 5 6 7 8 ]; %RX channel order on TI 4-chip cascade EVM
TI_Cascade_Antenna_DesignFreq = 76.8; % antenna distance is designed for this frequency

%% constants
speedOfLight        = 3e8;
scaleFactor         = [0.0625, 0.03125, 0.015625, 0.0078125, 0.00390625, 0.001953125, 0.0009765625, 0.00048828125]*4;

%% define TX/RX antennas used for virtual array analysis. It can be a subset of the antennas enabled in data capture phase

RxForMIMOProcess = TI_Cascade_RX_ID; %using all 16 RXs, user can also choose subset of RXs for MIMO data analysis
D_TX = TI_Cascade_TX_position_azi(TxToEnable); %TX azimuth antenna coordinates
D_TX_ele = TI_Cascade_TX_position_ele(TxToEnable);%TX elevation antenna coordinates

D_RX = TI_Cascade_RX_position_azi(RxForMIMOProcess); %RX azimuth antenna coordinate
D_RX_ele = TI_Cascade_RX_position_ele(RxForMIMOProcess);%RX elevation antenna coordinate

%% derived parameters
DopplerFFTSize = 2^(ceil(log2(nchirp_loops)));
numChirpsPerFrame = nchirp_loops*numTxAnt;
chirpRampTime       = numADCSample/adcSampleRate;
chirpBandwidth      = chirpSlope * chirpRampTime; % Hz
chirpInterval       = chirpRampEndTime + chirpIdleTime;
carrierFrequency    = startFreqConst +  (adcStartTimeConst + chirpRampTime/2)*chirpSlope; % Hz center frequency
lambda              = speedOfLight/carrierFrequency;
maximumVelocity     = lambda / (chirpInterval*4) ; % m/s
maxRange            = speedOfLight*adcSampleRate*chirpRampTime/(2*chirpBandwidth);
numSamplePerChirp   = round(chirpRampTime*adcSampleRate);
rangeFFTSize        = 2^(ceil(log2(numSamplePerChirp)));
numChirpsPerVirAnt  = nchirp_loops;
numVirtualRxAnt     = length(TxToEnable) * length(RxForMIMOProcess) ;
rangeResolution     = speedOfLight/2/chirpBandwidth;
rangeResolution     = rangeResolution*numSamplePerChirp/rangeFFTSize;
velocityResolution  = lambda / (2*nchirp_loops * chirpInterval*numTxAnt);
velocityResolution  = velocityResolution*numChirpsPerVirAnt/DopplerFFTSize;


%% generate calibration matrix module parameters
calibrationInterp              = 5;     %interpolation factor used for range FFT for frequency calibration, determined at calibration stage
genCalibrationMatrixCascade_enable = 1;
genCalibrationMatrixCascade_calibrateFileName = [];
genCalibrationMatrixCascade_targetRange = 0;
genCalibrationMatrixCascade_frameIdx = 1;  % always use the first frame to generate calibration matrix
genCalibrationMatrixCascade_numSamplePerChirp = numSamplePerChirp;
genCalibrationMatrixCascade_Sampling_Rate_sps = adcSampleRate;
genCalibrationMatrixCascade_nchirp_loops = nchirp_loops;
genCalibrationMatrixCascade_numChirpsPerFrame = numChirpsPerFrame;
genCalibrationMatrixCascade_TxToEnable = TxToEnable;
genCalibrationMatrixCascade_Slope_calib = chirpSlope;
genCalibrationMatrixCascade_calibrationInterp = calibrationInterp;
genCalibrationMatrixCascade_TI_Cascade_RX_ID = TI_Cascade_RX_ID;
genCalibrationMatrixCascade_RxForMIMOProcess = RxForMIMOProcess;
genCalibrationMatrixCascade_TxForMIMOProcess = TxToEnable;
genCalibrationMatrixCascade_numRxToEnable = numRxToEnable;
genCalibrationMatrixCascade_phaseCalibOnly = 0; % 1: only phase calibration; 0: phase and amplitude calibration
genCalibrationMatrixCascade_adcCalibrationOn = 1; %1: adc data calibration on; 0 calibration off
genCalibrationMatrixCascade_rangeResolution = rangeResolution;
genCalibrationMatrixCascade_dataPlatform = dataPlatform;
genCalibrationMatrixCascade_NumDevices = NumDevices; 
genCalibrationMatrixCascade_binDataFile = [];
genCalibrationMatrixCascade_RxOrder = TI_Cascade_RX_ID;

