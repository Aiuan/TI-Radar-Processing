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

% cascade_Read_TX_Cal_Data.m
% This script reads in the TX cal datasets
%%
function calData = cascade_Read_TX_Cal_Data(obj)

    Samples_per_Chirp = obj.numSamplePerChirp;
    nchirp_loops = obj.nchirp_loops;
    Chirps_per_Frame = obj.numChirpsPerFrame;
    Sampling_Rate_sps = obj.Sampling_Rate_sps;
    TxToEnable = obj.TxToEnable;
    numTX = length(obj.TxToEnable);
    interp_fact = obj.calibrationInterp;
    Slope_calib = obj.Slope_calib;
    frameIdx = obj.frameIdx;
    binDataFile = obj.binDataFile;
    calibrateFileName = obj.calibrateFileName;
    targetRange = obj.targetRange;
    RxOrder = obj.RxOrder;

    numRX = obj.numRxToEnable;
    NumDevices = obj.NumDevices;

    range_bin_search_min = round((Samples_per_Chirp)*interp_fact*((targetRange-2)*2*Slope_calib/(3e8*Sampling_Rate_sps))+1,0);
    range_bin_search_max = round((Samples_per_Chirp)*interp_fact*((targetRange+2)*2*Slope_calib/(3e8*Sampling_Rate_sps))+1,0);

    switch obj.dataPlatform  
         case 'TDA2'
            numChirpPerLoop = length(obj.TxToEnable);
            numLoops = obj.nchirp_loops; % Only valid for TDM-MIMO
            numRXPerDevice = 4; % Fixed number  
            [radar_data_Rxchain] = read_ADC_bin_TDA2_separateFiles(obj.binDataFile, frameIdx, Samples_per_Chirp, numChirpPerLoop, numLoops, numRXPerDevice, 1);
        
        otherwise
            error('Not supported data capture platform!');
    end

    calData = radar_data_Rxchain;


end