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

%calibrationCascade.m
%
% calibrationCascade module definition. This module calibrates the ADC data with the calibration
% matrix installed with the path name given by calibrationfilePath.
% Calibration is done directly on the raw ADC data before any further
% processing. Apply frequency and phase calibration in time domain; amplitude
% calibration is optional, can be turned on or off


%% Class definition
classdef calibrationCascade < Module
    
    %% properties
    properties (Access = public)
        %method       
       binfilePath = []   
       calibrationfilePath = []   
       frameIdx = 1
       adcCalibrationOn = 1;  
       
       %chirpParameters
       numSamplePerChirp = 0
       nchirp_loops = 0
       numChirpsPerFrame = 0
       TxToEnable = []
       Slope_calib = 0
       Sampling_Rate_sps = 0;
       fs_calib = 0;
       chirpSlope = 0
       calibrationInterp = 0
       TI_Cascade_RX_ID = []
       RxForMIMOProcess = []
       IdTxForMIMOProcess = []
       numRxToEnable = 0
       phaseCalibOnly = 0
       ADVANCED_FRAME_CONFIG = 0
       N_TXForMIMO = 0
       NumAnglesToSweep = 0
       dataPlatform = [];
       RxOrder = [];
       NumDevices = 4;
       
    end
    
    methods
        
        %% constructor
        function obj = calibrationCascade(varargin)
            if(isempty(find(strcmp(varargin,'name'), 1)))
                varargin = [varargin, 'name','calibrationCascade'];
            end
            obj@Module(varargin{:});
            
            % Set parameters
            obj.enable    = getParameter(obj, 'enable');    
            obj.binfilePath = getParameter(obj, 'binfilePath');          
            obj.calibrationfilePath = getParameter(obj, 'calibrationfilePath');       
            obj.frameIdx = getParameter(obj, 'frameIdx');             
            obj.numSamplePerChirp = getParameter(obj, 'numSamplePerChirp');    
            obj.nchirp_loops = getParameter(obj, 'nchirp_loops');            
            obj.numChirpsPerFrame = getParameter(obj, 'numChirpsPerFrame');             
            obj.TxToEnable = getParameter(obj, 'TxToEnable');            
            obj.Slope_calib = getParameter(obj, 'Slope_calib');            
            obj.Sampling_Rate_sps = getParameter(obj, 'Sampling_Rate_sps');  
            obj.fs_calib = getParameter(obj, 'fs_calib');  
            obj.chirpSlope = getParameter(obj, 'chirpSlope');             
            obj.calibrationInterp = getParameter(obj, 'calibrationInterp');  
            obj.TI_Cascade_RX_ID = getParameter(obj, 'TI_Cascade_RX_ID');             
            obj.RxForMIMOProcess = getParameter(obj, 'RxForMIMOProcess');             
            obj.IdTxForMIMOProcess = getParameter(obj, 'IdTxForMIMOProcess');           
            obj.numRxToEnable = getParameter(obj, 'numRxToEnable');             
            obj.phaseCalibOnly = getParameter(obj, 'phaseCalibOnly');           
            obj.adcCalibrationOn = getParameter(obj, 'adcCalibrationOn');  
            obj.ADVANCED_FRAME_CONFIG = getParameter(obj, 'ADVANCED_FRAME_CONFIG');  
            if obj.ADVANCED_FRAME_CONFIG  == 1
                obj.N_TXForMIMO = getParameter(obj, 'N_TXForMIMO');
                obj.NumAnglesToSweep = getParameter(obj, 'NumAnglesToSweep');
            end
            obj.dataPlatform = getParameter(obj, 'dataPlatform');  
            obj.RxOrder = getParameter(obj, 'RxOrder');  
            obj.NumDevices = getParameter(obj, 'NumDevices');  
            
          
                       
            
            % overwritten the property value inside parameter file
            %setProperties(obj, nargin, varargin{:});
            obj = set(obj, varargin{:});
            
        end
        
        %% datapath function
        % input: adc data, assuming size(input) = [numSamplePerChirp, numChirpsPerFrame numAntenna]
        [out] = datapath(obj)
                  
      
        
    end
    
end

