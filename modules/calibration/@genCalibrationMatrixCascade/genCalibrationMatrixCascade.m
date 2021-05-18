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

%genCalibrationMatrixCascade.m
%
% genCalibrationMatrixCascade module definition. This module generates the
% calibration maxtrix for frequency and phase calibration. The calibration
% data is given by calibrateFileName. The calibration results are saved in calibResult


%% Class definition
classdef genCalibrationMatrixCascade < Module
    
    %% properties
    properties (Access = public)
        %method 
       calibrateFileName = []   
       targetRange  = 0       
       frameIdx = 1
       
       %chirpParameters
       numSamplePerChirp = 0
       nchirp_loops = 0
       numChirpsPerFrame = 0
       TxToEnable = []
       Slope_calib = 0
       Sampling_Rate_sps = 0;
       chirpSlope = 0
       calibrationInterp = 0
       TI_Cascade_RX_ID = []
       RxForMIMOProcess = []
       TxForMIMOProcess = []
       numRxToEnable = 0
       rangeResolution = 0;
       dataPlatform = [];
       NumDevices = 4;
       binDataFile = [];
       RxOrder = [];
       
    end
    
    methods
        
        %% constructor
        function obj = genCalibrationMatrixCascade(varargin)
            if(isempty(find(strcmp(varargin,'name'), 1)))
                varargin = [varargin, 'name','genCalibrationMatrixCascade'];
            end
            obj@Module(varargin{:});
            
            % Set parameters
            obj.enable = getParameter(obj, 'enable');    
            obj.calibrateFileName = getParameter(obj, 'calibrateFileName'); 
            obj.targetRange = getParameter(obj, 'targetRange');               
            obj.frameIdx = getParameter(obj, 'frameIdx');           
            obj.numSamplePerChirp = getParameter(obj, 'numSamplePerChirp');             
            obj.nchirp_loops = getParameter(obj, 'nchirp_loops');                
            obj.numChirpsPerFrame = getParameter(obj, 'numChirpsPerFrame');               
            obj.TxToEnable = getParameter(obj, 'TxToEnable');                
            obj.Slope_calib = getParameter(obj, 'Slope_calib');                
            obj.Sampling_Rate_sps = getParameter(obj, 'Sampling_Rate_sps');               
            obj.calibrationInterp = getParameter(obj, 'calibrationInterp');                
            obj.TI_Cascade_RX_ID = getParameter(obj, 'TI_Cascade_RX_ID');               
            obj.RxForMIMOProcess = getParameter(obj, 'RxForMIMOProcess');               
            obj.TxForMIMOProcess = getParameter(obj, 'TxForMIMOProcess');               
            obj.numRxToEnable = getParameter(obj, 'numRxToEnable');  
            obj.rangeResolution = getParameter(obj, 'rangeResolution');    
            obj.dataPlatform = getParameter(obj, 'dataPlatform');    
            obj.NumDevices = getParameter(obj, 'NumDevices'); 
            obj.binDataFile = getParameter(obj, 'binDataFile'); 
            obj.RxOrder = getParameter(obj, 'RxOrder'); 
            
                       
            % overwritten the property value inside parameter file
            %setProperties(obj, nargin, varargin{:});
            obj = set(obj, varargin{:});
            
        end
        
        %% datapath function
       
       calibResult = dataPath(obj)
                  
      
        
    end
    
end

