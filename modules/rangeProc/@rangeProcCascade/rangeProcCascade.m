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


%rangeProcCascade.m
%
%rangeProcCascade module definition. Perform range FFT


%% Class definition
classdef rangeProcCascade < Module
    
    %% properties
    properties (Access = public)
        %method 
        numAntenna = 16             % number of antennas
        numAdcSamplePerChirp = 256   % number of samples per chirp
        rangeFFTSize = 256           % FFT size       
        rangeWindowEnable  = 1     % flag to enable or disable windowing before range FFT
        rangeWindowCoeff  = []      % range FFT window coefficients (one side)
        rangeWindowCoeffVec = []    % range FFT window coefficients of length rangeFFTSize
        scaleFactorRange  = 1   
        FFTOutScaleOn = 0;
        
    end
    
    methods
        
        %% constructor
        function obj = rangeProcCascade(varargin)
            if(isempty(find(strcmp(varargin,'name'), 1)))
                varargin = [varargin, 'name','rangeProcCascade'];
            end
            obj@Module(varargin{:});
            
            % Set parameters
            obj.enable = getParameter(obj, 'enable');           
            obj.numAntenna = getParameter(obj, 'numAntenna');             
            obj.numAdcSamplePerChirp = getParameter(obj, 'numAdcSamplePerChirp');                  
            obj.rangeFFTSize = getParameter(obj, 'rangeFFTSize');  
            obj.rangeWindowEnable = getParameter(obj, 'rangeWindowEnable');            
            obj.rangeWindowCoeff = getParameter(obj, 'rangeWindowCoeff');             
            obj.scaleFactorRange = getParameter(obj, 'scaleFactorRange');            
            obj.FFTOutScaleOn = getParameter(obj, 'FFTOutScaleOn');  
            % set all coefficients to 1 if range windowing is disabled
            if ~obj.rangeWindowEnable
                obj.rangeWindowCoeff     = ones(length(obj.rangeWindowCoeff),1);
            end
            
            % form the range window coeffients vector
            rangeWinLen                 = length(obj.rangeWindowCoeff);
            rangeWindowCoeffVec         = ones(obj.numAdcSamplePerChirp, 1);
            rangeWindowCoeffVec(1:rangeWinLen) = obj.rangeWindowCoeff;
            rangeWindowCoeffVec(obj.numAdcSamplePerChirp-rangeWinLen+1:obj.numAdcSamplePerChirp) = rangeWindowCoeffVec(rangeWinLen:-1:1);
            obj.rangeWindowCoeffVec     = rangeWindowCoeffVec;
            
            
            % overwritten the property value inside parameter file
            %setProperties(obj, nargin, varargin{:});
            obj = set(obj, varargin{:});
            
        end
        
        %% datapath function
        % input: adc data, assuming size(input) = [numSamplePerChirp, numChirpsPerFrame numAntenna]
        function [out] = datapath(obj, input)
            
            numLines  = size(input,2);
            numAnt    = size(input,3);
            % dopplerWinLen  = length(obj.dopplerWindowCoeff);
            
            if obj.enable           
                
                % initialize
                out = zeros(obj.rangeFFTSize, numLines, numAnt);
                
                for i_an = 1:numAnt  
                   
                    % vectorized version
                    inputMat    = squeeze(input(:,:,i_an));
                    
                    % DC offset compensation
                    inputMat    = bsxfun(@minus, inputMat, mean(inputMat));
                    
                    % apply range-domain windowing
                    inputMat    = bsxfun(@times, inputMat, obj.rangeWindowCoeffVec);
                    
                    % Range FFT
                    fftOutput   = fft(inputMat, obj.rangeFFTSize);
                    
                    % apply Doppler windowing and scaling to the output. Doppler windowing moved to DopplerProc.m (5/24)
                    % fftOutput   = bsxfun(@times, fftOutput, obj.scaleFactorRange*obj.dopplerWindowCoeffVec.');
                    if  obj.FFTOutScaleOn == 1
                        fftOutput   = fftOutput * obj.scaleFactorRange;
                    end
                    
                    % populate in the data cube
                    out(:,:,i_an) = fftOutput;
                    
                end            
                
            else
                out     = input;
                
            end
            
        end
        
    end
    
end

