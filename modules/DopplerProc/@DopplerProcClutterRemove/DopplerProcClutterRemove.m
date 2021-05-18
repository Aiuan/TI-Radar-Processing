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

%DopplerProcClutterRemove.m
%
%DopplerProcClutterRemove module definition. Perform Doppler FFT with
%clutter removal as an option


%% Class definition
classdef DopplerProcClutterRemove < Module
    %% properties
    properties (Access = 'public')
        
        numAntenna = 0           % number of antennas
        numDopplerLines = 0        % number of Doppler lines
        dopplerFFTSize  = 0        % Doppler FFT size
        numChirpsPerVirAnt = 0       
        dopplerWindowEnable  = 0   % flag to enable or disable windowing before doppler FFT
        dopplerWindowCoeff  = []    % Doppler FFT window coefficients (one side)
        dopplerWindowCoeffVec = []  % Doppler FFT window coefficients of length dopplerFFTSize
        scaleFactorDoppler = 0       
        clutterRemove = 0
        FFTOutScaleOn = 0
        
    end
    
    methods
        %% constructor
        function obj = DopplerProcClutterRemove(varargin)
            if(isempty(find(strcmp(varargin,'name'), 1)))
                varargin = [varargin, 'name','DopplerProcClutterRemove'];
            end
            obj@Module(varargin{:});
            
            obj.enable = getParameter(obj, 'enable');            
            obj.numAntenna = getParameter(obj, 'numAntenna');          
            obj.numDopplerLines = getParameter(obj, 'numDopplerLines');           
            obj.dopplerFFTSize = getParameter(obj, 'dopplerFFTSize');           
            obj.numChirpsPerVirAnt = getParameter(obj, 'numChirpsPerVirAnt');           
            obj.dopplerWindowEnable = getParameter(obj, 'dopplerWindowEnable');          
            obj.dopplerWindowCoeff = getParameter(obj, 'dopplerWindowCoeff');          
            obj.scaleFactorDoppler = getParameter(obj, 'scaleFactorDoppler');         
            obj.clutterRemove = getParameter(obj, 'clutterRemove');           
            obj.FFTOutScaleOn = getParameter(obj, 'FFTOutScaleOn');
          
           
            
            % form the Doppler window coefficients vector
            % set all coefficients to 1 if Doppler windowing is disabled
            if ~obj.dopplerWindowEnable
                obj.dopplerWindowCoeff = ones(length(obj.dopplerWindowCoeff), 1);
            end
            
            dopplerWinLen               = length(obj.dopplerWindowCoeff);
            dopplerWindowCoeffVec       = ones(obj.numChirpsPerVirAnt, 1);
            dopplerWindowCoeffVec(1:dopplerWinLen) = obj.dopplerWindowCoeff;
            dopplerWindowCoeffVec(obj.numChirpsPerVirAnt-dopplerWinLen+1:obj.numChirpsPerVirAnt) = dopplerWindowCoeffVec(dopplerWinLen:-1:1);
            obj.dopplerWindowCoeffVec   = dopplerWindowCoeffVec;            
           
            % overwritten the property value inside parameter file
            %setProperties(obj, nargin, varargin{:});
            obj       = set(obj, varargin{:});
            
        end
        
        % datapath function
        % input: adc data, assuming size(input) = [numSamplePerChipr numChirpsPerFrame numAntenna]
        function [out] = datapath(obj, input)
            
            numLines  = size(input,1);
            numAnt    = size(input,3);
            
            if obj.enable                  
                
                % initialize
                out = zeros(numLines, obj.dopplerFFTSize, numAnt);
                
                for i_an = 1:numAnt                  
                   
                    %% vectorized version
                    inputMat    = squeeze(input(:,:,i_an));
                    inputMat    = bsxfun(@times, inputMat, obj.dopplerWindowCoeffVec.');
                    if obj.clutterRemove ==1
                        inputMat = inputMat - (repmat(mean(inputMat'),size(inputMat,2),1))';
                    end
                    fftOutput   = fft(inputMat, obj.dopplerFFTSize, 2);                         
                    
                    if obj.FFTOutScaleOn ==1
                        fftOutput   = fftshift(fftOutput, 2) * obj.scaleFactorDoppler;
                    else
                        fftOutput   = fftshift(fftOutput, 2);
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
