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

%CFAR_CASO.m
%
% CFAR_CASO module definition. This module implements the CFAR cell average
% algorithm, but the noise is estimated as the smaller average noise levels
% estimated from the left and right noise bins.



%% Class definition

classdef CFAR_CASO < Module
    %% properties
    properties (Access = public)
        detectMethod = 0% detection method choose to run
        numAntenna = 0 %number of antennas
        refWinSize = 0 %number of reference cells to estimate noise variance
        guardWinSize = 0 %number of gap cells to prevent leakage being detected as signal
        K0 = 0%detection threshold        
        maxEnable = 0  
        rangeBinSize = 0
        velocityBinSize = 0
        dopplerFFTSize = 0
        powerThre = 0
        discardCellLeft = 0
        discardCellRight = 0
        numRxAnt = 0         
        TDM_MIMO_numTX = 0
        antenna_azimuthonly = 0
        overlapAntenna_ID = 0
        overlapAntenna_ID_2TX = 0
        overlapAntenna_ID_3TX = 0
        applyVmaxExtend = 0
        minDisApplyVmaxExtend = 0
    end
    
    methods
        %% constructor
        function obj = CFAR_CASO(varargin)
            if(isempty(find(strcmp(varargin,'name'), 1)))
               varargin = [varargin, 'name','CFAR_CASO']; 
            end 
            obj@Module(varargin{:});
           
            obj.enable = getParameter(obj, 'enable');           
            obj.detectMethod = getParameter(obj, 'detectMethod');           
            obj.numAntenna = getParameter(obj, 'numAntenna');            
            obj.refWinSize = getParameter(obj, 'refWinSize');            
            obj.guardWinSize = getParameter(obj, 'guardWinSize');            
            obj.K0 = getParameter(obj, 'K0');              
            obj.maxEnable = getParameter(obj, 'maxEnable');            
            obj.rangeBinSize = getParameter(obj, 'rangeBinSize');            
            obj.velocityBinSize = getParameter(obj, 'velocityBinSize');            
            obj.dopplerFFTSize = getParameter(obj, 'dopplerFFTSize');            
            obj.powerThre = getParameter(obj, 'powerThre');            
            obj.discardCellLeft = getParameter(obj, 'discardCellLeft');              
            obj.discardCellRight = getParameter(obj, 'discardCellRight');           
            obj.numRxAnt = getParameter(obj, 'numRxAnt');           
            obj.TDM_MIMO_numTX = getParameter(obj, 'TDM_MIMO_numTX');           
            obj.antenna_azimuthonly = getParameter(obj, 'antenna_azimuthonly');           
            obj.overlapAntenna_ID = getParameter(obj, 'overlapAntenna_ID');            
            obj.overlapAntenna_ID_2TX = getParameter(obj, 'overlapAntenna_ID_2TX');            
            obj.overlapAntenna_ID_3TX = getParameter(obj, 'overlapAntenna_ID_3TX');            
            obj.applyVmaxExtend = getParameter(obj, 'applyVmaxExtend');     
            obj.minDisApplyVmaxExtend = getParameter(obj, 'minDisApplyVmaxExtend');     
           

            % overwritten the property value inside parameter file
            %setProperties(obj, nargin, varargin{:});
            obj = set(obj, varargin{:});

        end
        % datapath function
        %% input: adc data, assuming size(input) = [numSamplePerChipr numChirpsPerFrame numAntenna]
        [detection_results N_obj Ind_obj]  = datapath(obj, input)
    end
    methods (Access = protected)
        %% input: adc data, assuming size(input) = [numSamplePerChipr numChirpsPerFrame numAntenna]
        % the list of other class methods
        [N_obj, Ind_obj, noise_obj, noise_obj_an] = CFAR_CASO_Doppler_overlap(obj, Ind_obj_Rag, sigCpml, sig_integ)
        [N_obj, Ind_obj, noise_obj, noise_obj_an] = CFAR_CASO_Range(obj, sigCpml, sig)
    end    
end

