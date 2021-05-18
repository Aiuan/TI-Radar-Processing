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

%DOACascade.m
%
% DOACascade module definition. This module estimates the azimuth and
% elevation (if enabled) angles using FFT based beamforming method. Muitple
% peaks are detected in the azimuth direction, only maximum peak is
% detected in the elevation direction.

%% Class definition

classdef DOACascade < Module
    %% properties
    properties (Access = public)       
        numAntenna = 0
        antPos = [];
        antDis = 0
        method = 0
        standardWidth_gamma = 0    
        dopplerFFTSize = 0     
        angles_DOA_az = []
        angles_DOA_ele = []
        gamma = 0;
        sidelobeLevel_dB_azim = 0
        sidelobeLevel_dB_elev = 0
        sidelobeLevel_dB = 0;
        D = []
        DOAFFTSize = 0        
        
    end
    
    methods
        %% constructor
        function obj = DOACascade(varargin)
            if(isempty(find(strcmp(varargin,'name'), 1)))
                varargin = [varargin, 'name','DOACascade'];
            end
            obj@Module(varargin{:});            
           
            obj.enable = getParameter(obj, 'enable');               
            obj.numAntenna = getParameter(obj, 'numAntenna');            
            obj.antPos = getParameter(obj, 'antPos');               
            obj.antDis = getParameter(obj, 'antDis');              
            obj.method = getParameter(obj, 'method');              
            obj.angles_DOA_az = getParameter(obj, 'angles_DOA_az');              
            obj.angles_DOA_ele = getParameter(obj, 'angles_DOA_ele');               
            obj.gamma = getParameter(obj, 'gamma');              
            obj.sidelobeLevel_dB_azim = getParameter(obj, 'sidelobeLevel_dB_azim');            
            obj.sidelobeLevel_dB_elev = getParameter(obj, 'sidelobeLevel_dB_elev');              
            obj.DOAFFTSize = getParameter(obj, 'DOAFFTSize');              
            obj.D = getParameter(obj, 'D');   
             
           
            try                
                obj.dopplerFFTSize = getParameter(obj, 'dopplerFFTSize');                 
            end
            
            % overwritten the property value inside parameter file
            %setProperties(obj, nargin, varargin{:});
            obj = set(obj, varargin{:});           
        end
                
        %% datapath function
        function out = datapath(obj, detected_obj)
            numObj = length(detected_obj);
            out = detected_obj;
            numAoAObjCnt = 0;
            % extended detection_obj to include the angles information
            for i_obj = 1:numObj
                current_obj = detected_obj(i_obj);
                estSNR = 10*log10(sum(abs(current_obj.bin_val).^2)/sum(current_obj.noise_var));
                X = current_obj.bin_val; 
                R = X*X';
                switch obj.method
                    
                    case 1
                        %2D beamforming angle estimation, azimuth is estimated based on 1D FFT output                        
                        [DOA_angles angle_sepc_2D_fft ]= DOA_beamformingFFT_2D(obj, X);
                        if (numAoAObjCnt == 0)
                            out = [];
                        end
                        
                        for i_obj = 1:size(DOA_angles,2)
                            numAoAObjCnt = numAoAObjCnt+1;
                            out(numAoAObjCnt).rangeInd = current_obj.rangeInd;
                            out(numAoAObjCnt).dopplerInd = current_obj.dopplerInd;
                            out(numAoAObjCnt).range = current_obj.range;
                            out(numAoAObjCnt).doppler_corr = current_obj.doppler_corr;
                            out(numAoAObjCnt).dopplerInd_org = current_obj.dopplerInd_org;

                            out(numAoAObjCnt).noise_var = current_obj.noise_var;
                            out(numAoAObjCnt).bin_val = current_obj.bin_val;
                            out(numAoAObjCnt).estSNR = current_obj.estSNR;
                            out(numAoAObjCnt).doppler_corr_overlap = current_obj.doppler_corr_overlap;
                            out(numAoAObjCnt).doppler_corr_FFT = current_obj.doppler_corr_FFT;
                            
                            out(numAoAObjCnt).angles = DOA_angles(:,i_obj);
                            out(numAoAObjCnt).spectrum = angle_sepc_2D_fft;
                           
                            
                        end
                    case 2
                        %2D beamforming, angle estimated after 2D FFT jointly
                        [DOA_angles angle_sepc_2D_fft ]= DOA_beamformingFFT_2D_joint(obj, X);
                        if (numAoAObjCnt == 0)
                            out = [];
                        end
                        
                         for i_obj = 1:size(DOA_angles,2)
                            numAoAObjCnt = numAoAObjCnt+1;
                            out(numAoAObjCnt).rangeInd = current_obj.rangeInd;
                            out(numAoAObjCnt).dopplerInd = current_obj.dopplerInd;
                            out(numAoAObjCnt).range = current_obj.range;
                            out(numAoAObjCnt).doppler_corr = current_obj.doppler_corr;
                            out(numAoAObjCnt).dopplerInd_org = current_obj.dopplerInd_org;

                            out(numAoAObjCnt).noise_var = current_obj.noise_var;
                            out(numAoAObjCnt).bin_val = current_obj.bin_val;
                            out(numAoAObjCnt).estSNR = current_obj.estSNR;
                            out(numAoAObjCnt).doppler_corr_overlap = current_obj.doppler_corr_overlap;
                            out(numAoAObjCnt).doppler_corr_FFT = current_obj.doppler_corr_FFT;
                            
                            out(numAoAObjCnt).angles = DOA_angles(:,i_obj);
                            out(numAoAObjCnt).spectrum = angle_sepc_2D_fft;
                           
                            
                        end
                        
                    otherwise
                        error('Not specified DOA method')
                        
                end
            end
        end
        
    end
end

