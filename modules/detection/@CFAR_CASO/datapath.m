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

%datapath.m
%
% datapath function of CFAR_CASO module. This module implements the CFAR cell average
% algorithm, but the noise is estimated as the smaller average noise levels
% estimated from the left and right noise bins. The detection is done in
% two steps. Step1: CFAR_CASO_Range along each range lines; Step2: at the
% detected range bins, do CFAR along the Doppler direction. The maximum
% velocity is extened due to TDM MIMO. The phase is also corrected after
% determine the extended velocity, so that the angle estimation can be
% performed correctly.

%input
%   obj: object instance of CFAR_CASO
%   input: a 3D matrix, range x Doppler x antenna array


%output
%   detection_results: detecion results
%


function [detection_results] = datapath(obj, input)


% non-coherent signal combination along the antenna array
sig_integrate = sum((abs(input)).^2,3) + 1;

angleFFTSize = 128;
angleBinSkipLeft = 4;
angleBinSkipRight = 4;


if (obj.detectMethod == 1) % Cell dual-pass CASO-CFAR
    [N_obj_Rag, Ind_obj_Rag, noise_obj, CFAR_SNR] = CFAR_CASO_Range(obj, sig_integrate);
    
    N_obj = 0;
    Ind_obj = [];
    detection_results = {};
    if (N_obj_Rag>0)
        [N_obj, Ind_obj] = CFAR_CASO_Doppler_overlap(obj, Ind_obj_Rag, input, sig_integrate);
        detection_results = [];
        
        % Use aggregate noise estimation from the first pass and apply
        % it to objects confirmed in the second pass
        noise_obj_agg = [];
        for i_obj = 1:N_obj
            indx1R = Ind_obj(i_obj,1);
            indx1D = Ind_obj(i_obj,2);
            ind2R = find(Ind_obj_Rag(:,1) == indx1R);
            ind2D = find(Ind_obj_Rag(ind2R,2) == indx1D);
            noiseInd = ind2R(ind2D);
            noise_obj_agg(i_obj) = noise_obj(noiseInd);
        end
        
        for i_obj = 1:N_obj
            xind = (Ind_obj(i_obj,1)-1) +1;
            detection_results(i_obj).rangeInd = Ind_obj(i_obj, 1) - 1;  %range index
            detection_results(i_obj).range = (detection_results(i_obj).rangeInd) * obj.rangeBinSize;  %range estimation
            dopplerInd  = Ind_obj(i_obj, 2) - 1;  %Doppler index
            detection_results(i_obj).dopplerInd_org = dopplerInd;
            detection_results(i_obj).dopplerInd = dopplerInd;
            
            %velocity estimation
            detection_results(i_obj).doppler = (dopplerInd-obj.dopplerFFTSize/2)*obj.velocityBinSize;
            detection_results(i_obj).doppler_corr = detection_results (i_obj).doppler;
            detection_results(i_obj).noise_var = noise_obj_agg(i_obj);       %noise variance
            detection_results(i_obj).bin_val  = reshape(input(xind, Ind_obj(i_obj,2),:),obj.numAntenna,1);  %2d FFT value for the 4 antennas
            %detection_results(i_obj).estSNR  = 10*log10(sum(abs(detection_results (i_obj).bin_val).^2)/sum(detection_results (i_obj).noise_var));  %2d FFT value for the 4 antennas
            detection_results(i_obj).estSNR  = (sum(abs(detection_results (i_obj).bin_val).^2)/sum(detection_results (i_obj).noise_var));  
            
            sig_bin = [];
            %only apply max velocity extention if it is enabled and distance is larger
            %than minDisApplyVmaxExtend
            if (obj.applyVmaxExtend == 1 && (detection_results(i_obj).range > obj.minDisApplyVmaxExtend) && (~isempty(obj.overlapAntenna_ID)))
                velocityObj_est = detection_results(i_obj).doppler;
                if mod(obj.TDM_MIMO_numTX,2)==1
                    %odd number
                    dopplerInd_unwrap = dopplerInd + ((1:obj.TDM_MIMO_numTX)-ceil(obj.TDM_MIMO_numTX/2))*obj.dopplerFFTSize;
                    
                else
                    %even number
                    if velocityObj_est>0
                        dopplerInd_unwrap = dopplerInd + ((1:obj.TDM_MIMO_numTX)-(obj.TDM_MIMO_numTX/2+1))*obj.dopplerFFTSize;
                        
                    else
                        dopplerInd_unwrap = dopplerInd + ((1:obj.TDM_MIMO_numTX)-obj.TDM_MIMO_numTX/2)*obj.dopplerFFTSize;
                        
                    end
                end
                sig_bin_org = detection_results (i_obj).bin_val;                
                %Doppler phase correction due to TDM MIMO             
                deltaPhi = 2*pi*(dopplerInd_unwrap-obj.dopplerFFTSize/2)/( obj.TDM_MIMO_numTX*obj.dopplerFFTSize);
               
                % construct all possible signal vectors based on the number
                % of possible hypothesis
                for i_TX = 1:obj.TDM_MIMO_numTX
                    RX_ID = (i_TX-1)*obj.numRxAnt+1 : i_TX*obj.numRxAnt;
                    sig_bin(RX_ID,: )= sig_bin_org(RX_ID )* exp(-1j*(i_TX-1)*deltaPhi);
                end
                
                % use overlap antenna to do max velocity unwrap
                overlapAntenna_ID = obj.overlapAntenna_ID;                              
                signal_overlap = sig_bin_org(overlapAntenna_ID(:,1:2));                
                
                %check the phase difference of each overlap antenna pair
                %for each hypothesis
                angle_sum_test = [];                
                for i_sig = 1:size(signal_overlap,1)
                    for i_test = 1:length(deltaPhi)
                        signal2 = signal_overlap(1:i_sig,2)*exp(-j*deltaPhi(i_test));
                        angle_sum_test(i_sig,i_test) = angle(sum(signal_overlap(1:i_sig,1).*conj(signal2)));
                        
                    end
                end
                
                %chosee the hypothesis with minimum phase difference to
                %estimate the unwrap factor
                [val_doppler_unwrap_integ_overlap doppler_unwrap_integ_overlap] = min(abs(angle_sum_test),[],2);
                             
                
                %test the angle FFT SNR
                sig_bin_row1 = sig_bin(obj.antenna_azimuthonly,:);
                sig_bin_row1_fft = fftshift(fft(sig_bin_row1,angleFFTSize),1);
                sig_bin_row1_fft_cut = abs(sig_bin_row1_fft(angleBinSkipLeft+1:(angleFFTSize-angleBinSkipRight),:));
                [val doppler_unwrap_integ_FFT] = max(max(sig_bin_row1_fft_cut));
                
                              
                b = unique(doppler_unwrap_integ_overlap);
                c = histc(doppler_unwrap_integ_overlap(:),b);
                [val ind] = max(c);
                doppler_unwrap_integ_overlap_sel = b(ind);
                doppler_unwrap_integ = doppler_unwrap_integ_overlap_sel;           
                
                
                %overlap antenna method is applied by default
                detection_results(i_obj).bin_val = sig_bin(:,doppler_unwrap_integ);                  
               
                %corret velocity after applying the integer value
                dopplerInd = dopplerInd_unwrap(doppler_unwrap_integ);
                dopplerInd_FFT = dopplerInd_unwrap(doppler_unwrap_integ_FFT);
                dopplerInd_overlap = dopplerInd_unwrap(doppler_unwrap_integ_overlap_sel);
                detection_results (i_obj).dopplerInd = dopplerInd;
                %velocity estimation
                detection_results (i_obj).doppler_corr = (dopplerInd-obj.dopplerFFTSize/2)*obj.velocityBinSize;                
                %both overlap antenna and FFT results are reported 
                detection_results(i_obj).doppler_corr_overlap = (dopplerInd_overlap-obj.dopplerFFTSize/2)*obj.velocityBinSize;
                detection_results(i_obj).doppler_corr_FFT = (dopplerInd_FFT-obj.dopplerFFTSize/2)*obj.velocityBinSize;
                detection_results(i_obj).overlapTests = doppler_unwrap_integ_overlap;
                detection_results(i_obj).overlapTestsVal = val_doppler_unwrap_integ_overlap;
            else
                %Doppler phase correction due to TDM MIMO without apply
                %Vmax extention algorithm
                
                deltaPhi = 2*pi*(dopplerInd-obj.dopplerFFTSize/2)/( obj.TDM_MIMO_numTX*obj.dopplerFFTSize);
                sig_bin_org = detection_results (i_obj).bin_val;
                for i_TX = 1:obj.TDM_MIMO_numTX
                    RX_ID = (i_TX-1)*obj.numRxAnt+1 : i_TX*obj.numRxAnt;
                    sig_bin(RX_ID,: )= sig_bin_org(RX_ID )* exp(-1j*(i_TX-1)*deltaPhi);
                end
                detection_results(i_obj).bin_val = sig_bin;
                detection_results(i_obj).doppler_corr_overlap = detection_results(i_obj).doppler_corr;
                detection_results(i_obj).doppler_corr_FFT = detection_results(i_obj).doppler_corr;
                
            end
            
        end
    end
else
    disp('Not supported detction method!');
    
end




