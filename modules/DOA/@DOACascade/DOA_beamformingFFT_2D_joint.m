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

%DOA_beamformingFFT_2D_joint.m
%
%DOA_beamformingFFT_2D_joint function perform 2D angle estimation based on FFT beamforming, both azimuth/elevation peak selection
%is done in 2D FFT domain


%input:
%   obj: object instance
%   sig: complex signal vector, with each value corresponding to each
%   antenna. The length of this vector equals to numTX x numRX enabled.
%   There can be overlapped antennas. this signal needs to be re-arranged
%   based on D value to form the virtual antenna array

%output:
%   angleObj_est: angle estimation results
%   angle_sepc_2D_fft: angle 2D fft spectrum



function [angleObj_est angle_sepc_2D_fft ]= DOA_beamformingFFT_2D_joint(obj, sig)


%field of view to do beamforming
angles_DOA_az = obj.angles_DOA_az;
angles_DOA_ele = obj.angles_DOA_ele;

%distance unit in terms of wavelength
d = obj.antDis;
%2D matrix providing antenna coordinates
D = obj.D;
angleFFTSize = obj.DOAFFTSize;

%FFT based implementation
%first form a 2D matrix based on the antenna coordinates
D = D + 1;
apertureLen_azim = max(D(:,1));
apertureLen_elev = max(D(:,2));
sig_2D = zeros(apertureLen_azim,apertureLen_elev);
for i_line = 1:apertureLen_elev
    ind = find(D(:,2) == i_line);
    D_sel = D(ind,1);
    sig_sel = sig(ind);
    [val indU] = unique(D_sel);
    
    sig_2D(D_sel(indU),i_line) = sig_sel(indU);
    
end

%run FFT on azimuth and elevation
angle_sepc_1D_fft=fftshift(fft(sig_2D,angleFFTSize,1),1); %corr1 is simply a 2D-fft of s1
angle_sepc_2D_fft=fftshift(fft(angle_sepc_1D_fft,angleFFTSize,2),2); %corr1 is simply a 2D-fft of s1

wx_vec=[-pi:2*pi/angleFFTSize:pi];
wz_vec=[-pi:2*pi/angleFFTSize:pi];
wx_vec = wx_vec(1:end-1);
wz_vec = wz_vec(1:end-1);
%convert to degree
elev_vec = asind(wz_vec/(2*pi*d));

angleObj_est= [];
cnt = 1;
if apertureLen_elev ==1
    %azimuth array only, no elevation antennas
    spec_azim = abs(angle_sepc_1D_fft(:,1));
    obj.sidelobeLevel_dB = obj.sidelobeLevel_dB_azim;
    [peakVal_azim, peakLoc_azim] = DOA_BF_PeakDet_loc(obj, spec_azim);
   
    for i_obj = 1:length(peakLoc_azim)
        ind = peakLoc_azim(i_obj);
        
        azim_est = asind(wx_vec(ind)/(2*pi*d));
        if (azim_est >= angles_DOA_az(1) && azim_est <= angles_DOA_az(2))
            angleObj_est(1,cnt) = azim_est;
            angleObj_est(2,cnt) = 0;
            angleObj_est(3,cnt) = ind;
            angleObj_est(4,cnt) = 0;
            cnt = cnt+1;
            
        else
            continue;
        end
    end
    
else
    %for each azimuth angle, find the max value in elevation direction, only
    %single target detection in elevation direction
    [spec_azim peak_ele_ind ]= max(abs(angle_sepc_2D_fft'));
    
    %detect multiple peaks in azimuth direction
    obj.sidelobeLevel_dB = obj.sidelobeLevel_dB_azim;
    [peakVal_azim, peakLoc_azim] = DOA_BF_PeakDet_loc(obj, spec_azim);
    
    
    for i_obj = 1:length(peakVal_azim)
        
        elev_est = elev_vec(peak_ele_ind(peakLoc_azim(i_obj)));
        azim_sind = (wx_vec(peakLoc_azim(i_obj))/(2*pi*d))/cosd(elev_est);
        if (abs(azim_sind)<1)
            azim_est = asind(azim_sind);
            if ( azim_est>= angles_DOA_az(1) && azim_est <= angles_DOA_az(2) ...
                    &&elev_est >= angles_DOA_ele(1) && elev_est <= angles_DOA_ele(2))
                
                angleObj_est(1,cnt) = azim_est;
                angleObj_est(2,cnt) = elev_est;
                angleObj_est(3,cnt) = peakLoc_azim(i_obj);
                angleObj_est(4,cnt) = peak_ele_ind(peakLoc_azim(i_obj));
                %plot(angleObj_est(4,cnt),angleObj_est(3,cnt) ,'x','MarkerSize',12, 'LineWidth',2);
                %hold on
                cnt = cnt+1;
            end        
        end
    end
    
end

end

%hold off





