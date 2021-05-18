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

% plot_range_azimuth_2D.m
%
% Function to plot range and azimuth heat map

%input
%   range_resolution: range resolution to calculate axis to plot
%   radar_data_pre_3dfft: input 3D matrix, rangeFFT x DopplerFFT x virtualArray
%   TDM_MIMO_numTX: number of TXs used for processing
%   numRxAnt: : number of RXs used for processing
%   antenna_azimuthonly: azimuth array ID
%   LOG: 1:plot non-linear scale, ^0.4 by default
%   STATIC_ONLY: 1 = plot heatmap for zero-Doppler; 0 = plot heatmap for nonzero-Doppler
%   PLOT_ON: 1 = plot on; 0 = plot off
%   minRangeBinKeep: start range index to keep
%   rightRangeBinDiscard: number of right most range bins to discard

%output
%   mag_data_static: zero Doppler range/azimuth heatmap
%   mag_data_dynamic: non-zero Doppler range/azimuth heatmap
%   y_axis: y axis used for visualization
%   x_axis: x axis used for visualization



function  [mag_data_static mag_data_dynamic y_axis x_axis] = plot_range_azimuth_2D(range_resolution, radar_data_pre_3dfft,TDM_MIMO_numTX,numRxAnt,...
    antenna_azimuthonly, LOG, STATIC_ONLY, PLOT_ON, minRangeBinKeep,  rightRangeBinDiscard)

dopplerFFTSize = size(radar_data_pre_3dfft,2);
rangeFFTSize = size(radar_data_pre_3dfft,1);
angleFFTSize = 256;
% ratio used to decide engergy threshold used to pick non-zero Doppler bins
ratio = 0.5;
DopplerCorrection = 0;

if DopplerCorrection == 1
    % add Doppler correction before generating the heatmap
    radar_data_pre_3dfft_DopCor= [];
    for dopplerInd = 1: dopplerFFTSize
        deltaPhi = 2*pi*(dopplerInd-1-dopplerFFTSize/2)/( TDM_MIMO_numTX*dopplerFFTSize);
        sig_bin_org =squeeze(radar_data_pre_3dfft(:,dopplerInd,:));
        for i_TX = 1:TDM_MIMO_numTX
            RX_ID = (i_TX-1)*numRxAnt+1 : i_TX*numRxAnt;
            corVec = repmat(exp(-1j*(i_TX-1)*deltaPhi), rangeFFTSize, numRxAnt);
            radar_data_pre_3dfft_DopCor(:,dopplerInd, RX_ID)= sig_bin_org(:,RX_ID ).* corVec;
        end
    end
    
    radar_data_pre_3dfft = radar_data_pre_3dfft_DopCor;
end
radar_data_pre_3dfft = radar_data_pre_3dfft(:,:,antenna_azimuthonly);

radar_data_angle_range = fft(radar_data_pre_3dfft, angleFFTSize, 3);
n_angle_fft_size = size(radar_data_angle_range,3);
n_range_fft_size = size(radar_data_angle_range,1);


%decide non-zerp doppler bins to be used for dynamic range-azimuth heatmap
DopplerPower = sum(mean((abs(radar_data_pre_3dfft(:,:,:))),3),1);
DopplerPower_noDC = DopplerPower([1: dopplerFFTSize/2-1 dopplerFFTSize/2+3:end]);
[peakVal peakInd] = max(DopplerPower_noDC);
threshold = peakVal*ratio;
indSel = find(DopplerPower_noDC >threshold);
for ii = 1:length(indSel)
    if indSel(ii) > dopplerFFTSize/2-1
        indSel(ii) = indSel(ii) + 3;
    end
end

radar_data_angle_range_dynamic = squeeze(sum(abs(radar_data_angle_range(:,indSel,:)),2));
radar_data_angle_range_Static = squeeze(sum(abs(radar_data_angle_range(:,dopplerFFTSize/2+1,:)),2));


indices_1D = (minRangeBinKeep:n_range_fft_size-rightRangeBinDiscard);
max_range = (n_range_fft_size-1)*range_resolution;
max_range = max_range/2;
d = 1;

%generate range/angleFFT for zeroDoppler and non-zero Doppler respectively
radar_data_angle_range_dynamic = fftshift(radar_data_angle_range_dynamic,2);
radar_data_angle_range_Static = fftshift(radar_data_angle_range_Static,2);


sine_theta = -2*((-n_angle_fft_size/2:n_angle_fft_size/2)/n_angle_fft_size)/d;
cos_theta = sqrt(1-sine_theta.^2);

[R_mat, sine_theta_mat] = meshgrid(indices_1D*range_resolution,sine_theta);
[~, cos_theta_mat] = meshgrid(indices_1D,cos_theta);

x_axis = R_mat.*cos_theta_mat;
y_axis = R_mat.*sine_theta_mat;
mag_data_dynamic = squeeze(abs(radar_data_angle_range_dynamic(indices_1D+1,[1:end 1])));
mag_data_static = squeeze(abs(radar_data_angle_range_Static(indices_1D+1,[1:end 1])));


mag_data_dynamic = mag_data_dynamic';
mag_data_static = mag_data_static';
mag_data_dynamic = flipud(mag_data_dynamic);
mag_data_static = flipud(mag_data_static);


if PLOT_ON
    log_plot = LOG;
    if STATIC_ONLY == 1
        if log_plot
            surf(y_axis, x_axis, (mag_data_static).^0.1,'EdgeColor','none');
            caxis([min(min(abs((mag_data_static).^0.1))), max(max(abs((mag_data_static).^0.1)))]);
            %caxis([max(max(abs((mag_data_static).^0.1)))*0.5, max(max(abs((mag_data_static).^0.1)))]);
        else
            surf(y_axis, x_axis, abs(mag_data_static),'EdgeColor','none');
        end
    else
        if log_plot
            surf(y_axis, x_axis, (mag_data_dynamic).^0.1,'EdgeColor','none');
        else
            surf(y_axis, x_axis, abs(mag_data_dynamic),'EdgeColor','none');
        end
    end
    
    view(2);
    xlabel('meters')
    ylabel('meters')
    colormap('jet')
    
    
end
end