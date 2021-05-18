function Azimuth_Heatmap_Exp(adcData)

vxData = reshape(adcData, size(adcData, 1), size(adcData, 2), size(adcData, 3)*size(adcData, 4));
n_range_fft_size = size(adcData, 1);
n_Doppler_fft_size = size(adcData, 2);
n_angle_fft_size = 256;
range_resolution = 0.0488;
d = 1;
minRangeBinKeep = 5;
rightRangeBinDiscard = 20;
Azi_Idx = [1; 2; 3; 4; 17; 18; 19; 20; 33; 34; 35; 5; 6; 7; 8; 21; 22; 23; 24; 37; 38; 39; 40; 53; 54; 55; 56; 69; 70; 71; 72; 85; 86; 87; 88; 101; 102; 103; 104; 117; 118; 119; 120; 133; 134; 135; 9; 10; 11; 12; 13; 14; 15; 16; 29; 30; 31; 32; 45; 46; 47; 48; 61; 62; 63; 64; 77; 78; 79; 80; 93; 94; 95; 96; 109; 110; 111; 112; 125; 126; 127; 128; 141; 142; 143; 144];
vxData_Azi = vxData(:, :, Azi_Idx);
RangeFFT = fft(vxData_Azi, n_range_fft_size, 1);
DopplerFFT = fft(RangeFFT, n_Doppler_fft_size, 2);
AngleFFT = fft(DopplerFFT, n_angle_fft_size, 3);
% AngleFFT = squeeze(AngleFFT(:,2,:));
AngleFFT = squeeze(sum(AngleFFT(:,1:n_Doppler_fft_size,:),2));
AngleFFT = fftshift(AngleFFT,2);

vxData_Sum = sum(vxData(:,:,1:end),3);
vxData_Sum = sum(vxData_Sum(:,1:end),2);
figure()
subplot(211)
plot(([0 :n_range_fft_size- 1].*range_resolution)',abs(fft(vxData_Sum)));

indices_1D = (minRangeBinKeep:n_range_fft_size - rightRangeBinDiscard);
sine_theta = -2 * ((-n_angle_fft_size / 2:n_angle_fft_size / 2) / n_angle_fft_size) / d;
cos_theta = sqrt(1-sine_theta.^2);

[R_mat, sine_theta_mat] = meshgrid(indices_1D*range_resolution, sine_theta);
[~, cos_theta_mat] = meshgrid(indices_1D, cos_theta);

x_axis = R_mat .* cos_theta_mat;
y_axis = R_mat .* sine_theta_mat;
mag_data = squeeze(abs(AngleFFT(indices_1D + 1, [1:end, 1])));
mag_data = mag_data';
mag_data = flipud(mag_data);
subplot(212)
surf(y_axis, x_axis, 20*log10(mag_data), 'EdgeColor', 'none');
caxis([min(min(abs(20*log10(mag_data))))+50, max(max(abs(20*log10(mag_data))))]);
% caxis([100, 150]);
view(2);
xlabel('meters')
ylabel('meters')
colormap('jet')
set(gcf,'unit','centimeters','position',[1,2,25,40])
colorbar;

end