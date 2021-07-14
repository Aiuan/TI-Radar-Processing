clear all;clc;

numADCSample = 9.000000e+01; 
adcSampleRate = 2.500000e+07; %Hz/s 
startFreqConst = 7.700000e+10; %Hz 
chirpSlope = 4.688000e+13; %Hz/s 
chirpIdleTime = 2.000000e-06; %s 
adcStartTimeConst = 3.630000e-06; %s 
chirpRampEndTime = 7.440000e-06; %s 
framePeriodicty = 1.000000e-01; 
NumDevices = 4; 
framePeriodicty = 1.000000e-01; 
frameCount = 3.300000e+03; %s 
numChirpsInLoop = 1.200000e+01; %s 
nchirp_loops = 128; 
numTxAnt = 12; 
TxToEnable = [12  11  10   9   8   7   6   5   4   3   2   1];
speedOfLight = 3e8;

Rmax = adcSampleRate * speedOfLight / 2 / chirpSlope;
freqCenter = startFreqConst + (adcStartTimeConst + numADCSample / adcSampleRate / 2) * chirpSlope;
lambda = speedOfLight / freqCenter;
Tc = chirpIdleTime + chirpRampEndTime;
Vmax = lambda / 4 / size(TxToEnable, 2) / Tc;


load('./analyseData/20210428mode3Group1_0002_520.mat');
fprintf('adcData size: (%d, %d, %d, %d)\n', size(adcData,1), size(adcData,2), size(adcData,3), size(adcData,4));

% adcData 存储顺序: 
% 第4维度：按时间先后顺序(发射顺序)，TXID：[12  11  10   9   8   7   6   5   4   3   2   1]
% 第3维度：同时接收，RXID：[1  2  3  4  5  6  7  8  9  10  11  12  13  14  15  16]

rangeFFT =  fft(adcData, 128, 1);
fprintf('rangeFFT size: (%d, %d, %d, %d)\n', size(rangeFFT,1), size(rangeFFT,2), size(rangeFFT,3), size(rangeFFT,4));

dopplerFFT = fftshift(fft(rangeFFT, 128, 2),2);
fprintf('dopplerFFT size: (%d, %d, %d, %d)\n', size(dopplerFFT,1), size(dopplerFFT,2), size(dopplerFFT,3), size(dopplerFFT,4));

dopplerFFT_virtual = reshape(dopplerFFT, size(dopplerFFT,1), size(dopplerFFT,2), size(dopplerFFT,3)*size(dopplerFFT,4));
fprintf('dopplerFFT_virtual size: (%d, %d, %d)\n', size(dopplerFFT_virtual,1), size(dopplerFFT_virtual,2), size(dopplerFFT_virtual,3));

sig_integrate = 10*log10(sum((abs(dopplerFFT_virtual)).^2,3) + 1);


figure();
rangeList = (1:size(sig_integrate,1))*Rmax/size(sig_integrate,1);
velocityList = (-size(sig_integrate,2)/2: size(sig_integrate,2)/2-1) * 2*Vmax/size(sig_integrate,2);
imagesc(velocityList, rangeList, sig_integrate);
colormap(jet);
