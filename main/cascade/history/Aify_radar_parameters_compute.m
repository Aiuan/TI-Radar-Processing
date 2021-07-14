%author aifuyuan 
%计算雷达相关性能指标

%% dataset mode1 
clear all;
% % clc;

disp('===========================================')
Fs = 25e6;%Hz
c = 3e8;%m/s
S = 46.880*1e6/1e-6;%Hz/s
Rmax = Fs*c/2/S;%m

fprintf('Fs = %d Mkps, S = %f MHz/us\n',Fs/1e6, S/1e12);

adc_samples = 90;
BW = adc_samples/Fs*S;%Hz
Rres = c/2/BW;%m


idle_time = 2e-6;%s
adc_start_time = 3.63e-6;%s
ramp_end_time = 7.44e-6;%s
BW_use = ramp_end_time * S;
fprintf('adc_samples = %d, BW = %f MHz, BW_use = %f MHz\n',adc_samples,BW/1e6, BW_use/1e6);
fprintf('Rmax = %d m, Rres = %f m\n',Rmax, Rres);

numTxAnt = 4;
fprintf('numTxAnt = %d\n',numTxAnt);
Tc = (idle_time + ramp_end_time)*numTxAnt;%s
f_start = 77e9;%Hz
f_center = f_start+(idle_time+adc_start_time+adc_samples/Fs/2)*S;
lambda = c/f_center;
Vmax = lambda/4/Tc;
num_chirps = 128;
Tf = num_chirps*Tc;
Vres = lambda/2/Tf;
fprintf('Vmax = %f m/s, Vres = %f m/s\n', Vmax, Vres);
fprintf('Vmax = %f km/h\n', Vmax*3.6);

fps_max = 1/(Tf);
fprintf('fps_max = %f\n',fps_max);

%% dataset mode2
clear all;
% % clc;

disp('===========================================')
Fs = 22.5e6;%Hz
c = 3e8;%m/s
S = 43.017*1e6/1e-6;%Hz/s
Rmax = Fs*c/2/S;%m

fprintf('Fs = %d Mkps, S = %f MHz/us, Rmax = %d m\n',Fs/1e6, S/1e12, Rmax);

adc_samples = 90;
BW = adc_samples/Fs*S;%Hz
Rres = c/2/BW;%m


idle_time = 2e-6;%s
adc_start_time = 3.64e-6;%s
ramp_end_time = 7.86e-6;%s
BW_use = ramp_end_time * S;
fprintf('adc_samples = %d, BW = %f MHz, BW_use = %f MHz\n',adc_samples,BW/1e6, BW_use/1e6);
fprintf('Rmax = %d m, Rres = %f m\n',Rmax, Rres);

numTxAnt = 4;
fprintf('numTxAnt = %d\n',numTxAnt);
Tc = (idle_time + ramp_end_time)*numTxAnt;%s
f_start = 77e9;%Hz
f_center = f_start+(idle_time+adc_start_time+adc_samples/Fs/2)*S;
lambda = c/f_center;
Vmax = lambda/4/Tc;
num_chirps = 128;
Tf = num_chirps*Tc;
Vres = lambda/2/Tf;
fprintf('Vmax = %f m/s, Vres = %f m/s\n', Vmax, Vres);
fprintf('Vmax = %f km/h\n', Vmax*3.6);

fps_max = 1/(Tf);
fprintf('fps_max = %f\n',fps_max);

%% dataset mode3 12T16R
clear all;
% % clc;

disp('===========================================')
Fs = 25e6;%Hz
c = 3e8;%m/s
S = 46.880*1e6/1e-6;%Hz/s
Rmax = Fs*c/2/S;%m

fprintf('Fs = %d Mkps, S = %f MHz/us\n',Fs/1e6, S/1e12);

adc_samples = 90;
BW = adc_samples/Fs*S;%Hz
Rres = c/2/BW;%m


idle_time = 2e-6;%s
adc_start_time = 3.63e-6;%s
ramp_end_time = 7.44e-6;%s
BW_use = ramp_end_time * S;
fprintf('adc_samples = %d, BW = %f MHz, BW_use = %f MHz\n',adc_samples,BW/1e6, BW_use/1e6);
fprintf('Rmax = %d m, Rres = %f m\n',Rmax, Rres);

numTxAnt = 12;
fprintf('numTxAnt = %d\n',numTxAnt);
Tc = (idle_time + ramp_end_time)*numTxAnt;%s
f_start = 77e9;%Hz
f_center = f_start+(idle_time+adc_start_time+adc_samples/Fs/2)*S;
lambda = c/f_center;
Vmax = lambda/4/Tc;
num_chirps = 128;
Tf = num_chirps*Tc;
Vres = lambda/2/Tf;
fprintf('Vmax = %f m/s, Vres = %f m/s\n', Vmax, Vres);
fprintf('Vmax = %f km/h\n', Vmax*3.6);

fps_max = 1/(Tf);
fprintf('fps_max = %f\n',fps_max);

%% dataset mode4 12T16R
clear all;
% % clc;

disp('===========================================')
Fs = 25e6;%Hz
c = 3e8;%m/s
S = 70.006*1e6/1e-6;%Hz/s
Rmax = Fs*c/2/S;%m

fprintf('Fs = %d Mkps, S = %f MHz/us\n',Fs/1e6, S/1e12);

adc_samples = 60;
BW = adc_samples/Fs*S;%Hz
Rres = c/2/BW;%m


idle_time = 2e-6;%s
adc_start_time = 4.23e-6;%s
ramp_end_time = 6.84e-6;%s
BW_use = ramp_end_time * S;
fprintf('adc_samples = %d, BW = %f MHz, BW_use = %f MHz\n',adc_samples,BW/1e6, BW_use/1e6);
fprintf('Rmax = %d m, Rres = %f m\n',Rmax, Rres);

numTxAnt = 12;
fprintf('numTxAnt = %d\n',numTxAnt);
Tc = (idle_time + ramp_end_time)*numTxAnt;%s
f_start = 77e9;%Hz
f_center = f_start+(idle_time+adc_start_time+adc_samples/Fs/2)*S;
lambda = c/f_center;
Vmax = lambda/4/Tc;
num_chirps = 128;
Tf = num_chirps*Tc;
Vres = lambda/2/Tf;
fprintf('Vmax = %f m/s, Vres = %f m/s\n', Vmax, Vres);
fprintf('Vmax = %f km/h\n', Vmax*3.6);

fps_max = 1/(Tf);
fprintf('fps_max = %f\n',fps_max);

%% wuzhijing
clear all;
% % clc;

disp('===========================================')
Fs = 25e6;%Hz
c = 3e8;%m/s
S = 300*1e6/1e-6;%Hz/s
Rmax = Fs*c/2/S;%m

fprintf('Fs = %d Mkps, S = %f MHz/us\n',Fs/1e6, S/1e12);

adc_samples = 256;
BW = adc_samples/Fs*S;%Hz
Rres = c/2/BW;%m


idle_time = 7e-6;%s
adc_start_time = 2.88e-6;%s
ramp_end_time = 13.33e-6;%s
BW_use = ramp_end_time * S;
fprintf('adc_samples = %d, BW = %f MHz, BW_use = %f MHz\n',adc_samples,BW/1e6, BW_use/1e6);
fprintf('Rmax = %d m, Rres = %f m\n',Rmax, Rres);

numTxAnt = 12;
fprintf('numTxAnt = %d\n',numTxAnt);
Tc = (idle_time + ramp_end_time)*numTxAnt;%s
f_start = 77e9;%Hz
f_center = f_start+(idle_time+adc_start_time+adc_samples/Fs/2)*S;
lambda = c/f_center;
Vmax = lambda/4/Tc;
num_chirps = 200;
Tf = num_chirps*Tc;
Vres = lambda/2/Tf;
fprintf('Vmax = %f m/s, Vres = %f m/s\n', Vmax, Vres);
fprintf('Vmax = %f km/h\n', Vmax*3.6);

fps_max = 1/(Tf);
fprintf('fps_max = %f\n',fps_max);


