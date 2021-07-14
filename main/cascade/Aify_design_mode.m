% author aifuyuan 
% 设置雷达发射模式

%% mode 1
clear all;close all;clc;
chirps = struct();
% chirp1
chirpIdx = 1;
chirps(chirpIdx).Fs = 22.5e6;%Hz
chirps(chirpIdx).c = 3e8;%m/s
chirps(chirpIdx).S = 42.003*1e6/1e-6;%Hz/s
chirps(chirpIdx).Rmax = chirps(chirpIdx).Fs*chirps(chirpIdx).c/2/chirps(chirpIdx).S;%m
chirps(chirpIdx).adc_samples = 256;
chirps(chirpIdx).BW = chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs*chirps(chirpIdx).S;%Hz
chirps(chirpIdx).Rres = chirps(chirpIdx).c/2/chirps(chirpIdx).BW;%m
chirps(chirpIdx).idle_time = 2e-6;%s
chirps(chirpIdx).adc_start_time = 2.72e-6;%s
chirps(chirpIdx).ramp_end_time = 14.32e-6;%s
chirps(chirpIdx).BW_use = chirps(chirpIdx).ramp_end_time * chirps(chirpIdx).S;
chirps(chirpIdx).numTxAnt = 12;
chirps(chirpIdx).Tc = (chirps(chirpIdx).idle_time + chirps(chirpIdx).ramp_end_time)*chirps(chirpIdx).numTxAnt;%s
chirps(chirpIdx).f_start = 77e9;%Hz
chirps(chirpIdx).f_center = chirps(chirpIdx).f_start+(chirps(chirpIdx).adc_start_time+chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs/2)*chirps(chirpIdx).S;
chirps(chirpIdx).lambda = chirps(chirpIdx).c/chirps(chirpIdx).f_center;
chirps(chirpIdx).Vmax = chirps(chirpIdx).lambda/4/chirps(chirpIdx).Tc;
chirps(chirpIdx).num_chirps = 64;
chirps(chirpIdx).Tf = chirps(chirpIdx).num_chirps*chirps(chirpIdx).Tc;
chirps(chirpIdx).Vres = chirps(chirpIdx).lambda/2/chirps(chirpIdx).Tf;
chirps(chirpIdx).fps_max = 1/(chirps(chirpIdx).Tf);
% show chirps
showChirpInfo(chirps);
%% mode 2
clear all;close all;clc;
chirps = struct();

% chirp1
chirpIdx = 1;
chirps(chirpIdx).Fs = 22.5e6;%Hz
chirps(chirpIdx).c = 3e8;%m/s
chirps(chirpIdx).S = 42.003*1e6/1e-6;%Hz/s
chirps(chirpIdx).Rmax = chirps(chirpIdx).Fs*chirps(chirpIdx).c/2/chirps(chirpIdx).S;%m
chirps(chirpIdx).adc_samples = 256;
chirps(chirpIdx).BW = chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs*chirps(chirpIdx).S;%Hz
chirps(chirpIdx).Rres = chirps(chirpIdx).c/2/chirps(chirpIdx).BW;%m
chirps(chirpIdx).idle_time = 2e-6;%s
chirps(chirpIdx).adc_start_time = 2.72e-6;%s
chirps(chirpIdx).ramp_end_time = 14.32e-6;%s
chirps(chirpIdx).BW_use = chirps(chirpIdx).ramp_end_time * chirps(chirpIdx).S;
chirps(chirpIdx).numTxAnt = 12;
chirps(chirpIdx).Tc = (chirps(chirpIdx).idle_time + chirps(chirpIdx).ramp_end_time)*chirps(chirpIdx).numTxAnt;%s
chirps(chirpIdx).f_start = 77e9;%Hz
chirps(chirpIdx).f_center = chirps(chirpIdx).f_start+(chirps(chirpIdx).adc_start_time+chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs/2)*chirps(chirpIdx).S;
chirps(chirpIdx).lambda = chirps(chirpIdx).c/chirps(chirpIdx).f_center;
chirps(chirpIdx).Vmax = chirps(chirpIdx).lambda/4/chirps(chirpIdx).Tc;
chirps(chirpIdx).num_chirps = 64;
chirps(chirpIdx).Tf = chirps(chirpIdx).num_chirps*chirps(chirpIdx).Tc;
chirps(chirpIdx).Vres = chirps(chirpIdx).lambda/2/chirps(chirpIdx).Tf;
chirps(chirpIdx).fps_max = 1/(chirps(chirpIdx).Tf);

chirpIdx = 2;
chirps(chirpIdx).Fs = 22.5e6;%Hz
chirps(chirpIdx).c = 3e8;%m/s
chirps(chirpIdx).S = 42.003*1e6/1e-6;%Hz/s
chirps(chirpIdx).Rmax = chirps(chirpIdx).Fs*chirps(chirpIdx).c/2/chirps(chirpIdx).S;%m
chirps(chirpIdx).adc_samples = 256;
chirps(chirpIdx).BW = chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs*chirps(chirpIdx).S;%Hz
chirps(chirpIdx).Rres = chirps(chirpIdx).c/2/chirps(chirpIdx).BW;%m
chirps(chirpIdx).idle_time = 10e-6;%s
chirps(chirpIdx).adc_start_time = 2.72e-6;%s
chirps(chirpIdx).ramp_end_time = 14.32e-6;%s
chirps(chirpIdx).BW_use = chirps(chirpIdx).ramp_end_time * chirps(chirpIdx).S;
chirps(chirpIdx).numTxAnt = 12;
chirps(chirpIdx).Tc = (chirps(chirpIdx).idle_time + chirps(chirpIdx).ramp_end_time)*chirps(chirpIdx).numTxAnt;%s
chirps(chirpIdx).f_start = 77e9;%Hz
chirps(chirpIdx).f_center = chirps(chirpIdx).f_start+(chirps(chirpIdx).adc_start_time+chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs/2)*chirps(chirpIdx).S;
chirps(chirpIdx).lambda = chirps(chirpIdx).c/chirps(chirpIdx).f_center;
chirps(chirpIdx).Vmax = chirps(chirpIdx).lambda/4/chirps(chirpIdx).Tc;
chirps(chirpIdx).num_chirps = 64;
chirps(chirpIdx).Tf = chirps(chirpIdx).num_chirps*chirps(chirpIdx).Tc;
chirps(chirpIdx).Vres = chirps(chirpIdx).lambda/2/chirps(chirpIdx).Tf;
chirps(chirpIdx).fps_max = 1/(chirps(chirpIdx).Tf);

% show chirps
showChirpInfo(chirps);

%% mode 3
clear all;close all;clc;
chirps = struct();

% chirp1
chirpIdx = 1;
chirps(chirpIdx).Fs = 22.5e6;%Hz
chirps(chirpIdx).c = 3e8;%m/s
chirps(chirpIdx).S = 42.003*1e6/1e-6;%Hz/s
chirps(chirpIdx).Rmax = chirps(chirpIdx).Fs*chirps(chirpIdx).c/2/chirps(chirpIdx).S;%m
chirps(chirpIdx).adc_samples = 256;
chirps(chirpIdx).BW = chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs*chirps(chirpIdx).S;%Hz
chirps(chirpIdx).Rres = chirps(chirpIdx).c/2/chirps(chirpIdx).BW;%m
chirps(chirpIdx).idle_time = 2e-6;%s
chirps(chirpIdx).adc_start_time = 2.72e-6;%s
chirps(chirpIdx).ramp_end_time = 14.32e-6;%s
chirps(chirpIdx).BW_use = chirps(chirpIdx).ramp_end_time * chirps(chirpIdx).S;
chirps(chirpIdx).numTxAnt = 12;
chirps(chirpIdx).Tc = (chirps(chirpIdx).idle_time + chirps(chirpIdx).ramp_end_time)*chirps(chirpIdx).numTxAnt;%s
chirps(chirpIdx).f_start = 77e9;%Hz
chirps(chirpIdx).f_center = chirps(chirpIdx).f_start+(chirps(chirpIdx).adc_start_time+chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs/2)*chirps(chirpIdx).S;
chirps(chirpIdx).lambda = chirps(chirpIdx).c/chirps(chirpIdx).f_center;
chirps(chirpIdx).Vmax = chirps(chirpIdx).lambda/4/chirps(chirpIdx).Tc;
chirps(chirpIdx).num_chirps = 64;
chirps(chirpIdx).Tf = chirps(chirpIdx).num_chirps*chirps(chirpIdx).Tc;
chirps(chirpIdx).Vres = chirps(chirpIdx).lambda/2/chirps(chirpIdx).Tf;
chirps(chirpIdx).fps_max = 1/(chirps(chirpIdx).Tf);

chirpIdx = 2;
chirps(chirpIdx).Fs = 22.5e6;%Hz
chirps(chirpIdx).c = 3e8;%m/s
chirps(chirpIdx).S = 42.003*1e6/1e-6;%Hz/s
chirps(chirpIdx).Rmax = chirps(chirpIdx).Fs*chirps(chirpIdx).c/2/chirps(chirpIdx).S;%m
chirps(chirpIdx).adc_samples = 256;
chirps(chirpIdx).BW = chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs*chirps(chirpIdx).S;%Hz
chirps(chirpIdx).Rres = chirps(chirpIdx).c/2/chirps(chirpIdx).BW;%m
chirps(chirpIdx).idle_time = 2e-6;%s
chirps(chirpIdx).adc_start_time = 2.72e-6;%s
chirps(chirpIdx).ramp_end_time = 14.32e-6;%s
chirps(chirpIdx).BW_use = chirps(chirpIdx).ramp_end_time * chirps(chirpIdx).S;
chirps(chirpIdx).numTxAnt = 1;
chirps(chirpIdx).Tc = (chirps(chirpIdx).idle_time + chirps(chirpIdx).ramp_end_time)*chirps(chirpIdx).numTxAnt;%s
chirps(chirpIdx).f_start = 77e9;%Hz
chirps(chirpIdx).f_center = chirps(chirpIdx).f_start+(chirps(chirpIdx).adc_start_time+chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs/2)*chirps(chirpIdx).S;
chirps(chirpIdx).lambda = chirps(chirpIdx).c/chirps(chirpIdx).f_center;
chirps(chirpIdx).Vmax = chirps(chirpIdx).lambda/4/chirps(chirpIdx).Tc;
chirps(chirpIdx).num_chirps = 1024;
chirps(chirpIdx).Tf = chirps(chirpIdx).num_chirps*chirps(chirpIdx).Tc;
chirps(chirpIdx).Vres = chirps(chirpIdx).lambda/2/chirps(chirpIdx).Tf;
chirps(chirpIdx).fps_max = 1/(chirps(chirpIdx).Tf);

% show chirps
showChirpInfo(chirps);

%% mode 4
clear all;close all;clc;
chirps = struct();

% chirp1
chirpIdx = 1;
chirps(chirpIdx).Fs = 22.5e6;%Hz
chirps(chirpIdx).c = 3e8;%m/s
chirps(chirpIdx).S = 42.003*1e6/1e-6;%Hz/s
chirps(chirpIdx).Rmax = chirps(chirpIdx).Fs*chirps(chirpIdx).c/2/chirps(chirpIdx).S;%m
chirps(chirpIdx).adc_samples = 256;
chirps(chirpIdx).BW = chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs*chirps(chirpIdx).S;%Hz
chirps(chirpIdx).Rres = chirps(chirpIdx).c/2/chirps(chirpIdx).BW;%m
chirps(chirpIdx).idle_time = 2e-6;%s
chirps(chirpIdx).adc_start_time = 2.72e-6;%s
chirps(chirpIdx).ramp_end_time = 14.32e-6;%s
chirps(chirpIdx).BW_use = chirps(chirpIdx).ramp_end_time * chirps(chirpIdx).S;
chirps(chirpIdx).numTxAnt = 12;
chirps(chirpIdx).Tc = (chirps(chirpIdx).idle_time + chirps(chirpIdx).ramp_end_time)*chirps(chirpIdx).numTxAnt;%s
chirps(chirpIdx).f_start = 77e9;%Hz
chirps(chirpIdx).f_center = chirps(chirpIdx).f_start+(chirps(chirpIdx).adc_start_time+chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs/2)*chirps(chirpIdx).S;
chirps(chirpIdx).lambda = chirps(chirpIdx).c/chirps(chirpIdx).f_center;
chirps(chirpIdx).Vmax = chirps(chirpIdx).lambda/4/chirps(chirpIdx).Tc;
chirps(chirpIdx).num_chirps = 64;
chirps(chirpIdx).Tf = chirps(chirpIdx).num_chirps*chirps(chirpIdx).Tc;
chirps(chirpIdx).Vres = chirps(chirpIdx).lambda/2/chirps(chirpIdx).Tf;
chirps(chirpIdx).fps_max = 1/(chirps(chirpIdx).Tf);

chirpIdx = 2;
chirps(chirpIdx).Fs = 22.5e6;%Hz
chirps(chirpIdx).c = 3e8;%m/s
chirps(chirpIdx).S = 42.003*1e6/1e-6;%Hz/s
chirps(chirpIdx).Rmax = chirps(chirpIdx).Fs*chirps(chirpIdx).c/2/chirps(chirpIdx).S;%m
chirps(chirpIdx).adc_samples = 256;
chirps(chirpIdx).BW = chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs*chirps(chirpIdx).S;%Hz
chirps(chirpIdx).Rres = chirps(chirpIdx).c/2/chirps(chirpIdx).BW;%m
chirps(chirpIdx).idle_time = 2e-6;%s
chirps(chirpIdx).adc_start_time = 2.72e-6;%s
chirps(chirpIdx).ramp_end_time = 14.32e-6;%s
chirps(chirpIdx).BW_use = chirps(chirpIdx).ramp_end_time * chirps(chirpIdx).S;
chirps(chirpIdx).numTxAnt = 4;
chirps(chirpIdx).Tc = (chirps(chirpIdx).idle_time + chirps(chirpIdx).ramp_end_time)*chirps(chirpIdx).numTxAnt;%s
chirps(chirpIdx).f_start = 77e9;%Hz
chirps(chirpIdx).f_center = chirps(chirpIdx).f_start+(chirps(chirpIdx).adc_start_time+chirps(chirpIdx).adc_samples/chirps(chirpIdx).Fs/2)*chirps(chirpIdx).S;
chirps(chirpIdx).lambda = chirps(chirpIdx).c/chirps(chirpIdx).f_center;
chirps(chirpIdx).Vmax = chirps(chirpIdx).lambda/4/chirps(chirpIdx).Tc;
chirps(chirpIdx).num_chirps = 256;
chirps(chirpIdx).Tf = chirps(chirpIdx).num_chirps*chirps(chirpIdx).Tc;
chirps(chirpIdx).Vres = chirps(chirpIdx).lambda/2/chirps(chirpIdx).Tf;
chirps(chirpIdx).fps_max = 1/(chirps(chirpIdx).Tf);

% show chirps
showChirpInfo(chirps);