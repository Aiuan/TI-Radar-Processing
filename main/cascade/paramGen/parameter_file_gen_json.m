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

% parameter_file_gen_Jason.m
%  
% Function call to generate a complete parameter file using Jason file before signal processing, including chirp parameters and module parameters.
% This function does not support advanced frame config
%input:
%   test_name: test data file name which contains the chirp configuration
%   file in a Jason file format
%   dataFolder_calib: calibration data file path which contains the pre-calibrated .mat file. This .mat file includes the calibration matrix to be used in adc data calibration module
%   module_param_file: contains parameters to initialize each signal processing module
%   pathGenParaFile: the complete parameter file name to be generated
%   dataPlatform: flag for data capture platform


function parameter_file_gen_Jason(test_name, dataFolder_calib,module_param_file, pathGenParaFile, dataPlatform)

% create param file
%param_fileName = strcat(test_name,'test_param.m');
fidParam = fopen(pathGenParaFile, 'w');



 %% read header 对新生成的参数文件写入头部
    fidhead = fopen('header.m','r');
    tline = fgets(fidhead);
    while ischar(tline)
        fwrite(fidParam, tline);
        tline = fgets(fidhead);
    end
    fclose(fidhead);
    fprintf(fidParam, '\n');
    fprintf(fidParam, '\n');
    fprintf(fidParam, '\n');
    
    fprintf(fidParam, 'ADVANCED_FRAME_CONFIG = 0; \n');
    fprintf(fidParam, 'dataPlatform = ''%s''; \n', dataPlatform);

%% find *.json file in the same folder of the test data; if json files in the folder, this code will NOT work, please make sure only one exist
f=dir(fullfile(test_name,'*.mmwave.json'));%找到原始数据文件夹中config.mmwave.json的文件名
if length(f)~=1%如果没找到，显示异常并退出
    error('Unknown parameter file or too many chirpProfiles_*.json file!!');
else
    
    disp(['paramFile= ' fullfile(test_name,f.name)]);%打印找到的原始数据文件夹中config.mmwave.json的文件路径
    paramFile = fullfile(test_name,f.name);%存储原始数据文件夹中config.mmwave.json的文件路径
    
    %go to the test data folder to run the .json file and read the chirp parameters
    %associated with the data   
    params_chirp = JsonParser(paramFile);%读取json文件中的参数    
   
    numChirpConfig = length(params_chirp.DevConfig(1).Chirp);%1个frame中的chirp数量
    numTXPerDev = 3;%每个芯片发射通道为3
    totTx = numTXPerDev*params_chirp.NumDevices;%3*4=12，一共12发射通道
    TxEnableTable = zeros(numChirpConfig, totTx);%构建发射的chirp的使能表，维度为 chirp数量*12发射通道
    for iDev = 1:params_chirp.NumDevices%第iDev个设备
        for iconfig = 1:numChirpConfig%第iconfig个chirp
            TxEnableTable(iconfig,1+(iDev-1)*numTXPerDev) = ...
                params_chirp.DevConfig(iDev).Chirp(iconfig).Tx0Enable;
            TxEnableTable(iconfig, 2+(iDev-1)*numTXPerDev) = ...
                params_chirp.DevConfig(iDev).Chirp(iconfig).Tx1Enable;
            TxEnableTable(iconfig, 3+(iDev-1)*numTXPerDev) = ...
                params_chirp.DevConfig(iDev).Chirp(iconfig).Tx2Enable;
        end
    end
    
    TxChannelEnabled = zeros(1,numChirpConfig);%构建发射channel使能表，维度为 1*chirp数量
    for iconfig = 1:numChirpConfig%第iconfig个chirp
        [channelID] = find(TxEnableTable(iconfig,:)~=0);%寻找第iconfig个chirp使能的天线
         TxChannelEnabled(iconfig) = channelID;     
    end
    
    %% pass the chirp parameter file to _param.m 将json中的配置信息全部写入将要新生成的参数文件
    % assuming 4 devices use the same config, so only device 1 chirp configuration information
    % is passed to the post processing
    fprintf(fidParam, '%%pass the chirp parameters associated with test data \n');
    fprintf(fidParam, 'numADCSample = %e; \n', params_chirp.DevConfig(1).Profile.NumSamples);
    fprintf(fidParam, 'adcSampleRate = %e; %%Hz/s \n', params_chirp.DevConfig(1).Profile.SamplingRate*1e3);
    fprintf(fidParam, 'startFreqConst = %e; %%Hz \n', params_chirp.DevConfig(1).Profile.StartFreq*1e9);
    fprintf(fidParam, 'chirpSlope = %e; %%Hz/s \n',params_chirp.DevConfig(1).Profile.FreqSlope*1e12);
    fprintf(fidParam, 'chirpIdleTime = %e; %%s \n', params_chirp.DevConfig(1).Profile.IdleTime*1e-6);
    fprintf(fidParam, 'adcStartTimeConst = %e; %%s \n', params_chirp.DevConfig(1).Profile.AdcStartTime*1e-6);
    fprintf(fidParam, 'chirpRampEndTime = %d; %%s \n', params_chirp.DevConfig(1).Profile.RampEndTime*1e-6);
    fprintf(fidParam, 'framePeriodicty = %d; \n', params_chirp.DevConfig(1).FrameConfig.Periodicity * 1e-3);
    fprintf(fidParam, 'NumDevices = %d; \n', params_chirp.NumDevices);
    fprintf(fidParam, 'framePeriodicty = %d; \n', params_chirp.DevConfig(1).FrameConfig.Periodicity * 1e-3);

    fprintf(fidParam, 'frameCount = %e; %%s \n', params_chirp.DevConfig(1).FrameConfig.NumFrames); 
    fprintf(fidParam, 'numChirpsInLoop = %e; %%s \n', params_chirp.DevConfig(1).NumChirps);        

        
    fprintf(fidParam, 'nchirp_loops = %d; \n', params_chirp.DevConfig(1).FrameConfig.NumChirpLoops);
        
    fprintf(fidParam, 'numTxAnt = %d; \n', length(params_chirp.TxToEnable));   
    fprintf(fidParam, ['TxToEnable = [' num2str(TxChannelEnabled) ']' ';\n']);
    fprintf(fidParam, 'numRxToEnable = %d; \n', length(params_chirp.RxToEnable));      
   
    Start_Freq_GHz = params_chirp.DevConfig(1).Profile.StartFreq;
    Samples_per_Chirp = params_chirp.DevConfig(1).Profile.NumSamples;
    Sampling_Rate_ksps = params_chirp.DevConfig(1).Profile.SamplingRate;
    Slope_MHzperus = params_chirp.DevConfig(1).Profile.FreqSlope;
    fprintf(fidParam, 'centerFreq = %d; \n', ((Start_Freq_GHz + Samples_per_Chirp/...
        Sampling_Rate_ksps*Slope_MHzperus/2)));
    
    
    %% calibration cascade parameters
    load(dataFolder_calib)%加载校准参数
    paramsCalib = params;%存储校准参数
    clear params;%删除临时变量
    fprintf(fidParam, '%%pass the slope used for calibration \n');
    fprintf(fidParam, 'Slope_calib = %d; \n\n', paramsCalib.Slope_MHzperus*1e12);
    fprintf(fidParam, 'fs_calib = %d; \n\n', paramsCalib.Sampling_Rate_sps);
    
    %% combine with the rest of the signal 利用.\paramGen\module_param.m载入额外的配置信息
    fprintf(fidParam, '%%pass all other parameters \n');
    fidCommon = fopen(module_param_file);%打开.\paramGen\module_param.m文件
    for ii = 1:32
        tline = fgets(fidCommon);
    end%跳过.\paramGen\module_param.m中前32行注释
    tline = fgets(fidCommon);
    while ischar(tline)%将.\paramGen\module_param.m后续部分写入./input/test1_param.m
        fwrite(fidParam, tline);
        tline = fgets(fidCommon);
    end
    fclose(fidParam);
    fclose(fidCommon);
    
end


