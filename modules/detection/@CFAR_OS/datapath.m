function [detection_results] = datapath(obj, input)
%% 距离维CFAR的输入数据为adcData，循环选择每个发射天线的数据（3D数据）

sig_integrate = sum((abs(input)).^2,3) + 1; %沿天线阵列组合非相干信号变成2维数据

angleFFTSize = 128; 
angleBinSkipLeft = 4;
angleBinSkipRight = 4;


if (obj.detectMethod == 1) %进行OS CFAR检测
    [N_obj_Rag, Ind_obj_Rag, noise_obj, CFAR_SNR] = CFAR_OS_Range(obj, sig_integrate); %进行距离维上目标检测
    %距离维检测返回检测目标数目和检测目标索引以及门限水平和SNR
    N_obj = 0; %多普勒维目标数目初始化
    Ind_obj = []; %多普勒维目标索引初始化
    detection_results = {}; %检测结果初始化
    if (N_obj_Rag>0) %如果距离维检测到目标
        [N_obj, Ind_obj] = CFAR_OS_Doppler_overlap(obj, Ind_obj_Rag, input, sig_integrate); %则进行多普勒维检测
        detection_results = [];
        
        %使用首次噪声估计并应用于第二次检测
        noise_obj_agg = [];
        for i_obj = 1:N_obj %对多普勒维的每个目标
            indx1R = Ind_obj(i_obj,1); %返回多普勒维目标索引
            indx1D = Ind_obj(i_obj,2); %返回多普勒维循环次数
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




