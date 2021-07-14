function showChirpInfo(chirps)
    
    for i=1:size(chirps,2)
        fprintf('==================chirp %d=======================\n', i );
        
        fprintf('Rmax = %f m, Rres = %f m\n', chirps(i).Rmax, chirps(i).Rres);
        fprintf('Vmax = %f m/s, Vres = %f m/s\n', chirps(i).Vmax, chirps(i).Vres);
        fprintf('Vmax = %f km/h, Vres = %f km/h\n', chirps(i).Vmax*3.6, chirps(i).Vres*3.6);
        
        fprintf('Fs = %f Mkps, S = %f MHz/us\n',chirps(i).Fs/1e6,chirps(i). S/1e12);
        fprintf('adc_samples = %d, BW = %f MHz, BW_use = %f MHz\n',chirps(i).adc_samples, chirps(i).BW/1e6, chirps(i).BW_use/1e6);
        fprintf('idel_time = %f us, adc_start_time = %f us, ramp_end_time = %f us\n', chirps(i).idle_time*1e6, chirps(i).adc_start_time*1e6, chirps(i).ramp_end_time*1e6);
        fprintf('numTxAnt = %d\n',chirps(i).numTxAnt);
        fprintf('num_chirps = %d\n',chirps(i).num_chirps);
        fprintf('fps_max = %f\n',chirps(i).fps_max);

    end

end