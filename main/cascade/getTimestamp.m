function timestamp = getTimestamp(adcIdxFileName, frameIdx)
    idxFile = fopen(adcIdxFileName,'r');    
    % ���ݷ���timestamp��λ��
    fseek(idxFile, 8+frameIdx*48, 'bof');       
    timestamp = fread(idxFile,1,'uint64');
    fclose(idxFile);    
end