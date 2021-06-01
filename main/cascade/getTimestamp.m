function timestamp = getTimestamp(adcIdxFileName, frameIdx)
    idxFile = fopen(adcIdxFileName,'r');    
    % 根据分析timestamp的位置
    fseek(idxFile, 8+frameIdx*48, 'bof');       
    timestamp = uint64(fread(idxFile,1,'uint64'));
    fclose(idxFile);    
end