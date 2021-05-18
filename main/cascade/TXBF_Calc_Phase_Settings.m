
function [psSettingIdeal, psSettingProgram, psSettingProgramVal, ... 
          psSettingProgramCal, psSettingProgramCalVal, psError] = ... 
          TXBF_Calc_Phase_Settings(theta, lambda, d, numAnts, psOffsetCal)
% %
% TXBFCalcPhaseSettings - function to calculate the TX phase shifter values
% to be applied to the AWRx antenna array for TX-BF operation. 
%
% Currently only handles a single linear array of antenna elements spaced d
% meters apart. Antenna array axes defined such that first antenna element  
% is placed on the +x axis and beam angle theta sweeps from +x to -x axis. 
%
% Resulting arrays consists of the phase shifter offsets to apply during
% programming of the AWRx chirps. 
%
%   Inputs: 
%   theta   - desired beam steering angle between 0 and 180 degrees. 
%   d       - distance in meters between the linear array elements
%   lambda  - calculated wavelength at RF operating frequency
%   numAnts - number of antenna elements in the linear array
%   psOffsetCal - numAnt x phase-shifter offsets array consisting of the 
%                 calibrated phase shifter offsets
%
%
%   Outputs:
%   psSettingIdeal        - 
%   psSettingProgram      - 
%   psSettingProgramCal   - 
%   psError             - 
%
%
%%


% AWRx programmable phase shifter constraints
psDiscreteDeltaPhi = 360 / 64; % 6-bit phase shifter LSB increment (deg/lsb)
psDisceteRange = 0:psDiscreteDeltaPhi:360-psDiscreteDeltaPhi; % Full discrete phase shifter range (deg)
% dummy output for now
psSettingIdeal = zeros(1, numAnts);
psSettingProgram = zeros(1, numAnts);
psSettingProgramVal = zeros(1, numAnts);
psSettingProgramCal = zeros(1, numAnts);
psSettingProgramCalVal = zeros(1, numAnts);
psError = zeros(1, numAnts);

%% Step 1: compute ideal phase offsets for each antenna element based on
% requested theta and array dimensions. 

%deltaPhi = WrapTo360(360 * d / lambda * cos(theta * pi / 180)); % Phase offset applied between all elements (deg), this assumes the theta = 90 deg axis is boresight
deltaPhi = WrapTo360(360 * d / lambda * sin(theta * pi / 180)); % Phase offset applied between all elements (deg), this assumes the theta = 0 deg axis is boresight

for antIdx = 1:numAnts
    
    psSettingIdeal(antIdx) = WrapTo360((antIdx - 1) * deltaPhi);    % Calculate phase offset for all elements (deg).  
                                                                    % Value wrapped to between 0 and 360 degrees.    
end

%% Step 2: compute ideal discrete offsets for each antenna element 
% find ideal values nearest to the discrete phase shifter values 
for antIdx = 1:numAnts
    
    tempPsVal = 0;
    tempPsDiscreteError = 0;
    
    for numPsIdx = 0:63
        
        tempPsVal = numPsIdx * psDiscreteDeltaPhi;                  % keep adding more LSB discrete phase-shifter offsets
        tempPsDiscreteError = psSettingIdeal(antIdx) - tempPsVal;   % subtract this discrete value from the ideal
        
        if(tempPsDiscreteError < psDiscreteDeltaPhi)                % if the difference is less than a single LSB offset          
            psSettingProgramVal(antIdx) = WrapTo360(tempPsVal);     % save off the value as the closest discrete approximation
            psSettingProgram(antIdx) = WrapTo360(tempPsVal) / psDiscreteDeltaPhi;        % save off the setting as the closest discrete approximation
            
            break;            
        end
        
    end
end

%% Step 3: Referencing the Phase Shifter Offset Calibration Matrix find the closest offset to the ideal phase value
% psOffsetCal is a numAnts x 64 phase shifter settings matrix 
% For each antenna element find the calibration matrix point closest to the the 
% ideal phase shifter setting stored in psSettingIdeal
for antIdx = 1:numAnts
    
    phaseError = zeros(numAnts, 64);
    phaseErrorMinVal = zeros(1, numAnts);
    phaseErrorMinIndex = zeros(1, numAnts);
    
    for psIdx = 1:64 % look through each phase shifter offset setting     
        phaseError(antIdx, psIdx) = abs(psOffsetCal(antIdx, psIdx) - psSettingIdeal(antIdx)); % make a vector of all the differences 
                                                                                              % from each phase shifter offset
                                                                                              % to the ideal phase shifter value. These are
                                                                                              % the phase shifter "error" values
    end
    
    [phaseErrorMinVal(antIdx), phaseErrorMinIndex(antIdx)] = min(phaseError(antIdx, :));      % find the minimum error values. These are the 
                                                                                              % closest values found in the cal matrix. 
    
    psSettingProgramCal(antIdx) = phaseErrorMinIndex(antIdx) - 1;                             % the index corresponds to the programmable phase-shifter setting
    psSettingProgramCalVal(antIdx) = psOffsetCal(antIdx, phaseErrorMinIndex(antIdx));         % the value corresponds to the phase value being programmed

end



%% Step 4: find the error from the ideal offset value to the calibrated value 
% This is a measure of how close could we get using the cal matrix values and discrete phase shifters
for antIdx = 1:numAnts
    
    psError(antIdx) = psSettingProgramCalVal(antIdx) - psSettingIdeal(antIdx);
    
end