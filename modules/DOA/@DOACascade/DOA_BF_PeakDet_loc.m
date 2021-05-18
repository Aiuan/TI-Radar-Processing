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
%

%DOA_BF_PeakDet_loc.m
%
% DOA_BF_PeakDet_loc function perform peak detection based on the input
% angle spectrum

%input:
%   obj: object instance
%   inData: angle spectrum

%output:
%   peakVal: value of detected peaks
%   peakLoc: index of detected peaks

function [peakVal, peakLoc] = DOA_BF_PeakDet_loc(obj, inData)


gamma = obj.gamma;
sidelobeLevel_dB = obj.sidelobeLevel_dB;

inData = inData(:);

minVal = Inf;
maxVal = 0;
maxLoc = 0;
maxData = [];

locateMax = 0;  % at beginning, not ready for peak detection
km = 1;         % constant value used in variance calculation

numMax = 0;
extendLoc = 0;
initStage = 1;
absMaxValue = 0;

i = 0;
N = length(inData);
while (i < (N + extendLoc - 1))
    i = i+1;
    i_loc = rem(i-1, N)+1;
    currentVal = inData(i_loc);
    % record the maximum value
    if currentVal > absMaxValue
        absMaxValue = currentVal;
    end
    % record the current max value and location
    if currentVal > maxVal
        maxVal = currentVal;
        maxLoc = i_loc;
        maxLoc_r = i;
    end
    
    % record for the current min value and location
    if currentVal < minVal,
        minVal = currentVal;
    end
    
    if locateMax
        if currentVal < maxVal/gamma
            numMax = numMax + 1;
            bwidth = i - maxLoc_r;
            % Assign maximum value only if the value has fallen below the max by
            % gamma, thereby declaring that the max value was a peak
            maxData = [maxData(1:numMax-1,:) ; maxLoc maxVal bwidth maxLoc_r];
            
            minVal = currentVal;
            locateMax = 0;
        end
    else
        if currentVal > minVal*gamma
            % Assign minimum value if the value has risen above the min by
            % gamma, thereby declaring that the min value was a valley
            locateMax = 1;
            maxVal = currentVal;
            if (initStage == 1)
                extendLoc = i;
                initStage = 0;
            end
        end
    end
end


%make sure the max value needs to be cetain dB higher than the side lobes
%to declare any detection
estVar = zeros(numMax, 1);
peakVal = zeros(numMax, 1);
peakLoc = zeros(numMax, 1);
delta = [];

%[v ind] = max(maxData(:,2));
%peakMean = mean(maxData([1:(ind-1) ind+1:end],2));
%SNR_DOA = v/peakMean;
%if v>peakMean*maxPeakThre
% if the max is different by more than sidelobeLevel_dB dB from the
% peak, then removed it as a sidelobe
maxData_ = [];
numMax_ = 0;
totPower = 0;
for i = 1:numMax
    if maxData(i, 2) >= absMaxValue * (10^(-sidelobeLevel_dB/10))
        numMax_ = numMax_ + 1;
        maxData_(numMax_,:) = maxData(i, :);
        totPower = totPower + maxData(i, 2);
    end
end
maxData = maxData_;
numMax = numMax_;

estVar = zeros(numMax, 1);
peakVal = zeros(numMax, 1);
peakLoc = zeros(numMax, 1);

delta = [];
for ind = 1:numMax
    peakVal(ind) = maxData(ind,2);
    peakLoc(ind) = rem(maxData(ind,1)-1, N)+1;
end






