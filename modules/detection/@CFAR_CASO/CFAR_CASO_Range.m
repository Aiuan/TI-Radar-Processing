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

%CFAR_CASO_Range.m
%
%This function performs CFAR_CASO detection along range direction

%input
%   obj: object instance of CFAR_CASO
%   sig: a 2D real valued matrix, range x Doppler 

%output
%   N_obj: number of objects detected
%   Ind_obj: 2D bin index of the detected object
%   noise_obj: noise value at the detected point after integration


function [N_obj, Ind_obj, noise_obj, CFAR_SNR] = CFAR_CASO_Range(obj, sig)

cellNum = obj.refWinSize;
gapNum = obj.guardWinSize;
cellNum = cellNum(1);
gapNum = gapNum(1);
K0 = obj.K0(1);

M_samp=size(sig, 1);
N_pul=size(sig, 2);
%% input:  assuming size(input) = [numSamplePerChipr numChirpsPerFrame numAntenna]


%for each point under test, gapNum samples on the two sides are excluded
%from averaging. Left cellNum/2 and right cellNum/2 samples are used for
%averaging
gaptot=gapNum + cellNum;
N_obj=0;
Ind_obj=[];
noise_obj = [];
CFAR_SNR = [];

discardCellLeft = obj.discardCellLeft;
discardCellRight = obj.discardCellRight;


%for the first gaptot samples only use the right sample
for k=1:N_pul
    sigv=(sig(:,k))';
    vec = sigv(discardCellLeft+1:M_samp-discardCellRight);
    vecLeft = vec(1:(gaptot));
    vecRight = vec(end-(gaptot)+1:end);
    vec = [vecLeft vec vecRight];
    for j=1:(M_samp-discardCellLeft-discardCellRight)
        cellInd=[j-gaptot: j-gapNum-1 j+gapNum+1:j+gaptot];
        cellInd=cellInd + gaptot;
        cellInda=[j-gaptot: j-gapNum-1];
        cellInda=cellInda + gaptot;
        cellIndb=[ j+gapNum+1:j+gaptot];
        cellIndb=cellIndb + gaptot;
        
        cellave1a =sum(vec(cellInda))/(cellNum);
        cellave1b =sum(vec(cellIndb))/(cellNum);
        cellave1 = min(cellave1a,cellave1b);
        
        %if((j > discardCellLeft) && (j < (M_samp-discardCellRight)))
        if obj.maxEnable==1 %check if it is local maximum peak
            maxInCell = max(vec([cellInd(1):cellInd(end)]));
            if (vec(j+gaptot)>K0*cellave1 && ( vec(j+gaptot)>=maxInCell))
                N_obj=N_obj+1;
                Ind_obj(N_obj,:)=[j+discardCellLeft, k];
                noise_obj(N_obj) = cellave1; %save the noise level
                CFAR_SNR(N_obj) = vec(j+gaptot)/cellave1;
            end
        else
            if vec(j+gaptot)>K0*cellave1
                N_obj=N_obj+1;
                Ind_obj(N_obj,:)=[j+discardCellLeft, k];
                noise_obj(N_obj) = cellave1; %save the noise level
                CFAR_SNR(N_obj) = vec(j+gaptot)/cellave1;
            end
        end        
    end
end

%get the noise variance for each antenna
for i_obj = 1:N_obj
    
    ind_range = Ind_obj(i_obj,1);
    ind_Dop = Ind_obj(i_obj,2);
    if ind_range<= gaptot
        %on the left boundary, use the right side samples twice
        cellInd=[ind_range+gapNum+1:ind_range+gaptot ind_range+gapNum+1:ind_range+gaptot];
    elseif ind_range>=M_samp-gaptot+1
        %on the right boundary, use the left side samples twice
        cellInd=[ind_range-gaptot: ind_range-gapNum-1 ind_range-gaptot: ind_range-gapNum-1];
    else
        cellInd=[ind_range-gaptot: ind_range-gapNum-1 ind_range+gapNum+1:ind_range+gaptot];
        
    end
    
    
end

