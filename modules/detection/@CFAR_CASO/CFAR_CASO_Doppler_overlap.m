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

%CFAR_CASO_Doppler_overlap.m
%
% This function performs 1D CFAR_CASO detection along the
% Doppler direction, and declare detection only if the index overlap with
% range detection results. 

%input
%   obj: object instance of CFAR_CASO
%   Ind_obj_Rag: index of range bins that has been determined by the first
%   step detection along the range direction
%   sigCpml: a 3D complex matrix, range x Doppler x antenna array
%   sig_integ: a 2D real valued matrix, range x Doppler 

%output
%   N_obj: number of objects detected
%   Ind_obj: 2D bin index of the detected object
%   noise_obj_an: antenna specific noise estimation before integration


function [N_obj, Ind_obj, noise_obj_an] = CFAR_CASO_Doppler_overlap(obj, Ind_obj_Rag, sigCpml, sig_integ)


maxEnable = obj.maxEnable;
cellNum0 = obj.refWinSize;
gapNum0 = obj.guardWinSize;
cellNum = cellNum0(2);
gapNum = gapNum0(2);
K0 = obj.K0(2);

rangeNumBins = size(sig_integ,1);

%extract the detected points after range detection
detected_Rag_Cell = unique(Ind_obj_Rag(:,1));
sig = sig_integ(detected_Rag_Cell,:);

M_samp=size(sig, 1);
N_pul=size(sig, 2);


%for each point under test, gapNum samples on the two sides are excluded
%from averaging. Left cellNum/2 and right cellNum/2 samples are used for
%averaging
gaptot=gapNum + cellNum;

N_obj=0;
Ind_obj=[];
noise_obj_an = [];
vec=zeros(1,N_pul+gaptot*2);
for k=1:M_samp
    %get the range index at current range index
    detected_Rag_Cell_i = detected_Rag_Cell(k);
    ind1 = find(Ind_obj_Rag(:,1) == detected_Rag_Cell_i);
    indR = Ind_obj_Rag(ind1, 2);
    %extend the left the vector by copying the left most the right most
    %gaptot samples are not detected.
    sigv=(sig(k,:));
    vec(1:gaptot) = sigv(end-gaptot+1:end);
    vec(gaptot+1: N_pul+gaptot) = sigv;
    vec(N_pul+gaptot+1:end) = sigv(1:gaptot);
    %start to process
    ind_loc_all = [];
    ind_loc_Dop = [];
    ind_obj_0 = 0;
    noiseEst = zeros(1,N_pul);
    for j=1+gaptot:N_pul+gaptot
        cellInd=[j-gaptot: j-gapNum-1 j+gapNum+1:j+gaptot];
        noiseEst(j-gaptot) = sum(vec(cellInd));
    end
    for j=1+gaptot:N_pul+gaptot
        j0 = j - gaptot;
        cellInd=[j-gaptot: j-gapNum-1 j+gapNum+1:j+gaptot];
        cellInda = [j-gaptot: j-gapNum-1 ];
        cellIndb =[j+gapNum+1:j+gaptot];
        
        cellave1a =sum(vec(cellInda))/(cellNum);
        cellave1b =sum(vec(cellIndb))/(cellNum);
        cellave1 = min(cellave1a,cellave1b);        
        
        maxInCell = max(vec(cellInd));
        if maxEnable==1
            %detect only if it is the maximum within window
            condition = ((vec(j)>K0*cellave1)) && ((vec(j)>maxInCell));
        else
            condition = vec(j)>K0*cellave1;
        end
        
        if condition==1
            %check if this detection overlap with the Doppler detection
            if(find(indR == j0))
                %find overlap, declare a detection
                ind_win = detected_Rag_Cell_i;
                %range index
                ind_loc_all = [ind_loc_all ind_win];
                %Doppler index
                ind_loc_Dop = [ind_loc_Dop j0];
            end
            
        end
        
    end
    ind_obj_0 = [];
    
    
    if (length(ind_loc_all)>0)
        ind_obj_0(:,1) = ((ind_loc_all));
        ind_obj_0(:,2) = ind_loc_Dop;
        if size(Ind_obj,1) ==0
            Ind_obj = ind_obj_0;
        else
            
            %following process is to avoid replicated detection points
            ind_obj_0_sum = ind_loc_all + 10000*ind_loc_Dop;
            Ind_obj_sum = Ind_obj(:,1) + 10000*Ind_obj(:,2);
            for ii= 1: length(ind_loc_all)
                if (length(find(Ind_obj_sum == ind_obj_0_sum(ii)))==0)
                    Ind_obj = [Ind_obj ; ind_obj_0(ii,:)];
                end
            end
        end
    end
    
end

N_obj = size(Ind_obj,1);

%reset the ref window size to range direction
cellNum = cellNum0(1);
gapNum = gapNum0(1);
gaptot=gapNum + cellNum;
%get the noise variance for each antenna
N_obj_valid = 0;
Ind_obj_valid = [];
for i_obj = 1:N_obj    
    ind_range = Ind_obj(i_obj,1);
    ind_Dop = Ind_obj(i_obj,2);
    %skip detected points with signal power less than obj.powerThre
    if (min(abs(sigCpml(ind_range, ind_Dop,:)).^2) < obj.powerThre)
        continue;
    end
    if ind_range<= gaptot
        %on the left boundary, use the right side samples twice
        cellInd=[ind_range+gapNum+1:ind_range+gaptot ind_range+gapNum+1:ind_range+gaptot];
    elseif ind_range>=rangeNumBins-gaptot+1
        %on the right boundary, use the left side samples twice
        cellInd=[ind_range-gaptot: ind_range-gapNum-1 ind_range-gaptot: ind_range-gapNum-1];
    else
        cellInd=[ind_range-gaptot: ind_range-gapNum-1 ind_range+gapNum+1:ind_range+gaptot];
        
    end
    
    N_obj_valid = N_obj_valid +1;
    noise_obj_an(:, i_obj) = reshape((mean(abs(sigCpml(cellInd, ind_Dop, :)).^2, 1)), obj.numAntenna, 1, 1);
    
    Ind_obj_valid(N_obj_valid,:) = Ind_obj(i_obj,:);    
    
end

N_obj = N_obj_valid;
Ind_obj = Ind_obj_valid;






