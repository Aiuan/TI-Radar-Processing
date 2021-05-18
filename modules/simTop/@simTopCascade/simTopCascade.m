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

%simTopCascade.m
%
%simTopCascade module definition. Contains some top level parameters. 

classdef simTopCascade < Module
    %% properties
    properties (Access = public)
        inputDataSource          = 'bin'; % choose from 'mat','bin','gen'
        outputDataSavingEnable   = 0;
        outputDataFileName       = [];        
        platform = [];
        totNumFrames = 0;
    end
    
    methods
        %% constructor
        function obj = simTopCascade(varargin)
            if(isempty(find(strcmp(varargin,'name'), 1)))
               varargin = [varargin, 'name','simTopCascade']; 
            end 
            obj@Module(varargin{:});
            
            obj.enable = getParameter(obj, 'enable');    
            obj.inputDataSource = getParameter(obj, 'inputDataSource');  
            obj.outputDataSavingEnable = getParameter(obj, 'outputDataSavingEnable');             
            obj.outputDataFileName = getParameter(obj, 'outputDataFileName');             
            obj.platform = getParameter(obj, 'platform');             
            obj.totNumFrames = getParameter(obj, 'totNumFrames');  
          
            
            % overwritten the property value inside parameter file
            %setProperties(obj, nargin, varargin{:});
            obj = set(obj, varargin{:});            

 
        end
        
        
        
        

    end        
end