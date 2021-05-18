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

%% MODULE
%   obj = Module(modName, pfile) creates module with modName and
%   parameter file pFile.  Module is disabled by default unless a
%   <ModuleName>_enable parameter is present in the parameter file in which
%   case the internal deault state is set to the corrensponding parameter.
%   You may redirect the parameter file by adding a <modName>_parameterFile
%   parameter in the passed parameter file. 
%
%   par = get(obj, dataMember), get the value of a class property
%
%   par = getParameter(obj, varargin) returns a parameter named
%   <ModuleName>_<varargin(1)>_<varargin(2)>_...
%   it uses the parameter file directly, if that fails
%   it returns and error. 
%
%   obj = loadParameters(obj) load all the module parameters from the
%   parameter file to the internal parameter data base.
% 
%   out = listParameters(obj) lists all parameters of the object.
%   
%   Set the module name and parameter file and debugMode and enable 
%   obj = moduleInit(obj, varargin)       
%   
%   obj = set(obj, varargin), set variable name and value in pairs, support
%   multiple of parameter setting pairs
%
%   04/20/2016, Zigang Yang, adapted from Lei's module block.


%% Class definition
classdef Module < matlab.System

    properties (Access = 'public')
        debugMode = 0    % turn on/off debugMode
        enable = 0       % enable or disable the object
        name             % name of the object
        pfile            % parameter file name
        parameters       % all the module-related parameters in parameter file
    end

    methods
        function obj = Module(varargin)
            % Construct the bla class

            % global paramsFile;
            % If the parameter file not passed, use the global one
            % if ~any(strcmp(varargin,'pfile'))
            %    varargin = [varargin, 'pfile',{paramsFile}];
            % end

            % Set the module name and parameter file and debugMode and enable
            obj = moduleInit(obj, varargin{:});

            % Load parameters from the file 
            obj = loadParameters(obj);
       
        end
       
        %% list of all other methods for the Module class
        par = get(obj, varargin)
        par = getParameter(obj, varargin)
        out = listParameters(obj)
        obj = loadParameters(obj)  
        obj = moduleInit(obj, varargin)
        obj = set(obj, varargin)
    end

end

    