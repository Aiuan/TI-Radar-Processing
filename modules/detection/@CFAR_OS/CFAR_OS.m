%% 本程序主要定义OS CFAR的类

classdef CFAR_OS < Module
    % 定义成员变量
    properties (Access = public)
         detectMethod = 0% detection method choose to run
        numAntenna = 0 %number of antennas
        refWinSize = 0 %number of reference cells to estimate noise variance
        guardWinSize = 0 %number of gap cells to prevent leakage being detected as signal
        K0 = 0%detection threshold        
        maxEnable = 0  
        rangeBinSize = 0
        velocityBinSize = 0
        dopplerFFTSize = 0
        powerThre = 0
        discardCellLeft = 0
        discardCellRight = 0
        numRxAnt = 0         
        TDM_MIMO_numTX = 0
        antenna_azimuthonly = 0
        overlapAntenna_ID = 0
        overlapAntenna_ID_2TX = 0
        overlapAntenna_ID_3TX = 0
        applyVmaxExtend = 0
        minDisApplyVmaxExtend = 0
    end
    
    % 定义成员方法
    methods
        function obj = CFAR_OS(varargin)
             %如果输入列表与name匹配则不为空跳过，否则执行
            if(isempty(find(strcmp(varargin,'name'),1)))
                varargin = [varargin,'name','CFAR_OS'];
            end
            obj@Module(varargin{:});
            
            obj.enable = getParameter(obj, 'enable');           
            obj.detectMethod = getParameter(obj, 'detectMethod');           
            obj.numAntenna = getParameter(obj, 'numAntenna');            
            obj.refWinSize = getParameter(obj, 'refWinSize');            
            obj.guardWinSize = getParameter(obj, 'guardWinSize');            
            obj.K0 = getParameter(obj, 'K0');              
            obj.maxEnable = getParameter(obj, 'maxEnable');            
            obj.rangeBinSize = getParameter(obj, 'rangeBinSize');            
            obj.velocityBinSize = getParameter(obj, 'velocityBinSize');            
            obj.dopplerFFTSize = getParameter(obj, 'dopplerFFTSize');            
            obj.powerThre = getParameter(obj, 'powerThre');            
            obj.discardCellLeft = getParameter(obj, 'discardCellLeft');              
            obj.discardCellRight = getParameter(obj, 'discardCellRight');           
            obj.numRxAnt = getParameter(obj, 'numRxAnt');           
            obj.TDM_MIMO_numTX = getParameter(obj, 'TDM_MIMO_numTX');           
            obj.antenna_azimuthonly = getParameter(obj, 'antenna_azimuthonly');           
            obj.overlapAntenna_ID = getParameter(obj, 'overlapAntenna_ID');            
            obj.overlapAntenna_ID_2TX = getParameter(obj, 'overlapAntenna_ID_2TX');            
            obj.overlapAntenna_ID_3TX = getParameter(obj, 'overlapAntenna_ID_3TX');            
            obj.applyVmaxExtend = getParameter(obj, 'applyVmaxExtend');     
            obj.minDisApplyVmaxExtend = getParameter(obj, 'minDisApplyVmaxExtend');
            
            obj = set(obj, varargin{:});
        end
        [detection_results,N_obj,Ind_obj]  = datapath(obj, input);
    end
    methods (Access = protected)
        [N_obj,Ind_obj,noise_obj,noise_obj_an] = CFAR_OS_Doppler_overlap(obj,Ind_obj_Rag,sigCpml,sig_integ);
        [N_obj, Ind_obj, noise_obj, noise_obj_an] = CFAR_OS_Range(obj, sigCpml, sig);
    end
end