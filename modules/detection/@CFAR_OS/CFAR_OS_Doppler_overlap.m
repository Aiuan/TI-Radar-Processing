function [N_obj, Ind_obj, noise_obj_an] = CFAR_OS_Doppler_overlap(obj, Ind_obj_Rag, sigCpml, sig_integ)
    % 本函数为OS CFAR多普勒维CFAR
    maxEnable = obj.maxEnable; %是否检测峰值最大状态
    cellNum0 = obj.refWinSize; %读取对象的参考窗长度
    gapNum0 = obj.guardWinSize; %读取对象的保护单元长度
    cellNum = cellNum0(2); %读取多普勒维的参考窗长度
    gapNum = gapNum0(2); %读取多普勒维的保护窗长度
    K0 = obj.K0(2); %多普勒维的门限系数

    rangeNumBins = size(sig_integ,1); %输入的sig为2D数据，分别表示每个chrip采样点数（距离维概念），Loop次数（阵列概念）

    detected_Rag_Cell = unique(Ind_obj_Rag(:,1)); %获取不重复的目标距离索引
    sig = sig_integ(detected_Rag_Cell,:); %统计所有目标的循环情况即radarcube的列向量

    M_samp = size(sig, 1); %获取采样点数，及距离维检测到目标的点数
    N_pul = size(sig, 2); %获取Loop次数

    gaptot = gapNum + cellNum; %参考窗

    N_obj = 0; %检测目标数初始化
    Ind_obj = []; %目标索引初始化
    noise_obj_an = []; 
    vec = zeros(1,N_pul+gaptot*2);
    for k = 1:M_samp %对每个目标点
        detected_Rag_Cell_i = detected_Rag_Cell(k); %获取第k个目标点的距离
        ind1 = find(Ind_obj_Rag(:,1) == detected_Rag_Cell_i);%第一次循环中目标位置与目标点的距离进行匹配
        indR = Ind_obj_Rag(ind1, 2); %返回索引

        sigv=(sig(k,:)); %获取每个目标距离对应的多普勒维信息
        vec(1:gaptot) = sigv(end-gaptot+1:end); %将右边界信息拷贝到左边界
        vec(gaptot+1: N_pul+gaptot) = sigv; %中间信息用sigv拷贝
        vec(N_pul+gaptot+1:end) = sigv(1:gaptot); %将左边界信息拷贝到右边界

        %CFAR处理
        ind_loc_all = []; %距离维索引矩阵
        ind_loc_Dop = []; %多普勒维索引矩阵
        ind_obj_0 = 0;
        noiseEst = zeros(1,N_pul); %噪声估计，长度为阵列数
        for j = 1+gaptot:N_pul+gaptot %去中间原始的数据进行处理
            cellInd = [j-gaptot:j-gapNum-1 j+gapNum+1:j+gaptot]; %获取参考窗
            noiseEst(j-gaptot) = sum(vec(cellInd)); %对参考窗内数据求和
        end
        for j = 1+gaptot:N_pul+gaptot
            j0 = j - gaptot;
            cellInd = [j-gaptot:j-gapNum-1 j+gapNum+1:j+gaptot];
            cellInda = [j-gaptot: j-gapNum-1]; %取左侧参考窗
            cellIndb =[j+gapNum+1:j+gaptot]; %取右侧参考窗

    %         cellave1a = sum(vec(cellInda))/(cellNum); %左侧参考窗多普勒维数据平均
    %         cellave1b = sum(vec(cellIndb))/(cellNum); %右侧参考窗多普勒维数据平均
    %         cellave1 = min(cellave1a,cellave1b); %两侧参考窗平均取最小        
            cell_k = sort(vec([cellInda,cellIndb]));
            cellave1 = cell_k(ceil(3/4*cellNum*2));
            maxInCell = max(vec(cellInd)); %获取峰值数据
            if maxEnable == 1 %检测该单元是否为参考窗内的峰值
                condition = ((vec(j)>K0*cellave1)) && ((vec(j)>maxInCell));
            else
                condition = vec(j)>K0*cellave1;
            end

            if condition == 1 %如果j单元是参考窗的峰值
                %检查该检测是否与多普勒检测重叠
                if(find(indR == j0))
                    %如果重叠则声明检测结果
                    ind_win = detected_Rag_Cell_i; %返回j0目标的多普勒维信息
                    ind_loc_all = [ind_loc_all ind_win]; 
                    ind_loc_Dop = [ind_loc_Dop j0]; %返回j0目标的索引，对应于距离维距离
                end
            end
        end
        ind_obj_0 = [];

        if (length(ind_loc_all)>0)
            ind_obj_0(:,1) = ((ind_loc_all)); %indobj第一列存放数据
            ind_obj_0(:,2) = ind_loc_Dop; %indobj第二列存放距离索引
            if size(Ind_obj,1) == 0 %如果没有数据则赋值给IndObj
                Ind_obj = ind_obj_0;
            else%否则
                %以下过程是为了避免重复的检测点
                ind_obj_0_sum = ind_loc_all + 10000 * ind_loc_Dop;
                Ind_obj_sum = Ind_obj(:,1) + 10000 * Ind_obj(:,2);
                for ii= 1: length(ind_loc_all)
                    if (length(find(Ind_obj_sum == ind_obj_0_sum(ii)))==0)
                        Ind_obj = [Ind_obj ; ind_obj_0(ii,:)];
                    end
                end
            end
        end
    end

    N_obj = size(Ind_obj,1);

    %重置参考窗口
    cellNum = cellNum0(1); %距离维窗口长度
    gapNum = gapNum0(1); %距离维保护单元
    gaptot = gapNum + cellNum;

    %获取每个阵列的噪声系数
    N_obj_valid = 0;
    Ind_obj_valid = [];
    for i_obj = 1:N_obj %对每个目标    
        ind_range = Ind_obj(i_obj,1);
        ind_Dop = Ind_obj(i_obj,2);
        %跳过信号功率小于obj.powerThre的检测点
        if (min(abs(sigCpml(ind_range, ind_Dop,:)).^2) < obj.powerThre)
            continue;
        end
        if ind_range <= gaptot%右边界
            cellInd = [ind_range+gapNum+1:ind_range+gaptot ind_range+gapNum+1:ind_range+gaptot];
        elseif ind_range >= rangeNumBins-gaptot+1%左边界
            cellInd = [ind_range-gaptot:ind_range-gapNum-1 ind_range-gaptot:ind_range-gapNum-1];
        else %中间边界
            cellInd = [ind_range-gaptot: ind_range-gapNum-1 ind_range+gapNum+1:ind_range+gaptot];
        end

        N_obj_valid = N_obj_valid +1;
        noise_obj_an(:, i_obj) = reshape((mean(abs(sigCpml(cellInd, ind_Dop, :)).^2, 1)), obj.numAntenna, 1, 1);
        Ind_obj_valid(N_obj_valid,:) = Ind_obj(i_obj,:);    

    end

    N_obj = N_obj_valid;
    Ind_obj = Ind_obj_valid;
end






